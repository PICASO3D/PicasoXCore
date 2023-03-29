//Copyright (c) 2022 Ultimaker B.V.
//Copyright (c) 2022 PICASO 3D
//PicasoXCore is released under the terms of the AGPLv3 or higher

//#define debug_wc_log

#include "WallsComputation.h"
#include "Application.h"
#include "ExtruderTrain.h"
#include "Slice.h"
#include "WallToolPaths.h"
#include "settings/types/Ratio.h"
#include "settings/ZSeamConfig.h"
#include "sliceDataStorage.h"
#include "utils/polygonUtils.h"
#include "utils/Simplify.h" //We're simplifying the spiralized insets.
#include "PathOrderOptimizer.h"

namespace cura
{

WallsComputation::WallsComputation(const Settings& settings, const LayerIndex layer_nr) : settings(settings), layer_nr(layer_nr)
{
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * generateWalls only reads and writes data for the current layer
 */
void WallsComputation::generateWalls(SliceLayerPart* part)
{
    size_t wall_count = settings.get<size_t>("wall_line_count");
    if (wall_count == 0) // Early out if no walls are to be generated
    {
        part->print_outline = part->outline;
        part->inner_area = part->outline;
        return;
    }

    const bool spiralize = settings.get<bool>("magic_spiralize");
    const size_t alternate = ((layer_nr % 2) + 2) % 2;
    if (spiralize && layer_nr < LayerIndex(settings.get<size_t>("initial_bottom_layers")) && alternate == 1) //Add extra insets every 2 layers when spiralizing. This makes bottoms of cups watertight.
    {
        wall_count += 5;
    }
    if (settings.get<bool>("alternate_extra_perimeter"))
    {
        wall_count += alternate;
    }

    const EGCodeFlavor flavor = settings.get<EGCodeFlavor>("machine_gcode_flavor");

    const bool first_layer = layer_nr == 0;
    const Ratio line_width_0_factor = first_layer ? settings.get<ExtruderTrain&>("wall_0_extruder_nr").settings.get<Ratio>("initial_layer_line_width_factor") : 1.0_r;
    const coord_t line_width_0 = settings.get<coord_t>("wall_line_width_0") * line_width_0_factor;
    const coord_t wall_0_inset = settings.get<coord_t>("wall_0_inset");

    const Ratio line_width_1_factor = first_layer ? settings.get<ExtruderTrain&>("wall_1_extruder_nr").settings.get<Ratio>("initial_layer_line_width_factor") : 1.0_r;
    const coord_t line_width_1 = settings.get<coord_t>("wall_line_width_1") * line_width_1_factor;

    const Ratio line_width_x_factor = first_layer ? settings.get<ExtruderTrain&>("wall_x_extruder_nr").settings.get<Ratio>("initial_layer_line_width_factor") : 1.0_r;
    const coord_t line_width_x = settings.get<coord_t>("wall_line_width_x") * line_width_x_factor;

    // When spiralizing, generate the spiral insets using simple offsets instead of generating toolpaths
    if (spiralize)
    {
        const bool recompute_outline_based_on_outer_wall =
                settings.get<bool>("support_enable") && !settings.get<bool>("fill_outline_gaps");

        generateSpiralInsets(part, line_width_0, wall_0_inset, recompute_outline_based_on_outer_wall);
        if (layer_nr <= static_cast<LayerIndex>(settings.get<size_t>("bottom_layers")))
        {
            WallToolPaths wall_tool_paths(part->outline, line_width_0, line_width_1, line_width_x, wall_count, wall_0_inset, settings);
            if (flavor == EGCodeFlavor::PICASO)
            {
                WallsPicasoOverride wpo(settings, layer_nr);
                part->wall_toolpaths = wpo.getToolPaths(wall_tool_paths.getToolPaths());
            }
            else 
            {
                part->wall_toolpaths = wall_tool_paths.getToolPaths();
            }
            part->inner_area = wall_tool_paths.getInnerContour();
        }
    }
    else
    {
        WallToolPaths wall_tool_paths(part->outline, line_width_0, line_width_1, line_width_x, wall_count, wall_0_inset, settings);
        if (flavor == EGCodeFlavor::PICASO)
        {
            WallsPicasoOverride wpo(settings, layer_nr);
            part->wall_toolpaths = wpo.getToolPaths(wall_tool_paths.getToolPaths());
//#ifdef debug_wc_log
//            logWarning("generateWalls[%i]: (size: %i)\n", layer_nr, part->wall_toolpaths.size());
//#endif // debug_wc_log
        }
        else
        {
            part->wall_toolpaths = wall_tool_paths.getToolPaths();
        }
        part->inner_area = wall_tool_paths.getInnerContour();
    }
    part->print_outline = part->outline;
}

WallsPicasoOverride::WallsPicasoOverride(const Settings& settings, const LayerIndex layer_nr) 
    : settings(settings)
    , layer_nr(layer_nr)
{
}

const Point WallsPicasoOverride::getZSeamHint(const Settings& settings, const Point& center_of_mass)
{
    Point pos(settings.get<coord_t>("z_seam_x"), settings.get<coord_t>("z_seam_y"));
    if (settings.get<bool>("z_seam_relative"))
    {
        pos += Point(center_of_mass.X, center_of_mass.Y);
    }
    return pos;
}

const std::vector<VariableWidthLines>& WallsPicasoOverride::getToolPaths(const std::vector<VariableWidthLines>& src_toolpaths)
{
    if (src_toolpaths.empty())
    {
        return src_toolpaths;
    }

    toolpaths.resize(src_toolpaths.size());

    const bool debug_normals_enabled = settings.get<bool>("magic_debug_normals_enabled");
    const coord_t machine_nozzle_size = settings.get<coord_t>("machine_nozzle_size");
    const ZSeamCrossConfig z_seam_cross_config_0(settings, true);
    const ZSeamCrossConfig z_seam_cross_config_1(settings, false);

    const EZSeamType& seam_type = settings.get<EZSeamType>("z_seam_type");
    const size_t seam_skip_epsilon = 20*20;

    for (const VariableWidthLines& src_vwl : src_toolpaths)
    {
        VariableWidthLines& new_vwl = toolpaths.emplace_back();
        new_vwl.reserve(src_vwl.size());

        for (const ExtrusionLine& src_el : src_vwl) // every path from 0 inset
        {
            // proceed only closed external (0, 1) insets
            if (src_el.is_closed == false 
                || src_el.inset_idx > 1
                || src_el.size() < 2)
            {
                new_vwl.emplace_back(src_el);
                continue;
            }

            const ZSeamCrossConfig z_seam_cross_config = src_el.inset_idx == 0 ? z_seam_cross_config_0 : z_seam_cross_config_1;

            const Polygon& simple_poly = src_el.toPolygon();
            const Point& poly_center = simple_poly.centerOfMass();
            const bool wall_orientation = simple_poly.orientation();

            Point seam_hint = getZSeamHint(settings, poly_center);
            // seam bounding box angled position
            if (seam_type == EZSeamType::USER_SPECIFIED
                && settings.get<EZSeamUserSpecifiedMode>("z_seam_back_type") == EZSeamUserSpecifiedMode::BB_ANGLED)
            {
                AngleDegrees z_seam_bb_center_angle = settings.get<AngleDegrees>("z_seam_bb_center_angle");
                const AABB seam_aabb(simple_poly);
                const Point seam_middle = seam_aabb.getMiddle();
                double seam_rotate_angle = z_seam_bb_center_angle / 180.0 * M_PI;
                float seam_aabb_radius = vSizef(seam_aabb.max - seam_aabb.getMiddle());
                seam_hint = rotate(Point(seam_aabb_radius, 0), seam_rotate_angle) + seam_middle;
            }

            const std::string wall_line_width_name = src_el.inset_idx == 0 ? "wall_line_width_0" : (src_el.inset_idx == 1 ? "wall_line_width_1" : "wall_line_width_x");
            const ZSeamConfig seam_config(seam_type, seam_hint, settings.get<EZSeamCornerPrefType>("z_seam_corner"), settings.get<coord_t>(wall_line_width_name) * 2);

            // start_vertex for single ExtrusionLine
            PathOrderOptimizer<const ExtrusionLine*> order_optimizer(seam_config.pos, seam_config);
            const ExtrusionLine* line = &src_el;
            order_optimizer.addPolygon(line);
            //order_optimizer.optimize(); // no need
            auto path = order_optimizer.paths.front();
            path.converted = path.getVertexData();
            path.start_vertex = order_optimizer.findStartLocation(path, seam_config.pos);

            // just use start_vertex
            size_t best_point_idx = path.start_vertex;
            Point best_point = (*path.vertices)[path.start_vertex].p;
            size_t offset_prev_idx = 0;
            size_t offset_next_idx = 0;

            // Smooth point find as inset intersection
            if (seam_config.type == EZSeamType::USER_SPECIFIED && seam_config.corner_pref == EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_SMOOTH)
            {
                ZSeamIntersection seamIntersection(simple_poly, best_point_idx);
                size_t ind_offset = 0;
                size_t cross_distance = 0;

                if (seamIntersection.CheckCrossing(best_point, poly_center, seam_config.pos, cross_distance))
                {
                    offset_next_idx = 1;
                }
                else
                {
                    Point pIntersection;
                    if (seamIntersection.FindIntersection(poly_center, seam_config.pos, ind_offset, pIntersection))
                    {
                        best_point_idx = (src_el.size() + best_point_idx + ind_offset) % src_el.size();
                        best_point = pIntersection;

                        offset_next_idx = 1; // dont ask why, this just work
                        offset_prev_idx = 1; // dont ask why, this just work

#ifdef debug_wc_log
                        logWarning("getToolPaths.FindIntersection[%i]: %i\n", layer_nr, best_point_idx);
#endif // debug_wc_log
                    }
                    else
                    {
                        // check cross line
                        if (seamIntersection.FindPointCrossLine(poly_center, seam_config.pos, ind_offset, pIntersection, cross_distance))
                        {
                            best_point_idx = (src_el.size() + best_point_idx + ind_offset) % src_el.size();
                            best_point = pIntersection;
                            offset_next_idx = 1;
#ifdef debug_wc_log
                            logWarning("getToolPaths.FindPointCrossLine[%i]: %i\n", layer_nr, best_point_idx);
#endif // debug_wc_log
                        }
#ifdef debug_wc_log
                        else
                        {
                            logWarning("getToolPaths.FindPointCrossLine[%i]: can't find Smooth point\n", layer_nr);
                        }
#endif // debug_wc_log
                    }

                    const size_t idx_prev = (src_el.size() + best_point_idx - 1) % src_el.size();
                    const Point point_prev = (*path.vertices)[idx_prev].p;
                    const size_t dist_prev = vSize2(point_prev - pIntersection);
                    offset_prev_idx += (dist_prev < seam_skip_epsilon) ? 1 : 0;

                    const size_t idx_next = (src_el.size() + best_point_idx + 1) % src_el.size();
                    const Point point_next = (*path.vertices)[idx_next].p;
                    const size_t dist_next = vSize2(point_next - pIntersection);
                    offset_next_idx += (dist_next < seam_skip_epsilon) ? 1 : 0;

#ifdef debug_wc_log
                    logWarning("getToolPaths.PrevNext[%i]: prev(%i, %i), next(%i, %i)\n", layer_nr, dist_prev, offset_prev_idx, dist_next, offset_next_idx);
#endif // debug_wc_log
                }
            }
            else
            {
                const size_t idx_prev = (src_el.size() + best_point_idx - 1) % src_el.size();
                const Point point_prev = (*path.vertices)[idx_prev].p;
                offset_prev_idx = (vSize2(point_prev - best_point) < seam_skip_epsilon) ? 1 : 0;

                const size_t idx_next = (src_el.size() + best_point_idx + 1) % src_el.size();
                const Point point_next = (*path.vertices)[idx_next].p;
                offset_next_idx = (vSize2(point_next - best_point) < seam_skip_epsilon) ? 1 : 0;
            }


            // Calculate seam crossing
            const ConstPolygonRef& wall = simple_poly;
            const Point p1 = best_point; // center seam
            const size_t start_idx = (best_point_idx) % src_el.size();

            ZSeamCrossing seamCrossing(wall, start_idx, offset_prev_idx, offset_next_idx);
            if (seam_config.corner_pref == EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_SMOOTH)
            {
                seamCrossing = ZSeamCrossing(wall, start_idx, p1, offset_prev_idx, offset_next_idx);
#ifdef debug_wc_log
                logWarning("getToolPaths.corner_pref_smooth[%i]: %i\n", layer_nr, start_idx);
#endif // debug_wc_log
            }

		    // calculate seam cross config
            WallSeamConfig cross_config(start_idx, p1, z_seam_cross_config.z_seam_cross, seam_config.corner_pref);
            switch (z_seam_cross_config.z_seam_cross)
            {
            case EZSeamCross::NORMAL:
            {
                Point p0; // start & end
                if (seamCrossing.FindCrossing_Mode_Normal(wall, p0, 
                    z_seam_cross_config.start_distance, z_seam_cross_config.finish_distance, !wall_orientation))
                {
                    WallSeamCornerPoint point_start(p0, z_seam_cross_config.start_width);
                    WallSeamCornerPoint point_finish(p0, z_seam_cross_config.finish_width);

                    cross_config = WallSeamConfig(start_idx, p1, point_start, point_finish, z_seam_cross_config.z_seam_cross, seam_config.corner_pref);
                }
                else
                {
                    // try again with corner_pref = None
                    seamCrossing = ZSeamCrossing(wall, start_idx, offset_prev_idx, offset_next_idx);
                    if (seamCrossing.FindCrossing_Mode_Normal(
                          wall, p0, z_seam_cross_config.start_distance, z_seam_cross_config.finish_distance, ! wall_orientation))
                    {
                        WallSeamCornerPoint point_start(p0, z_seam_cross_config.start_width);
                        WallSeamCornerPoint point_finish(p0, z_seam_cross_config.finish_width);

                        cross_config = WallSeamConfig(start_idx, p1, point_start, point_finish, z_seam_cross_config.z_seam_cross, EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE);
                    }
#ifdef debug_wc_log
                    else
                    {
                        logWarning("getToolPaths.FindCrossing_Mode_Normal_false[%i]: %i\n", layer_nr, start_idx);
                    }
#endif // debug_wc_log
                }
            }
            break;

            case EZSeamCross::HALF:
            case EZSeamCross::QUARTER:
            {
                const float angle_divider = (z_seam_cross_config.z_seam_cross == EZSeamCross::HALF) ? 2.0 : 4.0;
                Point p0_start;
                Point p0_end;
                if (seamCrossing.FindCrossing_Mode_Custom(wall, p0_end, p0_start, 
                    z_seam_cross_config.start_distance, z_seam_cross_config.finish_distance, angle_divider, !wall_orientation))
                {
                    WallSeamCornerPoint point_start(p0_start, z_seam_cross_config.start_width);
                    WallSeamCornerPoint point_finish(p0_end, z_seam_cross_config.finish_width);

                    cross_config = WallSeamConfig(start_idx, p1, point_start, point_finish, z_seam_cross_config.z_seam_cross, seam_config.corner_pref);
                }
                else
                {
                    // try again with corner_pref = None
                    seamCrossing = ZSeamCrossing(wall, start_idx, offset_prev_idx, offset_next_idx);
                    if (seamCrossing.FindCrossing_Mode_Custom(wall, p0_end, p0_start, 
                          z_seam_cross_config.start_distance, z_seam_cross_config.finish_distance, angle_divider, !wall_orientation))
                    {
                        WallSeamCornerPoint point_start(p0_start, z_seam_cross_config.start_width);
                        WallSeamCornerPoint point_finish(p0_end, z_seam_cross_config.finish_width);

                        cross_config = WallSeamConfig(start_idx, p1, point_start, point_finish, z_seam_cross_config.z_seam_cross, EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE);
                    }
#ifdef debug_wc_log
                    else
                    {
                        logWarning("getToolPaths.FindCrossing_Mode_Custom_false[%i]: %i\n", layer_nr, start_idx);
                    }
#endif // debug_wc_log
                }
            }
            break;

            default:
                break;
            }

            ExtrusionLine new_el;
            new_el.inset_idx = src_el.inset_idx;
            new_el.is_odd = src_el.is_odd;
            new_el.is_closed = src_el.is_closed;
            new_el.seam_idx = -1;

            // RANDOM and SHORTEST will override start points, this why it impossible use seam_cross
            if (seam_config.type == EZSeamType::USER_SPECIFIED 
                || seam_config.type == EZSeamType::SHARPEST_CORNER)
            {
                const ExtrusionJunction& best_ej = src_el[best_point_idx]; // best seam point index

                if (cross_config.point_crossing_found && cross_config.cross_cfg != EZSeamCross::NONE)
                {
                    size_t start_width = static_cast<size_t>(best_ej.w * cross_config.start.width_multiplier);
                    ExtrusionJunction ej_start(cross_config.start.point, start_width, best_ej.perimeter_index);
                    ej_start.pcf = PathConfigFeature::Inset0SeamCrossStart;
                    new_el.emplace_back(ej_start);
                }

                if (cross_config.corner_pref == EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_SMOOTH)
                {
                    ExtrusionJunction ej_center(best_point, best_ej.w, best_ej.perimeter_index);
                    new_el.emplace_back(ej_center);
                }

                const size_t start_idx = (best_point_idx) % src_el.size();
                const size_t end_idx = src_el.size() + start_idx;

                // rebuild all other wall from best_idx
                for (size_t i = start_idx; i < end_idx; ++i)
                {
                    const int here_idx = i % src_el.size();
                    new_el.emplace_back(src_el[here_idx]);
                }

                if (cross_config.point_crossing_found && cross_config.cross_cfg != EZSeamCross::NONE)
                {
                    if (cross_config.corner_pref == EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_SMOOTH)
                    {
                        ExtrusionJunction ej_center(best_point, best_ej.w, best_ej.perimeter_index);
                        new_el.emplace_back(ej_center);
                    }
                    else
                    {
                        new_el.emplace_back(src_el[start_idx]);
                    }

                    size_t finish_width = static_cast<size_t>(best_ej.w * cross_config.finish.width_multiplier);
                    ExtrusionJunction ej_finish(cross_config.finish.point, finish_width, best_ej.perimeter_index);
                    ej_finish.pcf = PathConfigFeature::Inset0SeamCrossFinish;
                    new_el.emplace_back(ej_finish);
                }

                new_el.seam_idx = 0; // already reordered all points

                if (cross_config.point_crossing_found && cross_config.cross_cfg != EZSeamCross::NONE)
                {
                    new_el.unlink_last_point = true;
                }

#ifdef debug_wc_log
                logWarning("getToolPaths.RebuildSeam[%i]: true\n", layer_nr);
#endif // debug_wc_log
                new_vwl.emplace_back(new_el);
            }
            else
            {
#ifdef debug_wc_log
                logWarning("getToolPaths.RebuildSeam[%i]: false\n", layer_nr);
#endif // debug_wc_log
                new_vwl.emplace_back(src_el);
            }

            // show needle normals (not for printing, debug only, only 0 inset)
            if (debug_normals_enabled && src_el.inset_idx == 0)
            {
                for (size_t ind = 0; ind < simple_poly.size(); ind++)
                {
                    const Point& p1 = simple_poly[ind];

                    Point p0;
                    ZSeamCrossing seamCrossing(simple_poly, ind, 0, 0);
                    if (seamCrossing.FindCrossing_Mode_Normal(simple_poly, p0, machine_nozzle_size, machine_nozzle_size, wall_orientation))
                    {
                        // single needle p0-p1
                        ExtrusionLine el_needle(1, true);
                        el_needle.junctions.emplace_back(p0, machine_nozzle_size / 10, 0);
                        el_needle.junctions.emplace_back(p1, machine_nozzle_size / 10, 0);
                        el_needle.seam_idx = 0;
                        new_vwl.emplace_back(el_needle);
                    }
                }
            }

        }
    }

    return toolpaths;
}

/*
 * This function is executed in a parallel region based on layer_nr.
 * When modifying make sure any changes does not introduce data races.
 *
 * generateWalls only reads and writes data for the current layer
 */
void WallsComputation::generateWalls(SliceLayer* layer)
{
    for(SliceLayerPart& part : layer->parts)
    {
        generateWalls(&part);
    }

    //Remove the parts which did not generate a wall. As these parts are too small to print,
    // and later code can now assume that there is always minimal 1 wall line.
    if(settings.get<size_t>("wall_line_count") >= 1 && !settings.get<bool>("fill_outline_gaps"))
    {
        for(size_t part_idx = 0; part_idx < layer->parts.size(); part_idx++)
        {
            if (layer->parts[part_idx].wall_toolpaths.empty() && layer->parts[part_idx].spiral_wall.empty())
            {
                if (part_idx != layer->parts.size() - 1)
                { // move existing part into part to be deleted
                    layer->parts[part_idx] = std::move(layer->parts.back());
                }
                layer->parts.pop_back(); // always remove last element from array (is more efficient)
                part_idx -= 1; // check the part we just moved here
            }
        }
    }
}

void WallsComputation::generateSpiralInsets(SliceLayerPart *part, coord_t line_width_0, coord_t wall_0_inset, bool recompute_outline_based_on_outer_wall)
{
    part->spiral_wall = part->outline.offset(-line_width_0 / 2 - wall_0_inset);

    //Optimize the wall. This prevents buffer underruns in the printer firmware, and reduces processing time in PicasoXCore.
    const ExtruderTrain& train_wall = settings.get<ExtruderTrain&>("wall_0_extruder_nr");
    part->spiral_wall = Simplify(train_wall.settings).polygon(part->spiral_wall);
    part->spiral_wall.removeDegenerateVerts();
    if (recompute_outline_based_on_outer_wall)
    {
        part->print_outline = part->spiral_wall.offset(line_width_0 / 2, ClipperLib::jtSquare);
    }
    else
    {
        part->print_outline = part->outline;
    }
}

}//namespace cura
