//Copyright (c) 2022 Ultimaker B.V.
//Copyright (c) 2022 PICASO 3D
//PicasoXCore is released under the terms of the AGPLv3 or higher

#include "InsetOrderOptimizer.h"
#include "ExtruderTrain.h"
#include "FffGcodeWriter.h"
#include "LayerPlan.h"
#include "WallToolPaths.h"

#include <iterator>

namespace cura
{

InsetOrderOptimizer::InsetOrderOptimizer(const FffGcodeWriter& gcode_writer,
                                         const SliceDataStorage& storage,
                                         LayerPlan& gcode_layer,
                                         const Settings& settings,
                                         const int extruder_nr,
                                         const GCodePathConfig& inset_0_non_bridge_config,
                                         const GCodePathConfig& inset_1_non_bridge_config,
                                         const GCodePathConfig& inset_X_non_bridge_config,
                                         const GCodePathConfig& inset_0_bridge_config,
                                         const GCodePathConfig& inset_1_bridge_config,
                                         const GCodePathConfig& inset_X_bridge_config,
                                         const GCodePathConfig& inset_0_seam_cross_start_config,
                                         const GCodePathConfig& inset_0_seam_cross_finish_config,
                                         const bool retract_before_outer_wall,
                                         const coord_t wall_0_wipe_dist,
                                         const coord_t wall_x_wipe_dist,
                                         const size_t wall_0_extruder_nr,
                                         const size_t wall_1_extruder_nr,
                                         const size_t wall_x_extruder_nr,
                                         const ZSeamConfig& z_seam_config,
                                         const std::vector<VariableWidthLines>& paths)
    : gcode_writer(gcode_writer)
    , storage(storage)
    , gcode_layer(gcode_layer)
    , settings(settings)
    , extruder_nr(extruder_nr)
    , inset_0_non_bridge_config(inset_0_non_bridge_config)
    , inset_1_non_bridge_config(inset_1_non_bridge_config)
    , inset_X_non_bridge_config(inset_X_non_bridge_config)
    , inset_0_bridge_config(inset_0_bridge_config)
    , inset_1_bridge_config(inset_1_bridge_config)
    , inset_X_bridge_config(inset_X_bridge_config)
    , inset_0_seam_cross_start_config(inset_0_seam_cross_start_config)
    , inset_0_seam_cross_finish_config(inset_0_seam_cross_finish_config)
    , retract_before_outer_wall(retract_before_outer_wall)
    , wall_0_wipe_dist(wall_0_wipe_dist)
    , wall_x_wipe_dist(wall_x_wipe_dist)
    , wall_0_extruder_nr(wall_0_extruder_nr)
    , wall_1_extruder_nr(wall_1_extruder_nr)
    , wall_x_extruder_nr(wall_x_extruder_nr)
    , z_seam_config(z_seam_config)
    , paths(paths)
    , layer_nr(gcode_layer.getLayerNr())
    , added_something(false)
    , retraction_region_calculated(false)
{
}

bool InsetOrderOptimizer::addToLayer()
{
    // Settings & configs:
    const auto flavor = settings.get<EGCodeFlavor>("machine_gcode_flavor"); // PicasoAddon
    const bool is_picaso = flavor == EGCodeFlavor::PICASO;

    const auto pack_by_inset = ! settings.get<bool>("optimize_wall_printing_order");
    const auto inset_direction = settings.get<InsetDirection>("inset_direction");
    const auto alternate_walls = settings.get<bool>("material_alternate_walls");

    const bool outer_to_inner = inset_direction == InsetDirection::OUTSIDE_IN || inset_direction == InsetDirection::OREDER_10_IN;
    const bool use_one_extruder = wall_0_extruder_nr == wall_x_extruder_nr && wall_0_extruder_nr == wall_1_extruder_nr;
    const bool current_extruder_is_wall_x = wall_x_extruder_nr == extruder_nr;
    const auto inset_order_10_join_grid = (flavor == EGCodeFlavor::PICASO && inset_direction == InsetDirection::OREDER_10_IN)
        ? settings.get<bool>("inset_order_10_join_grid")
        : false;

    const auto should_reverse = [&]()
    {
        if (use_one_extruder && current_extruder_is_wall_x)
        {
            // The entire wall is printed with the current extruder.
            // Reversing the insets now depends on the inverse of the inset direction.
            // If we want to print the outer insets first we start with the lowest and move forward
            // otherwise we start with the highest and iterate back.
            return ! outer_to_inner;
        }
        // If the wall is partially printed with the current extruder we need to move forward
        // for the outer wall extruder and iterate back for the inner wall extruder
        return current_extruder_is_wall_x;
    }; // Helper lambda to ensure that the reverse bool can be a const type
    const bool reverse = should_reverse();

    // Switches the begin()...end() forward iterator for a rbegin()...rend() reverse iterator
    // I can't wait till we use the C++20 standard and have access to ranges and views
    const auto get_walls_to_be_added = [&](const bool reverse, const std::vector<VariableWidthLines>& paths)
    {
        if (paths.empty())
        {
            return std::vector<const ExtrusionLine*>{};
        }
        if (is_picaso)
        {
            return wallsToBeAdded(paths.begin(), paths.end());
        }
        else
        {
            if (reverse)
            {
                if (use_one_extruder)
                {
                    return wallsToBeAdded(paths.rbegin(), paths.rend()); // Complete wall with one extruder
                }
                return wallsToBeAdded(paths.rbegin(), std::prev(paths.rend())); // Ignore inner wall
            }
            if (use_one_extruder)
            {
                return wallsToBeAdded(paths.begin(), paths.end()); // Complete wall with one extruder
            }
            return wallsToBeAdded(paths.begin(), std::next(paths.begin())); // Ignore outer wall
        }
    };
    const auto walls_to_be_added = get_walls_to_be_added(reverse, paths);

    const auto order = 
        pack_by_inset
            ? (inset_direction == InsetDirection::OREDER_10_IN
                ? (inset_order_10_join_grid 
                    ? getOrder10In_GridOrder(walls_to_be_added) 
                    : getOrder10In_InsetOrder(walls_to_be_added))
                : getInsetOrder(walls_to_be_added, outer_to_inner))
            : getRegionOrder(walls_to_be_added, outer_to_inner);
    
    constexpr Ratio flow = 1.0_r;

    bool added_something = false;

    constexpr bool detect_loops = false;
    constexpr Polygons* combing_boundary = nullptr;
    // When we alternate walls, also alternate the direction at which the first wall starts in.
    // On even layers we start with normal direction, on odd layers with inverted direction.
    constexpr bool reverse_all_paths = false;
    PathOrderOptimizer<const ExtrusionLine*> order_optimizer(gcode_layer.getLastPlannedPositionOrStartingPosition(), z_seam_config, detect_loops, combing_boundary, reverse_all_paths, order);

    for (const ExtrusionLine* line : walls_to_be_added)
    {
        if (line->is_closed)
        {
            order_optimizer.addPolygon(line, line->seam_idx);
        }
        else
        {
            order_optimizer.addPolyline(line);
        }
    }


    order_optimizer.optimize();

    cura::Point p_end{ 0, 0 };
    for (const PathOrderPath<const ExtrusionLine*>& path : order_optimizer.paths)
    {
        if (path.vertices->empty())
		    continue;

        // or thin wall 'gap filler'
        const auto inset_idx = path.vertices->inset_idx;
        const auto inset_extruder_nr =
          is_picaso 
            ? (inset_idx == 0 
                ? wall_0_extruder_nr 
                : (inset_idx == 1 
                    ? wall_1_extruder_nr 
                    : wall_x_extruder_nr)) 
            : extruder_nr;

        const bool is_outer_wall = is_picaso ? (inset_idx < 2) : (inset_idx == 0);
        const bool is_gap_filler = path.vertices->is_odd;
        const GCodePathConfig& non_bridge_config = 
            is_outer_wall 
                ? ((is_picaso && inset_idx == 1) 
                    ? inset_1_non_bridge_config 
                    : inset_0_non_bridge_config)
                : inset_X_non_bridge_config;
        const GCodePathConfig& bridge_config =
            is_outer_wall 
                ? ((is_picaso && inset_idx == 1) 
                    ? inset_1_bridge_config 
                    : inset_0_bridge_config) 
                : inset_X_bridge_config;
        const coord_t wipe_dist = is_outer_wall && ! is_gap_filler ? wall_0_wipe_dist : wall_x_wipe_dist;
        const bool retract_before = inset_order_10_join_grid 
            ? (inset_idx == 1) 
            : (is_outer_wall 
                ? retract_before_outer_wall 
                : false);

        const bool revert_inset = alternate_walls && (path.vertices->inset_idx % 2);
        const bool revert_layer = alternate_walls && (layer_nr % 2);
        const bool backwards = path.backwards != (revert_inset != revert_layer);
        const size_t start_index = (backwards != path.backwards) ? path.vertices->size() - (path.start_vertex + 1) : path.start_vertex;

        p_end = path.backwards ? path.vertices->back().p : path.vertices->front().p;
        const cura::Point p_start = path.backwards ? path.vertices->front().p : path.vertices->back().p;
        const bool linked_path = p_start != p_end;

        gcode_writer.setExtruder_addPrime(storage, gcode_layer, inset_extruder_nr);
        gcode_layer.setIsInside(true); //Going to print walls, which are always inside.
        gcode_layer.addWall(*path.vertices,
                            start_index,
                            settings,
                            non_bridge_config,
                            bridge_config,
                            inset_0_seam_cross_start_config,
                            inset_0_seam_cross_finish_config,
                            wipe_dist,
                            flow,
                            retract_before,
                            path.is_closed,
                            path.unlink_last_point,
                            backwards,
                            linked_path);
        added_something = true;
    }
    return added_something;
}


std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> InsetOrderOptimizer::getRegionOrder(const std::vector<const ExtrusionLine*>& input, const bool outer_to_inner)
{
    std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> order_requirements;

    // We build a grid where we map toolpath vertex locations to toolpaths,
    // so that we can easily find which two toolpaths are next to each other,
    // which is the requirement for there to be an order constraint.
    // 
    // We use a PointGrid rather than a LineGrid to save on computation time.
    // In very rare cases two insets might lie next to each other without having neighboring vertices, e.g.
    //  \            .
    //   |  /        .
    //   | /         .
    //   ||          .
    //   | \         .
    //   |  \        .
    //  /            .
    // However, because of how Arachne works this will likely never be the case for two consecutive insets.
    // On the other hand one could imagine that two consecutive insets of a very large circle
    // could be simplify()ed such that the remaining vertices of the two insets don't align.
    // In those cases the order requirement is not captured,
    // which means that the PathOrderOptimizer *might* result in a violation of the user set path order.
    // This problem is expected to be not so severe and happen very sparsely.

    coord_t max_line_w = 0u;
    for (const ExtrusionLine* line : input)
    { // compute max_line_w
        for (const ExtrusionJunction& junction : *line)
        {
            max_line_w = std::max(max_line_w, junction.w);
        }
    }
    if (max_line_w == 0u) return order_requirements;
    
    struct LineLoc
    {
        ExtrusionJunction j;
        const ExtrusionLine* line;
    };
    struct Locator
    {
        Point operator()(const LineLoc& elem)
        {
            return elem.j.p;
        }
    };
    
    // How much farther two verts may be apart due to corners.
    // This distance must be smaller than 2, because otherwise
    // we could create an order requirement between e.g.
    // wall 2 of one region and wall 3 of another region,
    // while another wall 3 of the first region would lie in between those two walls.
    // However, higher values are better against the limitations of using a PointGrid rather than a LineGrid.
    constexpr float diagonal_extension = 1.9;
    const coord_t searching_radius = max_line_w * diagonal_extension;
    using GridT = SparsePointGrid<LineLoc, Locator>;
    GridT grid(searching_radius);

    
    for (const ExtrusionLine* line : input)
    {
        for (const ExtrusionJunction& junction : *line)
        {
            grid.insert(LineLoc{junction, line});
        }
    }
    for (const std::pair<SquareGrid::GridPoint, LineLoc>& pair : grid)
    {
        const LineLoc& lineloc_here = pair.second;
        const ExtrusionLine* here = lineloc_here.line;
        Point loc_here = pair.second.j.p;
        std::vector<LineLoc> nearby_verts = grid.getNearby(loc_here, searching_radius);
        for (const LineLoc& lineloc_nearby : nearby_verts)
        {
            const ExtrusionLine* nearby = lineloc_nearby.line;
            if (nearby == here) continue;
            if (nearby->inset_idx == here->inset_idx) continue;
            if (nearby->inset_idx > here->inset_idx + 1) continue; // not directly adjacent
            if (here->inset_idx > nearby->inset_idx + 1) continue; // not directly adjacent
            if ( ! shorterThan(loc_here - lineloc_nearby.j.p, (lineloc_here.j.w + lineloc_nearby.j.w) / 2 * diagonal_extension)) continue; // points are too far away from each other
            if (here->is_odd || nearby->is_odd)
            {
                if (here->is_odd && ! nearby->is_odd && nearby->inset_idx < here->inset_idx)
                {
                    order_requirements.emplace(std::make_pair(nearby, here));
                }
                if (nearby->is_odd && ! here->is_odd && here->inset_idx < nearby->inset_idx)
                {
                    order_requirements.emplace(std::make_pair(here, nearby));
                }
            }
            else if ((nearby->inset_idx < here->inset_idx) == outer_to_inner)
            {
                order_requirements.emplace(std::make_pair(nearby, here));
            }
            else
            {
                assert((nearby->inset_idx > here->inset_idx) == outer_to_inner);
                order_requirements.emplace(std::make_pair(here, nearby));
            }
        }
    }
    return order_requirements;
}

std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> InsetOrderOptimizer::getInsetOrder(const std::vector<const ExtrusionLine*>& input, const bool outer_to_inner)
{
    std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> order;
    
    std::vector<std::vector<const ExtrusionLine*>> walls_by_inset;
    std::vector<std::vector<const ExtrusionLine*>> fillers_by_inset;

    for (const ExtrusionLine* line : input)
    {
        if (line->is_odd)
        {
            if (line->inset_idx >= fillers_by_inset.size())
            {
                fillers_by_inset.resize(line->inset_idx + 1);
            }
            fillers_by_inset[line->inset_idx].emplace_back(line);
        }
        else
        {
            if (line->inset_idx >= walls_by_inset.size())
            {
                walls_by_inset.resize(line->inset_idx + 1);
            }
            walls_by_inset[line->inset_idx].emplace_back(line);
        }
    }
    for (size_t inset_idx = 0; inset_idx + 1 < walls_by_inset.size(); inset_idx++)
    {
        for (const ExtrusionLine* line : walls_by_inset[inset_idx])
        {
            for (const ExtrusionLine* inner_line : walls_by_inset[inset_idx + 1])
            {
                const ExtrusionLine* before = inner_line;
                const ExtrusionLine* after = line;

                if (outer_to_inner)
                {
                    std::swap(before, after);
                }

                order.emplace(before, after);
            }
        }
    }
    for (size_t inset_idx = 1; inset_idx < fillers_by_inset.size(); inset_idx++)
    {
        for (const ExtrusionLine* line : fillers_by_inset[inset_idx])
        {
            if (inset_idx - 1 >= walls_by_inset.size())
                continue;
            for (const ExtrusionLine* enclosing_wall : walls_by_inset[inset_idx - 1])
            {
                order.emplace(enclosing_wall, line);
            }
        }
    }
    
    return order;
}

std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> InsetOrderOptimizer::getOrder10In_InsetOrder(const std::vector<const ExtrusionLine*>& input)
{
    std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> order;

    std::vector<std::vector<const ExtrusionLine*>> polygons_by_inset;
    std::vector<std::vector<const ExtrusionLine*>> polylines_by_inset;

    size_t max_inset_idx = 0;

    for (const ExtrusionLine* line : input)
    {
        if (line->inset_idx >= polygons_by_inset.size())
        {
            polygons_by_inset.resize(line->inset_idx + 1);
            polylines_by_inset.resize(line->inset_idx + 1);
        }

        if (line->is_closed)
        {
            polygons_by_inset[line->inset_idx].emplace_back(line);
        }
        else
        {
            polylines_by_inset[line->inset_idx].emplace_back(line);
        }

        max_inset_idx = std::max(max_inset_idx, line->inset_idx);
    }

    for (size_t inset_idx = 0; inset_idx < max_inset_idx; inset_idx++)
    {
        size_t before_inset_idx = inset_idx;
        size_t after_inset_idx = inset_idx + 1;

        switch (before_inset_idx)
        {
        case 0:
            before_inset_idx = 1;
            after_inset_idx = 0;
            break;

        case 1:
            before_inset_idx = 0;
            after_inset_idx = 2;
            break;
        }

        for (const ExtrusionLine* path_first : polygons_by_inset[before_inset_idx])
        {
            for (const ExtrusionLine* path_second : polygons_by_inset[after_inset_idx])
            {
                order.emplace(path_first, path_second);
            }
        }
        for (const ExtrusionLine* line_second : polylines_by_inset[after_inset_idx])
        {
            for (const ExtrusionLine* path_first : polygons_by_inset[before_inset_idx])
            {
                order.emplace(path_first, line_second);
            }
        }
    }

    return order;
}

std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> InsetOrderOptimizer::getOrder10In_GridOrder(const std::vector<const ExtrusionLine*>& input)
{
    std::unordered_set<std::pair<const ExtrusionLine*, const ExtrusionLine*>> order_requirements;

    // We build a grid where we map toolpath vertex locations to toolpaths,
    // so that we can easily find which two toolpaths are next to each other,
    // which is the requirement for there to be an order constraint.
    //
    // We use a PointGrid rather than a LineGrid to save on computation time.
    // In very rare cases two insets might lie next to each other without having neighboring vertices, e.g.
    //  \            .
    //   |  /        .
    //   | /         .
    //   ||          .
    //   | \         .
    //   |  \        .
    //  /            .
    // However, because of how Arachne works this will likely never be the case for two consecutive insets.
    // On the other hand one could imagine that two consecutive insets of a very large circle
    // could be simplify()ed such that the remaining vertices of the two insets don't align.
    // In those cases the order requirement is not captured,
    // which means that the PathOrderOptimizer *might* result in a violation of the user set path order.
    // This problem is expected to be not so severe and happen very sparsely.

    coord_t max_line_w = 0u;
    for (const ExtrusionLine* line : input)
    { // compute max_line_w
        for (const ExtrusionJunction& junction : *line)
        {
            max_line_w = std::max(max_line_w, junction.w);
        }
    }
    if (max_line_w == 0u)
        return order_requirements;

    struct LineLoc
    {
        ExtrusionJunction j;
        const ExtrusionLine* line;
    };
    struct Locator
    {
        Point operator()(const LineLoc& elem)
        {
            return elem.j.p;
        }
    };

    // How much farther two verts may be apart due to corners.
    // This distance must be smaller than 2, because otherwise
    // we could create an order requirement between e.g.
    // wall 2 of one region and wall 3 of another region,
    // while another wall 3 of the first region would lie in between those two walls.
    // However, higher values are better against the limitations of using a PointGrid rather than a LineGrid.
    constexpr float diagonal_extension = 2.9;
    const coord_t searching_radius = max_line_w * diagonal_extension;
    using GridT = SparsePointGrid<LineLoc, Locator>;
    GridT grid(searching_radius);

    bool outer_to_inner = true;

    for (const ExtrusionLine* line : input)
    {
        for (const ExtrusionJunction& junction : *line)
        {
            grid.insert(LineLoc{ junction, line });
        }
    }
    for (const std::pair<SquareGrid::GridPoint, LineLoc>& pair : grid)
    {
        const LineLoc& lineloc_here = pair.second;
        const ExtrusionLine* here = lineloc_here.line;
        Point loc_here = pair.second.j.p;
        std::vector<LineLoc> nearby_verts = grid.getNearby(loc_here, searching_radius);
        for (const LineLoc& lineloc_nearby : nearby_verts)
        {
            const ExtrusionLine* nearby = lineloc_nearby.line;
            if (nearby == here)
                continue;
            if (nearby->inset_idx == here->inset_idx)
                continue;
            if (nearby->inset_idx > here->inset_idx + 2)
                continue; // not directly adjacent
            if (here->inset_idx > nearby->inset_idx + 2)
                continue; // not directly adjacent
            if (! shorterThan(loc_here - lineloc_nearby.j.p, (lineloc_here.j.w + lineloc_nearby.j.w) / 2 * diagonal_extension))
                continue; // points are too far away from each other

            if (here->inset_idx == 1 && nearby->inset_idx == 0)
            {
                order_requirements.emplace(here, nearby);
                continue;
            }
            if (here->inset_idx == 0 && nearby->inset_idx == 2)
            {
                order_requirements.emplace(here, nearby);
                continue;
            }
            if (here->inset_idx > 1 && nearby->inset_idx > 2 && here->inset_idx < nearby->inset_idx)
            {
                order_requirements.emplace(here, nearby);
                continue;
            }

            if (nearby->inset_idx == 1 && here->inset_idx == 0)
            {
                order_requirements.emplace(nearby, here);
                continue;
            }
            if (nearby->inset_idx == 0 && here->inset_idx == 2)
            {
                order_requirements.emplace(nearby, here);
                continue;
            }
            if (nearby->inset_idx > 1 && here->inset_idx > 2 && nearby->inset_idx < here->inset_idx)
            {
                order_requirements.emplace(nearby, here);
                continue;
            }
        }
    }
    return order_requirements;
}


} // namespace cura
