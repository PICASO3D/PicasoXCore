//Copyright (c) 2021 Ultimaker B.V.
//Copyright (c) 2022 PICASO 3D
//PicasoXCore is released under the terms of the AGPLv3 or higher

#include "ZSeamConfig.h" //The definitions we're implementing.

namespace cura
{

ZSeamConfig::ZSeamConfig()
    : type(EZSeamType::SHORTEST)
    , pos(Point(0, 0))
    , corner_pref(EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE)
{}

ZSeamConfig::ZSeamConfig(const EZSeamType type, const Point pos, const EZSeamCornerPrefType corner_pref)
    : type(type)
    , pos(pos)
    , corner_pref(corner_pref)
{}

ZSeamCrossConfig::ZSeamCrossConfig(const SliceMeshStorage& mesh, const bool is_inset0)
    : is_inset0(is_inset0)
    , z_seam_cross(mesh.settings.get<EZSeamCross>(is_inset0 ? "z_seam_cross_0" : "z_seam_cross_1"))
    , start_flow(static_cast<float>(mesh.settings.get<Ratio>(is_inset0 ? "z_seam_cross_0_start_flow" : "z_seam_cross_1_start_flow")))
    , start_speed(static_cast<float>(mesh.settings.get<Ratio>(is_inset0 ? "z_seam_cross_0_start_speed" : "z_seam_cross_1_start_speed")))
    , start_distance(mesh.settings.get<coord_t>(is_inset0 ? "z_seam_cross_0_start_distance" : "z_seam_cross_1_start_distance"))
    , finish_flow(static_cast<float>(mesh.settings.get<Ratio>(is_inset0 ? "z_seam_cross_0_finish_flow" : "z_seam_cross_1_finish_flow")))
    , finish_speed(static_cast<float>(mesh.settings.get<Ratio>(is_inset0 ? "z_seam_cross_0_finish_speed" : "z_seam_cross_1_finish_speed")))
    , finish_distance(mesh.settings.get<coord_t>(is_inset0 ? "z_seam_cross_0_finish_distance" : "z_seam_cross_1_finish_distance"))
{}


ZSeamCrossing::ZSeamCrossing(const ConstPolygonRef& wall, const size_t start_idx)
    : pA(wall[(wall.size() + start_idx - 1) % wall.size()])
    , pB(wall[start_idx])
    , pC(wall[(start_idx + 1) % wall.size()])
{}

ZSeamCrossing::ZSeamCrossing(const ConstPolygonRef& wall, const size_t start_idx, const Point& p0)
    :pA(wall[(wall.size() + start_idx - 1) % wall.size()])
    , pB(p0)
    , pC(wall[start_idx])
{}

/*
*   c
*    \
*     \b
*   ,-`|
* d`   |
*      a
*/
bool ZSeamCrossing::FindCrossing_Mode_Normal(const ConstPolygonRef& wall, Point& point, coord_t point_start_distance, coord_t point_finish_distance, bool inside)
{
    float angle = LinearAlg2D::getAngleLeft(pA, pB, pC) / 2;
    Point d = RotateCustomLeft(pA, pB, angle);

    Point b_d = pB - d;
    coord_t r_bd = vSize(b_d);
    if (r_bd == 0)
    {
        return false;
    }

    coord_t mr = 100;
    // Forward normal
    const Point pD1 = pB + Point((coord_t)(mr * (float)b_d.X / r_bd), (coord_t)(mr * (float)b_d.Y / r_bd));
    // Reverse normal
    const Point pD2 = pB + Point((coord_t)(-mr * (float)b_d.X / r_bd), (coord_t)(-mr * (float)b_d.Y / r_bd));

    if (wall.inside(pD1, false) == inside)
    {
        mr = point_start_distance;
        point = pB + Point((coord_t)(mr * (float)b_d.X / r_bd), (coord_t)(mr * (float)b_d.Y / r_bd));
        return true;
    }

    if (wall.inside(pD2, false) == inside)
    {
        mr = point_finish_distance;
        point = pB + Point((coord_t)(-mr * (float)b_d.X / r_bd), (coord_t)(-mr * (float)b_d.Y / r_bd));
        return true;
    }

    return false;
}

/*
*   c
*    \
*(dbc)\b
*   ,-`|
* d`   |
* (abd)a
*/
bool ZSeamCrossing::FindCrossing_Mode_Custom(const ConstPolygonRef& wall, Point& point_start, Point& point_end, coord_t point_start_distance, coord_t point_finish_distance, float angle_divider, bool inside)
{
    float angle = LinearAlg2D::getAngleLeft(pA, pB, pC) / 2;
    Point d = RotateCustomLeft(pA, pB, angle);
    //Point d2 = RolAHalfLeft(pC, pB, pA);

    // and also half angle 
    angle = LinearAlg2D::getAngleLeft(d, pB, pC) / angle_divider;
    Point dbc = RotateCustomLeft(d, pB, -angle);

    angle = LinearAlg2D::getAngleLeft(pA, pB, d) / angle_divider;
    Point abd = RotateCustomLeft(d, pB, angle);

    Point b_d = pB - d;
    Point b_dbc = pB - dbc;
    Point b_abd = pB - abd;

    coord_t r_bd = vSize(b_d);
    coord_t r_dbc = vSize(b_dbc);
    coord_t r_abd = vSize(b_abd);

    if (r_bd == 0 || r_dbc == 0 || r_abd == 0)
    {
        return false;
    }

    coord_t mr = 100;
    // Forward normal
    const Point pD1 = pB + Point((coord_t)(mr * (float)b_d.X / r_bd), (coord_t)(mr * (float)b_d.Y / r_bd));
    // Reverse normal
    const Point pD2 = pB + Point((coord_t)(-mr * (float)b_d.X / r_bd), (coord_t)(-mr * (float)b_d.Y / r_bd));

    if (wall.inside(pD1, false) == inside)
    {
        coord_t mr_start = point_start_distance;
        coord_t mr_finish = point_finish_distance;
        point_start = pB + Point((coord_t)(mr_start * (float)b_dbc.X / r_dbc), (coord_t)(mr_start * (float)b_dbc.Y / r_dbc));
        point_end = pB + Point((coord_t)(mr_finish * (float)b_abd.X / r_abd), (coord_t)(mr_finish * (float)b_abd.Y / r_abd));
        return true;
    }

    if (wall.inside(pD2, false) == inside)
    {
        coord_t mr_start = point_start_distance;
        coord_t mr_finish = point_finish_distance;
        point_start = pB + Point((coord_t)(-mr_start * (float)b_dbc.X / r_dbc), (coord_t)(-mr_start * (float)b_dbc.Y / r_dbc));
        point_end = pB + Point((coord_t)(-mr_finish * (float)b_abd.X / r_abd), (coord_t)(-mr_finish * (float)b_abd.Y / r_abd));
        return true;
    }

    return false;
}

/*
*   result
*    \
*     \p0
* angle|
*      |
*      p1
*/
Point ZSeamCrossing::RotateCustomLeft(const Point& p1, const Point& p0, const float angle)
{
    float cos_a = cos(angle);
    float sin_a = sin(angle);
    Point ba = p1 - p0;
    Point result = p0 + Point((coord_t)(cos_a * ba.X + sin_a * ba.Y), (coord_t)(-sin_a * ba.X + cos_a * ba.Y));
    return result;
}

ZSeamIntersection::ZSeamIntersection(const ConstPolygonRef& wall, const int& start_idx)
    : wall(wall)
    , start_idx(start_idx)
{}

bool ZSeamIntersection::FindIntersection(const Point& p1, const Point& p2, int& ind_offset, Point& intersection)
{
    // optimization for high-poly mesh
    const size_t wall_size = wall.size();
    const size_t count = wall_size > 100 ? 50 : wall_size / 2;

    for (size_t offset = 0; offset < count; offset++)
    {
        // positive direction
        const Point& pA = wall[(wall_size + start_idx + offset - 1) % wall_size];
        const Point& pB = wall[(wall_size + start_idx + offset) % wall_size];

        if (ZSeamIntersection::FindIntersection(p1, p2, pA, pB, intersection))
        {
            ind_offset = offset;
            return true;
        }

        if (offset > 0)
        {
            // negative direction
            const Point& pA = wall[(wall_size + start_idx - offset - 1) % wall_size];
            const Point& pB = wall[(wall_size + start_idx - offset) % wall_size];

            if (ZSeamIntersection::FindIntersection(p1, p2, pA, pB, intersection))
            {
                ind_offset = -offset;
                return true;
            }
        }
    }

    return false;
}

/*
* Find intersection of 2 lines
*/
bool ZSeamIntersection::FindIntersection(const Point& start1, const Point& end1, const Point& start2, const Point& end2, Point& intersection)
{
    Point dir1 = end1 - start1;
    Point dir2 = end2 - start2;

    coord_t a1 = -dir1.Y;
    coord_t b1 = +dir1.X;
    coord_t d1 = -(a1 * start1.X + b1 * start1.Y);

    coord_t a2 = -dir2.Y;
    coord_t b2 = +dir2.X;
    coord_t d2 = -(a2 * start2.X + b2 * start2.Y);

    float seg1_line2_start = a2 * start1.X + b2 * start1.Y + d2;
    float seg1_line2_end = a2 * end1.X + b2 * end1.Y + d2;

    float seg2_line1_start = a1 * start2.X + b1 * start2.Y + d1;
    float seg2_line1_end = a1 * end2.X + b1 * end2.Y + d1;

    //no intersection
    if (seg1_line2_start * seg1_line2_end >= 0 || seg2_line1_start * seg2_line1_end >= 0)
        return false;

    float u = seg1_line2_start / (seg1_line2_start - seg1_line2_end);

    intersection = start1 + u * dir1;

    return true;
}

WallSeamCornerPoint::WallSeamCornerPoint(Point point, float flow_multiplier, float speed_multiplier)
    : point(point)
    , flow_multiplier(flow_multiplier)
    , speed_multiplier(speed_multiplier)
{}


WallSeamConfig::WallSeamConfig(int point_idx, Point point_center, EZSeamCross cross_cfg, EZSeamCornerPrefType corner_pref)
    : point_idx(point_idx)
    , point_center(point_center)
    , point_crossing_found(false)
    , start(WallSeamCornerPoint(Point(0, 0), 0.5, 1.0))
    , finish(WallSeamCornerPoint(Point(0, 0), 0.5, 1.0))
    , cross_cfg(cross_cfg)
    , corner_pref(corner_pref)
{}

WallSeamConfig::WallSeamConfig(int point_idx, Point point_center, WallSeamCornerPoint& start, WallSeamCornerPoint& finish, EZSeamCross cross_cfg, EZSeamCornerPrefType corner_pref)
    : point_idx(point_idx)
    , point_center(point_center)
    , point_crossing_found(true)
    , start(start)
    , finish(finish)
    , cross_cfg(cross_cfg)
    , corner_pref(corner_pref)
{}

}