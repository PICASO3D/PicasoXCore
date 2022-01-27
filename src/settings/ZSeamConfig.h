//Copyright (c) 2021 Ultimaker B.V.
//Copyright (c) 2022 PICASO 3D
//PicasoXCore is released under the terms of the AGPLv3 or higher

#ifndef ZSEAMCONFIG_H
#define ZSEAMCONFIG_H

#include "EnumSettings.h" //For the seam type and corner preference settings.
#include "../utils/IntPoint.h" //For Point.
#include "../utils/linearAlg2D.h"
#include "../sliceDataStorage.h"
#include "types/Ratio.h"

namespace cura
{

/*!
 * Helper class that encapsulates various criteria that define the location of
 * the seam.
 *
 * Instances of this are passed to the order optimizer to specify where the seam
 * is to be placed.
 */
struct ZSeamConfig
{
    /*!
     * Strategy to place the seam (user-specified, shortest distance, sharpest
     * corner, etc.)
     */
    EZSeamType type;

    /*!
     * When using a user-specified position for the seam, this is the position
     * that the user specified.
     */
    Point pos;

    /*!
     * Corner preference type, applicable to various strategies to filter on
     * which corners the seam is allowed to be located.
     */
    EZSeamCornerPrefType corner_pref;

    /*!
     * Default constructor for use when memory must be allocated before it gets
     * filled (like with some data structures).
     *
     * This will select the "shortest" seam strategy.
     */
    ZSeamConfig();

    /*!
     * Create a seam configuration with a custom configuration.
     * \param type The strategy to place the seam.
     * \param pos The position of a user-specified seam.
     * \param corner_pref The corner preference, applicable to some strategies.
     */
    ZSeamConfig(const EZSeamType type, const Point pos, const EZSeamCornerPrefType corner_pref);
};

/*
 * Config seam cross preference for inset0/inset1
 */
class ZSeamCrossConfig
{
public:
    const bool is_inset0;
    EZSeamCross z_seam_cross;
    float start_flow;
    float start_speed;
    coord_t start_distance;
    float finish_flow;
    float finish_speed;
    coord_t finish_distance;

    ZSeamCrossConfig(const SliceMeshStorage& mesh, const bool is_inset0);
};

/*
* Helper class find new points on tangent normal A-B-C
*/
class ZSeamCrossing
{
public:
    Point pA;
    Point pB;
    Point pC;

    // default
    ZSeamCrossing(const ConstPolygonRef& wall, const size_t start_idx);

    ZSeamCrossing(const ConstPolygonRef& wall, const size_t start_idx, const Point& p0);

    bool FindCrossing_Mode_Normal(const ConstPolygonRef& wall, Point& point, coord_t point_start_distance, coord_t point_finish_distance, bool inside);
    bool FindCrossing_Mode_Custom(const ConstPolygonRef& wall, Point& point_start, Point& point_finish, coord_t point_start_distance, coord_t point_finish_distance, float angle_divider, bool inside);

private:
    bool isCoincident(const Point& a, const Point& b)
    {
        return vSize2(a - b) < 10; // points are closer than 10uM, consider them coincident
    }

    /*
    *   result
    *    \
    *     \p0
    * angle|
    *      |
    *      p1
    */
    Point RotateCustomLeft(const Point& p1, const Point& p0, const float angle);
};

class ZSeamIntersection
{
public:
    const ConstPolygonRef& wall;
    const int& start_idx;

    ZSeamIntersection(const ConstPolygonRef& wall, const int& start_idx);

    bool FindIntersection(const Point& p1, const Point& p2, int& ind_offset, Point& intersection);

    /*
    * Find intersection of 2 lines
    */
    static bool FindIntersection(const Point& start1, const Point& end1, const Point& start2, const Point& end2, Point& intersection);
};

class WallSeamCornerPoint
{
public:
    Point point;
    float flow_multiplier; // ~0.5 = 50%
    float speed_multiplier; // ~1.0 = 100%

    WallSeamCornerPoint(Point point, float flow_multiplier, float speed_multiplier);
};

class WallSeamConfig
{
public:
    int point_idx;
    Point point_center;
    bool point_crossing_found;
    WallSeamCornerPoint start;
    WallSeamCornerPoint finish;
    EZSeamCross cross_cfg;
    EZSeamCornerPrefType corner_pref;

    // Default ctor
    WallSeamConfig(int point_idx, Point point_center, EZSeamCross cross_cfg, EZSeamCornerPrefType corner_pref);
    WallSeamConfig(int point_idx, Point point_center, WallSeamCornerPoint& start, WallSeamCornerPoint& finish, EZSeamCross cross_cfg, EZSeamCornerPrefType corner_pref);
};

}

#endif //ZSEAMCONFIG_H