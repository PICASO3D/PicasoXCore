//Copyright (c) 2020 Ultimaker B.V.
//Copyright (c) 2022 PICASO 3D
//PicasoXCore is released under the terms of the AGPLv3 or higher

#ifndef ZSEAMCONFIG_H
#define ZSEAMCONFIG_H

#include "EnumSettings.h" //For EZSeamType and EZSeamCornerPrefType.
#include "../utils/IntPoint.h" //To store the preferred seam position.
#include "../utils/linearAlg2D.h"
#include "../sliceDataStorage.h"
#include "types/Ratio.h"

namespace cura
{

/*!
 * Helper class that encapsulates the various criteria that define the location
 * of the z-seam.
 * Instances of this are passed to the PathOrderOptimizer to specify where the
 * seam is to be located.
 */
struct ZSeamConfig
{
    /*!
     * Strategy to place the seam (user-specified, shortest distance, sharpest
     * corner, etc.).
     */
    EZSeamType type;

    /*!
     * When using a user-specified position for the seam, this is the position
     * that the user specified.
     */
    Point pos;

    /*!
     * Corner preference type, if using the sharpest corner strategy.
     */
    EZSeamCornerPrefType corner_pref;

    /*!
     * Prevent 'smoothed out' corners (corners that are spread over multiple, very close together vertices),
     * by simplifying the polygon that the corners are detected on by this ammount.
     * This does _not_ influence the path, the simplified polygon is a temporary constructed within the algorithm.
     */
    coord_t simplify_curvature;

    /*!
     * Create a seam configuration with a custom configuration.
     * \param type The strategy to place the seam.
     * \param pos The position of a user-specified seam.
     * \param corner_pref The corner preference, when using the sharpest corner strategy.
     * \param by how much to simplify the curvature (when detecting corners), as otherwise 'smooth' corners are penalized.
     */
    ZSeamConfig
    (
        const EZSeamType type = EZSeamType::SHORTEST,
        const Point pos = Point(0, 0),
        const EZSeamCornerPrefType corner_pref = EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE,
        const coord_t simplify_curvature = 0
    );
};

/*
 * Config seam cross preference for inset0/inset1
 */
class ZSeamCrossConfig
{
  public:
    EZSeamCross z_seam_cross;
    float start_width;
    coord_t start_distance;
    float finish_width;
    coord_t finish_distance;

    ZSeamCrossConfig(const Settings& settings, const bool outer);
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
    ZSeamCrossing(const ConstPolygonRef& wall, const size_t start_idx, const size_t offset_prev_idx, const size_t offset_next_idx);

    ZSeamCrossing(const ConstPolygonRef& wall, const size_t start_idx, const Point& p0, const size_t offset_prev_idx, const size_t offset_next_idx);

    bool FindCrossing_Mode_Normal(const ConstPolygonRef& wall,
                                  Point& point,
                                  coord_t point_start_distance,
                                  coord_t point_finish_distance,
                                  bool inside);
    bool FindCrossing_Mode_Custom(const ConstPolygonRef& wall,
                                  Point& point_start,
                                  Point& point_finish,
                                  coord_t point_start_distance,
                                  coord_t point_finish_distance,
                                  float angle_divider,
                                  bool inside);

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

    bool FindIntersection(const Point& p1, const Point& p2, size_t& ind_offset, Point& intersection);

    /*
     * Find intersection of 2 lines
     */
    static bool FindIntersection(const Point& start1, const Point& end1, const Point& start2, const Point& end2, Point& intersection);

    /*
    * Find Point cross p1-p2
    */
    bool FindPointCrossLine(const Point& p1, const Point& p2, size_t& ind_offset, Point& cross_point, size_t& cross_distance);

    /*
     * Check Point on line between p1-p2
     */
    bool CheckCrossing(const Point& current, const Point& p1, const Point& p2, size_t& cross_distance);
};

class WallSeamCornerPoint
{
  public:
    Point point;
    float width_multiplier; // ~0.5 = 50%

    WallSeamCornerPoint(Point point, float width_multiplier);
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

} //Cura namespace.

#endif //ZSEAMCONFIG_H