//Copyright (c) 2018 Ultimaker B.V.
//Copyright (c) 2021 PICASO 3D
//PicasoXCore is released under the terms of the AGPLv3 or higher

#ifndef PATHOPTIMIZER_H
#define PATHOPTIMIZER_H

#include <stdint.h>
#include "settings/EnumSettings.h"
#include "utils/polygon.h"
#include "utils/polygonUtils.h"
#include "sliceDataStorage.h"
#include "settings/types/Ratio.h"

namespace cura {

/*!
 * Helper class that encapsulates the various criteria that define the location of the z-seam.
 * Instances of this are passed to the PathOrderOptimizer to specify where the z-seam is to be located.
 */
class ZSeamConfig
{
public:
    EZSeamType type;
    Point pos; //!< The position near where to create the z_seam (if \ref PathOrderOptimizer::type == 'back')
    EZSeamCornerPrefType corner_pref;
    // default constructor
    ZSeamConfig()
    : type(EZSeamType::SHORTEST)
    , pos(Point(0, 0))
    , corner_pref(EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE)
    {
    }
    ZSeamConfig(EZSeamType type, Point pos, EZSeamCornerPrefType corner_pref)
    : type(type)
    , pos(pos)
    , corner_pref(corner_pref)
    {
    }
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

	ZSeamCrossConfig(const SliceMeshStorage& mesh, const bool is_inset0)
		: is_inset0(is_inset0)
		, z_seam_cross(mesh.settings.get<EZSeamCross>(is_inset0 ? "z_seam_cross_0" : "z_seam_cross_1"))
		, start_flow(static_cast<float>(mesh.settings.get<Ratio>(is_inset0 ? "z_seam_cross_0_start_flow": "z_seam_cross_1_start_flow")))
		, start_speed(static_cast<float>(mesh.settings.get<Ratio>(is_inset0 ? "z_seam_cross_0_start_speed" : "z_seam_cross_1_start_speed")))
		, start_distance(mesh.settings.get<coord_t>(is_inset0 ? "z_seam_cross_0_start_distance" : "z_seam_cross_1_start_distance"))
		, finish_flow(static_cast<float>(mesh.settings.get<Ratio>(is_inset0 ? "z_seam_cross_0_finish_flow" : "z_seam_cross_1_finish_flow")))
		, finish_speed(static_cast<float>(mesh.settings.get<Ratio>(is_inset0 ? "z_seam_cross_0_finish_speed" : "z_seam_cross_1_finish_speed")))
		, finish_distance(mesh.settings.get<coord_t>(is_inset0 ? "z_seam_cross_0_finish_distance" : "z_seam_cross_1_finish_distance"))
	{
	}
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
	ZSeamCrossing(const ConstPolygonRef& wall, const size_t start_idx)
		: pA(wall[(wall.size() + start_idx - 1) % wall.size()])
		, pB(wall[start_idx])
		, pC(wall[(start_idx + 1) % wall.size()])
	{
	}

	ZSeamCrossing(const ConstPolygonRef& wall, const size_t start_idx, const Point& p0)
		:pA(wall[(wall.size() + start_idx - 1) % wall.size()])
		, pB(p0)
		, pC(wall[start_idx])
	{
	}


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

	ZSeamIntersection(const ConstPolygonRef& wall, const int& start_idx)
		: wall(wall)
		, start_idx(start_idx)
	{
	}

	/*
	ZSeamIntersection(const ConstPolygonRef& wall, int start_idx)
		: pA3(wall[(wall.size() + start_idx - 3) % wall.size()])
		, pA2(wall[(wall.size() + start_idx - 2) % wall.size()])
		, pA1(wall[(wall.size() + start_idx - 1) % wall.size()])
		, pB(wall[start_idx])
		, pC1(wall[(start_idx + 1) % wall.size()])
		, pC2(wall[(start_idx + 2) % wall.size()])
		, pC3(wall[(start_idx + 3) % wall.size()])
	{
	}
	*/

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
	
	WallSeamCornerPoint(Point point, float flow_multiplier, float speed_multiplier)
		: point(point)
		, flow_multiplier(flow_multiplier)
		, speed_multiplier(speed_multiplier)
	{
	}
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
	WallSeamConfig(int point_idx, Point point_center, EZSeamCross cross_cfg, EZSeamCornerPrefType corner_pref)
		: point_idx(point_idx)
		, point_center(point_center)
		, point_crossing_found(false)
		, start(WallSeamCornerPoint(Point(0,0), 0.5, 1.0))
		, finish(WallSeamCornerPoint(Point(0, 0), 0.5, 1.0))
		, cross_cfg(cross_cfg)
		, corner_pref(corner_pref)
	{
	}

	WallSeamConfig(int point_idx, Point point_center, WallSeamCornerPoint& start, WallSeamCornerPoint& finish, EZSeamCross cross_cfg, EZSeamCornerPrefType corner_pref)
		: point_idx(point_idx)
		, point_center(point_center)
		, point_crossing_found(true)
		, start(start)
		, finish(finish)
		, cross_cfg(cross_cfg)
		, corner_pref(corner_pref)
	{
	}

};

/*!
 * Parts order optimization class.
 * 
 * Utility class for optimizing the path order by minimizing the distance traveled between printing different parts in the layer.
 * The order of polygons is optimized and the startingpoint within each polygon is chosen.
 */
class PathOrderOptimizer
{
public:
    Point startPoint; //!< A location near the prefered start location
    const ZSeamConfig config;
    std::vector<ConstPolygonPointer> polygons; //!< the parts of the layer (in arbitrary order)
    std::vector<int> polyStart; //!< polygons[i][polyStart[i]] = point of polygon i which is to be the starting point in printing the polygon
    std::vector<int> polyOrder; //!< the optimized order as indices in #polygons
	std::vector<bool> polyPointFound;
	std::vector<Point> polyPointValue;
    LocToLineGrid* loc_to_line;
    const Polygons* combing_boundary;
	int combing_avoid_distance;

    PathOrderOptimizer(Point startPoint, const ZSeamConfig config = ZSeamConfig(), const Polygons* combing_boundary = nullptr)
    : startPoint(startPoint)
    , config(config)
    , combing_boundary((combing_boundary != nullptr && combing_boundary->size() > 0) ? combing_boundary : nullptr)
	, combing_avoid_distance(2000)
    {
    }

    void addPolygon(PolygonRef polygon)
    {
        polygons.emplace_back(polygon);
    }

    void addPolygon(ConstPolygonRef polygon)
    {
        polygons.emplace_back(polygon);
    }

    void addPolygons(const Polygons& polygons)
    {
        for(unsigned int i = 0; i < polygons.size(); i++)
        {
            this->polygons.emplace_back(polygons[i]);
        }
    }

    void optimize(const bool calc_points = false); //!< sets #polyStart and #polyOrder

private:
    int getClosestPointInPolygon(Point prev, int i_polygon, bool calc_points); //!< returns the index of the closest point
    int getRandomPointInPolygon(int poly_idx);
};
//! Line path order optimization class.
/*!
* Utility class for optimizing the path order by minimizing the distance traveled between printing different lines within a part.
*/
class LineOrderOptimizer
{
public:
    Point startPoint; //!< The location of the nozzle before starting to print the current layer
    std::vector<ConstPolygonPointer> polygons; //!< the parts of the layer (in arbitrary order)
    std::vector<int> polyStart; //!< polygons[i][polyStart[i]] = point of polygon i which is to be the starting point in printing the polygon
    std::vector<int> polyOrder; //!< the optimized order as indices in #polygons
    LocToLineGrid* loc_to_line;
    const Polygons* combing_boundary; //!< travel moves that cross this boundary are penalised so they are less likely to be chosen

    LineOrderOptimizer(Point startPoint, const Polygons* combing_boundary = nullptr)
    {
        this->startPoint = startPoint;
        this->combing_boundary = (combing_boundary != nullptr && combing_boundary->size() > 0) ? combing_boundary : nullptr;
    }

    void addPolygon(PolygonRef polygon)
    {
        polygons.push_back(polygon);
    }

    void addPolygon(ConstPolygonRef polygon)
    {
        polygons.push_back(polygon);
    }

    void addPolygons(Polygons& polygons)
    {
        for(unsigned int i=0;i<polygons.size(); i++)
        {
            this->polygons.push_back(polygons[i]);
        }
    }

    /*!
     * Do the optimization
     *
     * \param find_chains Whether to determine when lines are chained together (i.e. zigzag infill)
     *
     * \return The squared travel distance between the two points
     */
    void optimize(bool find_chains = true); //!< sets #polyStart and #polyOrder

private:
    /*!
     * Update LineOrderOptimizer::polyStart if the current line is better than the current best.
     * 
     * \param poly_idx[in] The index in LineOrderOptimizer::polygons for the current line to test
     * \param best[in, out] The index of current best line
     * \param best_score[in, out] The distance score for the current best line
     * \param prev_point[in] The previous point from which to find the next best line
     * \param just_point[in] If not -1, only look at the line vertex with this index
     */
    void updateBestLine(unsigned int poly_idx, int& best, float& best_score, Point prev_point, int just_point = -1);

    /*!
     * Compute the squared distance from \p p0 to \p p1 using combing
     *
     * \param p0 A point
     * \param p1 Another point
     *
     * \return The squared travel distance between the two points
     */
    float combingDistance2(const Point &p0, const Point &p1);
};

}//namespace cura

#endif//PATHOPTIMIZER_H
