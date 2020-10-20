//Copyright (c) 2018 Ultimaker B.V.
//Copyright (c) 2020 PICASO 3D
//PicasoXCore is released under the terms of the AGPLv3 or higher

#include <map>
#include <cmath> // sin/cos
#include "pathOrderOptimizer.h"
#include "utils/logoutput.h"
#include "utils/SparsePointGridInclusive.h"
#include "utils/linearAlg2D.h"
#include "pathPlanning/LinePolygonsCrossings.h"
#include "pathPlanning/CombPath.h"

#define INLINE static inline

namespace cura {


/*
*   c
*    \
*     \b
*   ,-`|
* d`   |
*      a
*/
bool ZSeamCrossing::FindCrossing_Mode_Normal(const Polygons& walls, Point& point, coord_t point_start_distance, coord_t point_finish_distance, bool inside)
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

	if (walls.inside(pD1, false) == inside)
	{
		mr = point_start_distance;
		point = pB + Point((coord_t)(mr * (float)b_d.X / r_bd), (coord_t)(mr * (float)b_d.Y / r_bd));
		return true;
	}

	if (walls.inside(pD2, false) == inside)
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
bool ZSeamCrossing::FindCrossing_Mode_Custom(const Polygons& walls, Point& point_start, Point& point_end, coord_t point_start_distance, coord_t point_finish_distance, float angle_divider, bool inside)
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

	if (walls.inside(pD1, false) == inside)
	{
		coord_t mr_start = point_start_distance;
		coord_t mr_finish = point_finish_distance;
		point_start = pB + Point((coord_t)(mr_start * (float)b_dbc.X / r_dbc), (coord_t)(mr_start * (float)b_dbc.Y / r_dbc));
		point_end = pB + Point((coord_t)(mr_finish * (float)b_abd.X / r_abd), (coord_t)(mr_finish * (float)b_abd.Y / r_abd));
		return true;
	}

	if (walls.inside(pD2, false) == inside)
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


/**
*
*/
void PathOrderOptimizer::optimize(const bool calc_points)
{
    // NOTE: Keep this vector fixed-size, it replaces an (non-standard, sized at runtime) array:
    std::vector<bool> picked(polygons.size(), false);
    loc_to_line = nullptr;

    for (unsigned poly_idx = 0; poly_idx < polygons.size(); ++poly_idx) /// find closest point to initial starting point within each polygon +initialize picked
    {
        const ConstPolygonRef poly = *polygons[poly_idx];
        switch (config.type)
        {
            case EZSeamType::USER_SPECIFIED:
                polyStart.push_back(getClosestPointInPolygon(config.pos, poly_idx, calc_points));
                break;
            case EZSeamType::RANDOM:
                polyStart.push_back(getRandomPointInPolygon(poly_idx));
                break;
            case EZSeamType::SHARPEST_CORNER:
            case EZSeamType::SHORTEST:
            default:
                polyStart.push_back(getClosestPointInPolygon(startPoint, poly_idx, calc_points));
                break;
        }
        assert(poly.size() != 2);
    }


    Point prev_point;
    switch (config.type)
    {
        case EZSeamType::USER_SPECIFIED:
            prev_point = config.pos;
            break;
        case EZSeamType::RANDOM: //TODO: Starting position of the first polygon isn't random.
        case EZSeamType::SHARPEST_CORNER:
        case EZSeamType::SHORTEST:
        default:
            prev_point = startPoint;
    }
    for (unsigned int poly_order_idx = 0; poly_order_idx < polygons.size(); poly_order_idx++) /// actual path order optimizer
    {
        int best_poly_idx = -1;
        float bestDist2 = std::numeric_limits<float>::infinity();

        for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
        {
            if (picked[poly_idx] || polygons[poly_idx]->size() < 1) /// skip single-point-polygons
            {
                continue;
            }

            assert (polygons[poly_idx]->size() != 2);

            const Point& p = (*polygons[poly_idx])[polyStart[poly_idx]];
            float dist2 = vSize2f(p - prev_point);
            if (dist2 < bestDist2 && combing_boundary)
            {
                // using direct routing, this poly is the closest so far but as the combing boundary
                // is available, get the combed distance and use that instead
                if (PolygonUtils::polygonCollidesWithLineSegment(*combing_boundary, p, prev_point))
                {
                    if (!loc_to_line)
                    {
                        // the combing boundary has been provided so do the initialisation
                        // required to be able to calculate realistic travel distances to the start of new paths
                        //const int travel_avoid_distance = 2000; // assume 2mm - not really critical for our purposes
						const int travel_avoid_distance = combing_avoid_distance; // assume 2mm - not really critical for our purposes
                        loc_to_line = PolygonUtils::createLocToLineGrid(*combing_boundary, travel_avoid_distance);
                    }
                    CombPath comb_path;
                    if (LinePolygonsCrossings::comb(*combing_boundary, *loc_to_line, p, prev_point, comb_path, -40, 0, false))
                    {
                        float dist = 0;
                        Point last_point = p;
                        for (const Point& comb_point : comb_path)
                        {
                            dist += vSize(comb_point - last_point);
                            last_point = comb_point;
                        }
                        dist2 = dist * dist;
                    }
                }
            }
            if (dist2 < bestDist2)
            {
                best_poly_idx = poly_idx;
                bestDist2 = dist2;
            }

        }

        if (best_poly_idx > -1) /// should always be true; we should have been able to identify the best next polygon
        {
            assert(polygons[best_poly_idx]->size() != 2);

            prev_point = (*polygons[best_poly_idx])[polyStart[best_poly_idx]];

            picked[best_poly_idx] = true;
            polyOrder.push_back(best_poly_idx);
        }
        else
        {
            logError("Failed to find next closest polygon.\n");
        }
    }

    if (loc_to_line != nullptr)
    {
        delete loc_to_line;
    }
}

int PathOrderOptimizer::getClosestPointInPolygon(Point prev_point, int poly_idx, bool calc_points)
{
    ConstPolygonRef poly = *polygons[poly_idx];

    int best_point_idx = -1;
	float best_point_score = std::numeric_limits<float>::infinity();
    Point p0 = poly.back();
    for (unsigned int point_idx = 0; point_idx < poly.size(); point_idx++)
    {
        const Point& p1 = poly[point_idx];
        const Point& p2 = poly[(point_idx + 1) % poly.size()];
        // when type is SHARPEST_CORNER, actual distance is ignored, we use a fixed distance and decision is based on curvature only
        float dist_score = (config.type == EZSeamType::SHARPEST_CORNER && config.corner_pref != EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE)? 10000 : vSize2f(p1 - prev_point);
        const float corner_angle = LinearAlg2D::getAngleLeft(p0, p1, p2) / M_PI; // 0 -> 2
		float corner_shift;
        if (config.type == EZSeamType::SHORTEST)
        {
            // the more a corner satisfies our criteria, the closer it appears to be
            // shift 10mm for a very acute corner
            corner_shift = 10000 * 10000;
        }
        else
        {
            // the larger the distance from prev_point to p1, the more a corner will "attract" the seam
            // so the user has some control over where the seam will lie.

            // the divisor here may need adjusting to obtain the best results (TBD)
            corner_shift = dist_score / 10;
        }
		switch (config.corner_pref)
		{
		case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_INNER:
			if (corner_angle > 1)
			{
				// p1 lies on a concave curve so reduce the distance to favour it
				// the more concave the curve, the more we reduce the distance
				dist_score -= (corner_angle - 1) * corner_shift;
			}
			break;
		case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_OUTER:
			if (corner_angle < 1)
			{
				// p1 lies on a convex curve so reduce the distance to favour it
				// the more convex the curve, the more we reduce the distance
				dist_score -= (1 - corner_angle) * corner_shift;
			}
			break;
		case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_ANY:
			// the more curved the region, the more we reduce the distance
			dist_score -= fabs(corner_angle - 1) * corner_shift;
			break;
		case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_WEIGHTED:
			//More curve is better score (reduced distance), but slightly in favour of concave curves.
			dist_score -= fabs(corner_angle - 0.8) * corner_shift;
			break;
		case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_SMOOTH:
		case EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_NONE:
		default:
			// do nothing
			break;
		}
        if (dist_score < best_point_score)
        {
            best_point_idx = point_idx;
            best_point_score = dist_score;
        }
        p0 = p1;
    }

	if (calc_points)
	{
		bool intersection_found = false;

		if (config.corner_pref == EZSeamCornerPrefType::Z_SEAM_CORNER_PREF_SMOOTH)
		{
			const Point& poly_center = poly.centerOfMass();
			const Point& seam_target = prev_point;

			ZSeamIntersection seamIntersection(poly, best_point_idx);

			int ind_offset = 0;
			Point pIntersection;

			if (seamIntersection.FindIntersection(poly_center, seam_target, ind_offset, pIntersection))
			{
				polyPointValue.push_back(pIntersection);
				polyPointFound.push_back(true);

				best_point_idx = (poly.size() + best_point_idx + ind_offset) % poly.size();

				intersection_found = true;
			}
		}

		if (intersection_found == false)
		{
			polyPointValue.push_back(poly[best_point_idx]);
			polyPointFound.push_back(false);
		}
	}

    return best_point_idx;
}

int PathOrderOptimizer::getRandomPointInPolygon(int poly_idx)
{
	ConstPolygonRef poly = *polygons[poly_idx];
	int best_point_idx = rand() % poly.size();

	polyPointValue.push_back(poly[best_point_idx]);
	polyPointFound.push_back(false);

	return best_point_idx;
}

static inline bool pointsAreCoincident(const Point& a, const Point& b)
{
    return vSize2(a - b) < 25; // points are closer than 5uM, consider them coincident
}

/**
*
*/
void LineOrderOptimizer::optimize(bool find_chains)
{
    const int grid_size = 2000; // the size of the cells in the hash grid. TODO
    SparsePointGridInclusive<unsigned int> line_bucket_grid(grid_size);
    // NOTE: Keep this vector fixed-size, it replaces an (non-standard, sized at runtime) array:
    std::vector<bool> picked(polygons.size(), false);

    loc_to_line = nullptr;

    for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++) /// find closest point to initial starting point within each polygon +initialize picked
    {
        int best_point_idx = -1;
        float best_point_dist = std::numeric_limits<float>::infinity();
        ConstPolygonRef poly = *polygons[poly_idx];
        for (unsigned int point_idx = 0; point_idx < poly.size(); point_idx++) /// get closest point from polygon
        {
            float dist = vSize2f(poly[point_idx] - startPoint);
            if (dist < best_point_dist)
            {
                best_point_idx = point_idx;
                best_point_dist = dist;
            }
        }
        polyStart.push_back(best_point_idx);

        assert(poly.size() == 2);

        line_bucket_grid.insert(poly[0], poly_idx);
        line_bucket_grid.insert(poly[1], poly_idx);
    }

    // a map with an entry for each chain end discovered
    //   keys are the indices of the lines (polys) that start/end a chain of lines
    //   values indicate which of the line's points are at the end of the chain
    std::map<unsigned, unsigned> chain_ends;

    std::vector<unsigned> singletons; // indices of the line segments that don't join any other

    if (find_chains)
    {
        // locate the chain ends by finding lines that join exactly one other line at one end and join either 0 or 2 or more lines at the other end

        // we also consider lines that meet 2 or more lines at one end and nothing at the other end as chain ends

        // finally, those lines that do not join any other are added to the collection of singletons

        for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
        {
            int num_joined_lines[2];
            for (unsigned point_idx = 0; point_idx < 2; ++point_idx)
            {
                std::set<int> joined_lines; // use a set because getNearbyVals() appears to return duplicates (?)
                num_joined_lines[point_idx] = 0;
                const Point& p = (*polygons[poly_idx])[point_idx];
                // look at each of the lines that finish close to this line to see if either of its vertices are coincident this vertex
                for (unsigned int close_line_idx : line_bucket_grid.getNearbyVals(p, 10))
                {
                    if (close_line_idx != poly_idx && (pointsAreCoincident(p, (*polygons[close_line_idx])[0]) || pointsAreCoincident(p, (*polygons[close_line_idx])[1])))
                    {
                        joined_lines.insert(close_line_idx);
                    }
                }
                num_joined_lines[point_idx] = joined_lines.size();
            }
            if (num_joined_lines[0] != 1 && num_joined_lines[1] == 1)
            {
                // point 0 of candidate line starts a chain of 2 or more lines
                chain_ends[poly_idx] = 0;
            }
            else if (num_joined_lines[1] != 1 && num_joined_lines[0] == 1)
            {
                // point 1 of candidate line starts a chain of 2 or more lines
                chain_ends[poly_idx] = 1;
            }
            else if (num_joined_lines[0] == 0 && num_joined_lines[1] > 1)
            {
                // point 0 is the free end of a line that meets 2 or more lines at a junction
                chain_ends[poly_idx] = 0;
            }
            else if (num_joined_lines[1] == 0 && num_joined_lines[0] > 1)
            {
                // point 1 is the free end of a line that meets 2 or more lines at a junction
                chain_ends[poly_idx] = 1;
            }
            else if (num_joined_lines[0] == 0 && num_joined_lines[1] == 0)
            {
                // line is not connected to anything but if there are chains we may want to print it
                // before moving away to a different area so make it possible for it to be selected
                // before all the chains have been printed
                singletons.push_back(poly_idx);
            }
        }
    }

    Point prev_point = startPoint;

    for (unsigned int order_idx = 0; order_idx < polygons.size(); order_idx++) /// actual path order optimizer
    {
        int best_line_idx = -1;
        float best_score = std::numeric_limits<float>::infinity(); // distance score for the best next line

        const int close_point_radius = 5000;

        // for the first line we would prefer a line that is at the end of a sequence of connected lines (think zigzag) and
        // so we only consider the closest line when looking for the second line onwards
        if (order_idx > 0)
        {
            // first check if a line segment starts (really) close to last point
            // this will find the next line segment in a chain
            for(unsigned int close_line_idx : line_bucket_grid.getNearbyVals(prev_point, 10))
            {
                if (picked[close_line_idx]
                    || !(pointsAreCoincident(prev_point,(*polygons[close_line_idx])[0]) || pointsAreCoincident(prev_point, (*polygons[close_line_idx])[1])))
                {
                    continue;
                }
                updateBestLine(close_line_idx, best_line_idx, best_score, prev_point);
            }
        }

        if (best_line_idx == -1)
        {
            // we didn't find a chained line segment so now look for any lines that start within close_point_radius
            for(unsigned int close_line_idx : line_bucket_grid.getNearbyVals(prev_point, close_point_radius))
            {
                if (picked[close_line_idx])
                {
                    continue;
                }
                updateBestLine(close_line_idx, best_line_idx, best_score, prev_point);
            }
        }

        if (best_line_idx != -1 && best_score > (2 * close_point_radius * close_point_radius))
        {
            // we found a point that is close to prev_point as the crow flies but the score is high so it must have been
            // penalised due to the part boundary clashing with the straight line path so let's forget it and find something closer
            best_line_idx = -1;
            best_score = std::numeric_limits<float>::infinity();
        }

        if (best_line_idx != -1 && !pointsAreCoincident(prev_point, (*polygons[best_line_idx])[polyStart[best_line_idx]]))
        {
            // we found a point close to prev_point but it's not close enough for the points to be considered coincident so we would
            // probably be better off by ditching this point and finding an end of a chain instead (let's hope it's not too far away!)
            for (auto it = chain_ends.begin(); it != chain_ends.end(); ++it )
            {
                if (!picked[it->first])
                {
                    best_line_idx = -1;
                    best_score = std::numeric_limits<float>::infinity();
                    break;
                }
            }
        }

        if (best_line_idx == -1)
        {
            std::vector<std::map<unsigned, unsigned>::iterator> zombies; // chain end lines that have already been output

            // no point is close to the previous point, consider the points on the chain end lines that have yet to be picked

            for (auto it = chain_ends.begin(); it != chain_ends.end(); ++it )
            {
                if (picked[it->first])
                {
                    zombies.push_back(it);
                }
                else
                {
                    updateBestLine(it->first, best_line_idx, best_score, prev_point, it->second);
                }
            }

            for (auto zombie : zombies)
            {
                chain_ends.erase(zombie);
            }

            // if any singletons are not yet printed, consider them as well
            for (auto poly_idx : singletons)
            {
                if (!picked[poly_idx])
                {
                    updateBestLine(poly_idx, best_line_idx, best_score, prev_point);
                }
            }
        }

        // fallback to using the nearest unpicked line
        if (best_line_idx == -1) /// if single-line-polygon hasn't been found yet
        {
            for (unsigned int poly_idx = 0; poly_idx < polygons.size(); poly_idx++)
            {
                if (picked[poly_idx])
                {
                    continue;
                }

                updateBestLine(poly_idx, best_line_idx, best_score, prev_point);

            }
        }

        if (best_line_idx > -1) /// should always be true; we should have been able to identify the best next polygon
        {
            ConstPolygonRef best_line = *polygons[best_line_idx];

            int line_start_point_idx = polyStart[best_line_idx];
            int line_end_point_idx = line_start_point_idx * -1 + 1; /// 1 -> 0 , 0 -> 1
            const Point& line_end = best_line[line_end_point_idx];
            prev_point = line_end;

            picked[best_line_idx] = true;
            polyOrder.push_back(best_line_idx);
        }
        else
        {
            logError("Failed to find next closest line.\n");
        }
    }
    if (loc_to_line != nullptr)
    {
        delete loc_to_line;
    }
}

float LineOrderOptimizer::combingDistance2(const Point &p0, const Point &p1)
{
    if (loc_to_line == nullptr)
    {
        // do the initialisation required to be able to calculate realistic travel distances to the start of new paths
        loc_to_line = PolygonUtils::createLocToLineGrid(*combing_boundary, 1000); // 1mm grid to reduce computation time
    }

    CombPath comb_path;
    if (LinePolygonsCrossings::comb(*combing_boundary, *loc_to_line, p0, p1, comb_path, -40, 0, false))
    {
        float dist = 0;
        Point last_point = p0;
        for (const Point& comb_point : comb_path)
        {
            dist += vSize(comb_point - last_point);
            last_point = comb_point;
        }
        return dist * dist;
    }

    // couldn't comb, fall back to a large distance

    return vSize2f(p1 - p0) * 10000;
}

/*
in:
 poly_idx: candidate for best polygon index
 prev_point
 incoming_perpundicular_normal: incoming angle, turned 90 degrees CCW
 just_point: default -1, 0, or 1
out:
 best, best_score
*/
inline void LineOrderOptimizer::updateBestLine(unsigned int poly_idx, int& best, float& best_score, Point prev_point, int just_point)
{
    // when looking at a chain end, just_point will be either 0 or 1 depending on which vertex we are currently interested in testing
    // if just_point is -1, it means that we are not looking at a chain end and we will test both vertices to see if either is best

    const Point& p0 = (*polygons[poly_idx])[0];
    const Point& p1 = (*polygons[poly_idx])[1];

    if (just_point != 1)
    { /// check distance to first point on line (0)
        float score = vSize2f(p0 - prev_point);
        if (score < best_score
            && combing_boundary != nullptr
            && !pointsAreCoincident(p0, prev_point)
            && PolygonUtils::polygonCollidesWithLineSegment(*combing_boundary, p0, prev_point))
        {
            score = combingDistance2(p0, prev_point);
        }
        if (score < best_score)
        {
            best = poly_idx;
            best_score = score;
            polyStart[poly_idx] = 0;
        }
    }
    if (just_point != 0)
    { /// check distance to second point on line (1)
        float score = vSize2f(p1 - prev_point);
        if (score < best_score
            && combing_boundary != nullptr
            && !pointsAreCoincident(p1, prev_point)
            && PolygonUtils::polygonCollidesWithLineSegment(*combing_boundary, p1, prev_point))
        {
            score = combingDistance2(p1, prev_point);
        }
        if (score < best_score)
        {
            best = poly_idx;
            best_score = score;
            polyStart[poly_idx] = 1;
        }
    }
}

}//namespace cura