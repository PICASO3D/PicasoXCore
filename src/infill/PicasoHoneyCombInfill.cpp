//Copyright (c) 2021 PICASO 3D
//PicasoXCore is released under the terms of the AGPLv3 or higher

#include <sstream>
#include <algorithm>

#include "PicasoHoneyCombInfill.h"
#include "../utils/AABB.h"
#include "../utils/linearAlg2D.h"
#include "../utils/polygon.h"
#include "../utils/SVG.h"

namespace cura {

/*
* https://www.redblobgames.com/grids/hexagons/
*/
PicasoHoneyCombInfill::PicasoHoneyCombInfill() {
}

PicasoHoneyCombInfill::~PicasoHoneyCombInfill() {
}

void PicasoHoneyCombInfill::generateHoneyCombInfill(Polygons& result_lines, coord_t outline_offset, coord_t infill_line_width, coord_t line_distance, const Polygons& in_outline, coord_t z, int layer_nr, bool simple, AngleDegrees fill_angle)
{
	/*
	* Based on slic3r HoneyComb
	* https://github.com/prusa3d/PrusaSlicer/blob/master/src/libslic3r/Fill/FillHoneycomb.cpp
	*/

	const Polygons outline = in_outline.offset(outline_offset);
	const AABB outline_bounding_box(outline.offset(line_distance));

	HexPattern m = HexPattern();
	m.distance = line_distance;
	m.hex_side = m.distance / (std::sqrt(3) / 2);
	m.hex_width = m.distance * 2;
	coord_t hex_height = m.hex_side * 2;
	m.pattern_height = hex_height + m.hex_side;
	m.y_short = m.distance * sqrt(3) / 3;
	m.x_offset = infill_line_width / 3; // hex pattern double line x_offset
	m.y_offset = m.x_offset * sqrt(3) / 3;
	m.hex_center = Point(m.hex_width / 2, m.hex_side);

	const int rotationIndex = simple ? 0 : (layer_nr % 3);
	const int angleDegrees = (rotationIndex == 0 ? 0 : (rotationIndex == 1 ? 60 : 120));
	const double angleRadians = double(angleDegrees + fill_angle) * M_PI / double(180);

	AABB bounding_box(outline);
	{
		// rotate bounding box according to infill direction
		Polygon bb_polygon = bounding_box.toPolygon();
		bb_polygon.rotate(angleRadians, m.hex_center);
		bounding_box = AABB(bb_polygon);

		// extend bounding box so that our pattern will be aligned with other layers
		// $bounding_box->[X1] and [Y1] represent the displacement between new bounding box offset and old one
		// The infill is not aligned to the object bounding box, but to a world coordinate system. Supposedly good enough.
		bounding_box.include(_align_to_grid(bounding_box.min, Point(m.hex_width, m.pattern_height)));
	}

	Polygons polygons;
	{
		coord_t x = bounding_box.min.X;
		while (x <= bounding_box.max.X) {
			Polygon p;
			coord_t ax[2] = { 
				x + m.x_offset,
				x + m.distance - m.x_offset
			};

			for (size_t i = 0; i < 2; ++i) {
				p.reverse();
				for (coord_t y = bounding_box.min.Y; y <= bounding_box.max.Y; y += m.y_short + m.hex_side + m.y_short + m.hex_side) {
					p.add(Point(ax[1], y + m.y_offset));
					p.add(Point(ax[0], y + m.y_short - m.y_offset));
					p.add(Point(ax[0], y + m.y_short + m.hex_side + m.y_offset));
					p.add(Point(ax[1], y + m.y_short + m.hex_side + m.y_short - m.y_offset));
					p.add(Point(ax[1], y + m.y_short + m.hex_side + m.y_short + m.hex_side + m.y_offset));
				}
				ax[0] = ax[0] + m.distance;
				ax[1] = ax[1] + m.distance;
				std::swap(ax[0], ax[1]); // draw symmetrical pattern
				x += m.distance;
			}
			p.rotate(-angleRadians, m.hex_center);
			polygons.add(p);
		}
	}

	Polygons result;
	for (ConstPolygonRef poly : polygons)
	{
		Point last;
		bool is_first_point = true;
		for (Point current : poly)
		{
			if (!is_first_point)
			{
				result.addLine(last, current);
			}
			last = current;
			is_first_point = false;
		}
	}

	result = outline.intersectionPolyLines(result);
	result_lines = result;

	//std::ostringstream filePath;
	//filePath << "d:/tmp/Infill/infill_z_" << z << ".svg";
	//PicasoHoneyCombInfill::debugSVG(filePath.str(), outline_bounding_box, mesh_bounding_box, result, m);
}

/*
void PicasoHoneyCombInfill::generateSimpleHoneyCombInfill(Polygons& result_lines, bool zig_zaggify, coord_t outline_offset, coord_t infill_line_width, coord_t line_distance, const Polygons& in_outline, coord_t z, const AABB mesh_bounding_box)
{
    const Polygons outline = in_outline.offset(outline_offset);
    const AABB outline_bounding_box(outline.offset(line_distance));

    coord_t pitch = line_distance * 2.41; // this produces similar density to the "line" infill pattern

	//float min_spacing = 0.654498 * 10000;
	//float min_spacing = line_distance;
	//float density = 2;

	CacheData m = CacheData();
	//m.distance = min_spacing / params.density; // orig
	//m.distance = min_spacing / density; // orig+
	m.distance = line_distance;
	m.hex_side = m.distance / (std::sqrt(3) / 2);
	m.hex_width = m.distance * 2; // $m->{hex_width} == $m->{hex_side} * sqrt(3);
	coord_t hex_height = m.hex_side * 2;
	m.pattern_height = hex_height + m.hex_side;
	m.y_short = m.distance * sqrt(3) / 3;
	//m.x_offset = min_spacing / 2; // orig
	m.x_offset = line_distance;
	m.y_offset = m.x_offset * sqrt(3) / 3;
	m.hex_center = Point(m.hex_width / 2, m.hex_side);

	coord_t x_period = m.hex_width;
	coord_t y_period = m.y_short + m.hex_side + m.y_short + m.hex_side;

	const Point middle = mesh_bounding_box.getMiddle();
	const AngleRadians rotationDegrees(45);

	coord_t middle_x_count = (middle.X - mesh_bounding_box.min.X) / x_period;
	coord_t middle_y_count = (middle.Y - mesh_bounding_box.min.Y) / y_period;
	Point rotation_center = Point(
		mesh_bounding_box.min.X + x_period * middle_x_count + m.hex_center.X,
		mesh_bounding_box.min.Y + y_period * middle_y_count + m.hex_center.X
	);
	double rotation_sin = sin(rotationDegrees);
	double rotation_cos = cos(rotationDegrees);

	Polygons polygons;
	{
		coord_t pxind[5] = { 0, 1, 1, 0, 0 };
		coord_t pattern_offset = infill_line_width;
		
		for (coord_t x = mesh_bounding_box.min.X; x <= mesh_bounding_box.max.X; x += x_period)
		{
			// Pattern A
			//Polygon patternA;
			Polygons polygonsA;
			coord_t pxA[2] = {
				x + m.x_offset,
				x + m.distance - m.x_offset + pattern_offset
			};
			Point last_A;
			bool is_first_point_A = true;
			bool last_inside_A = false;

			// Pattern B
			//Polygon patternB;
			Polygons polygonsB;
			coord_t pxB[2] = {
				x + m.distance - m.x_offset + m.distance + pattern_offset,
				x + m.x_offset + m.distance
			};
			Point last_B;
			bool is_first_point_B = true;
			bool last_inside_B = false;

			for (coord_t y = mesh_bounding_box.min.Y; y <= mesh_bounding_box.max.Y; y += y_period)
			{
				//if (outline_bounding_box.contains(Point(x, y)) == false)
				//	break;

				coord_t py[5] = {
					y + m.y_short - m.y_offset,
					y + m.y_offset,
					y + m.y_short + m.hex_side + m.y_short - m.y_offset,
					y + m.y_short + m.hex_side + m.y_offset,
					y + m.y_short + m.hex_side + m.y_short + m.hex_side
				};

				is_first_point_A = true;
				is_first_point_B = true;

				for (size_t pyi = 0; pyi < 5; ++pyi)
				{
					// Pattern A
					{
						Point current = Point(pxA[pxind[pyi]], py[pyi]);
						//bool current_inside = outline.inside(current, true);
						if (!is_first_point_A)
						{
							polygonsA.addLine(last_A, current);
						}
						last_A = current;
						//last_inside_A = current_inside;
						is_first_point_A = false;
					}

					// Pattern B
					{
						Point current = Point(pxB[pxind[pyi]], py[pyi]);
						//bool current_inside = outline.inside(current, true);
						if (!is_first_point_B)
						{
							polygonsB.addLine(last_B, current);
						}
						last_B = current;
						//last_inside_B = current_inside;
						is_first_point_B = false;
					}
				}
			}

			if (polygonsA.size() > 0) {
				polygons.add(outline.intersectionPolyLines(polygonsA));
				//polygons.add(polygonsA);
			}
			if (polygonsB.size() > 0) {
				polygons.add(outline.intersectionPolyLines(polygonsB));
				//polygons.add(polygonsB);
			}
			//polygons.add(polygonsA);
			//polygons.add(polygonsB);
			//x += x_period;
		}
	}
	result_lines = polygons;

	std::ostringstream filePath;
	filePath << "d:/tmp/Infill/infill_z_" << z << ".svg";
	PicasoHoneyCombInfill::debugSVG(filePath.str(), outline_bounding_box, mesh_bounding_box, polygons, m);
}
*/

void PicasoHoneyCombInfill::debugSVG(std::string filename, const AABB& outline_bounding_box, const AABB mesh_bounding_box, Polygons& lines, HexPattern& m)
{
	SVG svg(filename.c_str(), mesh_bounding_box);

	Polygon outline_corner = outline_bounding_box.toPolygon();
	svg.writePolygon(outline_corner, SVG::Color::GREEN, 1.0F);

	Polygon mesh_corner = mesh_bounding_box.toPolygon();
	svg.writePolygon(mesh_corner, SVG::Color::GREEN, 1.0F);

	for (ConstPolygonRef p : lines)
	{
		svg.writePolygon(p, SVG::Color::YELLOW, 1.0F);
	}

	const AABB aabb(mesh_bounding_box);

	std::ostringstream text1;
	text1 << "Distance: " << m.distance;
	svg.writeText(Point(aabb.min.X, aabb.max.Y), text1.str(), SVG::Color::RED, 20);
	std::ostringstream text2;
	text2 << "Hex.Center: " << m.hex_center;
	svg.writeText(Point(aabb.min.X, aabb.max.Y - 2000), text2.str(), SVG::Color::RED, 20);
	std::ostringstream text3;
	text3 << "Hex.Side: " << m.hex_side;
	svg.writeText(Point(aabb.min.X, aabb.max.Y - 4000), text3.str(), SVG::Color::RED, 20);
	std::ostringstream text4;
	text4 << "Hex.Width: " << m.hex_width;
	svg.writeText(Point(aabb.min.X, aabb.max.Y - 6000), text4.str(), SVG::Color::RED, 20);
	std::ostringstream text5;
	text5 << "Pattern.Height: " << m.pattern_height;
	svg.writeText(Point(aabb.min.X, aabb.max.Y - 8000), text5.str(), SVG::Color::RED, 20);
	std::ostringstream text6;
	text6 << "Offset.X: " << m.x_offset;
	svg.writeText(Point(aabb.min.X, aabb.max.Y - 10000), text6.str(), SVG::Color::RED, 20);
	std::ostringstream text7;
	text7 << "Offset.Y: " << m.y_offset;
	svg.writeText(Point(aabb.min.X, aabb.max.Y - 12000), text7.str(), SVG::Color::RED, 20);
	std::ostringstream text8;
	text8 << "Short.Y: " << m.y_short;
	svg.writeText(Point(aabb.min.X, aabb.max.Y - 14000), text8.str(), SVG::Color::RED, 20);
}

} // namespace cura