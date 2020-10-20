//Copyright (c) 2020 PICASO 3D
//PicasoXCore is released under the terms of the AGPLv3 or higher

#include "../utils/Coord_t.h"
#include "../utils/AABB.h"
#include "../utils/polygon.h"

namespace cura
{

	class Polygons;

	typedef double coordf_t;

	struct HexPattern
	{
		coord_t	distance;
		coord_t hex_side;
		coord_t hex_width;
		coord_t	pattern_height;
		coord_t y_short;
		coord_t x_offset;
		coord_t	y_offset;
		Point	hex_center;
	};

	class PicasoHoneyCombInfill
	{
	public:
		PicasoHoneyCombInfill();

		~PicasoHoneyCombInfill();

		static void generateHoneyCombInfill(Polygons& result_lines, coord_t outline_offset, coord_t infill_line_width, coord_t line_distance, const Polygons& in_outline, coord_t z, int layer_nr, bool simple, AngleDegrees fill_angle);

		//static void generateSimpleHoneyCombInfill(Polygons& result_lines, bool zig_zaggify, coord_t outline_offset, coord_t infill_line_width, coord_t line_distance, const Polygons& in_outline, coord_t z, const AABB mesh_bounding_box);

	private:

		static void debugSVG(std::string filename, const AABB& outline_bounding_box, const AABB mesh_bounding_box, Polygons& lines, HexPattern& m);

		/*
		* https://github.com/prusa3d/PrusaSlicer/blob/d9b764bd10239c866e24ff5155f4fa9a86c0e850/src/libslic3r/Fill/FillBase.hpp
		*/

		// Align a coordinate to a grid. The coordinate may be negative,
		// the aligned value will never be bigger than the original one.
		static coord_t _align_to_grid(const coord_t coord, const coord_t spacing) {
			// Current C++ standard defines the result of integer division to be rounded to zero,
			// for both positive and negative numbers. Here we want to round down for negative
			// numbers as well.
			coord_t aligned = (coord < 0) ?
				((coord - spacing + 1) / spacing) * spacing :
				(coord / spacing) * spacing;
			assert(aligned <= coord);
			return aligned;
		}
		static Point   _align_to_grid(Point coord, Point spacing)
		{
			return Point(_align_to_grid(coord.X, spacing.X), _align_to_grid(coord.Y, spacing.Y));
		}
		static coord_t _align_to_grid(coord_t coord, coord_t spacing, coord_t base)
		{
			return base + _align_to_grid(coord - base, spacing);
		}
		static Point   _align_to_grid(Point coord, Point spacing, Point base)
		{
			return Point(_align_to_grid(coord.X, spacing.X, base.X), _align_to_grid(coord.Y, spacing.Y, base.Y));
		}
	};

} // namespace cura

