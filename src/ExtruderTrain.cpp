/** Copyright (C) 2016 Ultimaker - Copyright (c) 2020 PICASO 3D - Released under terms of the AGPLv3 License */

#include "ExtruderTrain.h"

namespace cura 
{

ExtruderTrain::ExtruderTrain(const size_t extruder_nr, Settings* parent_settings) : extruder_nr(extruder_nr)
{
    settings.setParent(parent_settings);
}

}//namespace cura