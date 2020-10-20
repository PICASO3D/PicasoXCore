//Copyright (c) 2017 Ultimaker B.V.
//Copyright (c) 2020 PICASO 3D
//PicasoXCore is released under the terms of the AGPLv3 or higher

#include "PolygonsPointIndex.h"

namespace cura
{

bool PolygonsPointIndex::initialized() const
{
    return polygons;
}

}