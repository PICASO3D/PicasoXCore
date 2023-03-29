//Copyright (c) 2022 Ultimaker B.V.
//Copyright (c) 2022 PICASO 3D
//PicasoXCore is released under the terms of the AGPLv3 or higher

#include "PathOrderPath.h" //The definitions we're implementing here.
#include "sliceDataStorage.h" //For SliceLayerPart.
#include "WallToolPaths.h"

namespace cura
{

    template<>
    ConstPolygonRef PathOrderPath<ConstPolygonPointer>::getVertexData()
    {
        return *vertices;
    }

    template<>
    ConstPolygonRef PathOrderPath<PolygonPointer>::getVertexData()
    {
        return *vertices;
    }

    template<>
    ConstPolygonRef PathOrderPath<const SkinPart*>::getVertexData()
    {
        return vertices->outline.outerPolygon();
    }

    template<>
    ConstPolygonRef PathOrderPath<const SliceLayerPart*>::getVertexData()
    {
        return vertices->outline.outerPolygon();
    }

    template<>
    ConstPolygonRef PathOrderPath<const SupportInfillPart*>::getVertexData()
    {
        return vertices->outline.outerPolygon();
    }
    template<>
    ConstPolygonRef PathOrderPath<const ExtrusionLine*>::getVertexData()
    {
        if ( ! cached_vertices)
        {
            // <PicasoAddon>
            seam_idx = vertices->seam_idx;
            unlink_last_point = vertices->unlink_last_point;
            // </PicasoAddon>
            cached_vertices = vertices->toPolygon();
        }
        return ConstPolygonRef(*cached_vertices);
    }

}
