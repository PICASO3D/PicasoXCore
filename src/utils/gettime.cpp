/** Copyright (C) 2013 Ultimaker - Copyright (c) 2021 PICASO 3D - Released under terms of the AGPLv3 License */

#include "gettime.h"

namespace cura
{
    
TimeKeeper::TimeKeeper()
{
    restart();
}

double TimeKeeper::restart()
{
    double ret = getTime() - startTime;
    startTime = getTime();
    return ret;
}

}//namespace cura