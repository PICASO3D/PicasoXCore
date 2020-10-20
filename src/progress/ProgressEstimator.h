/** Copyright (C) 2016 Tim Kuipers - Copyright (c) 2020 PICASO 3D - Released under terms of the AGPLv3 License */

#ifndef PROGRESS_PROGRESS_ESTIMATOR_H
#define PROGRESS_PROGRESS_ESTIMATOR_H

#include <vector>

namespace cura
{
/*
 * ProgressEstimator is a finger-tree with ProgressEstimatorLinear as leaves.
 * 
 * Each (non-leaf) node consists of a ProgressStageEstimator which consists of several stages.
 * 
 * The structure of this tree is an oversimplification of the call graph of PicasoXCore.
 * 
 */
    
class ProgressEstimator
{
public:
    virtual double progress(int current_step) = 0;
    virtual ~ProgressEstimator()
    {
    }
};

} // namespace cura

#endif // PROGRESS_PROGRESS_ESTIMATOR_H