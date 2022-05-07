//
// Created by federico on 07/05/22.
//

#ifndef TASK1_RUNGEKUTTA4_H
#define TASK1_RUNGEKUTTA4_H

#include "Integrator.h"

/**
 * RK4
 */
class Rungekutta4 : public Integrator {
public:
    Pose updatePose(Pose startingPose, Control control, double deltaTime) override;

private:
    Eigen::Matrix2d computeRotationMatrix(double theta);
};

#endif //TASK1_RUNGEKUTTA4_H
