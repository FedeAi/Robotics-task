//
// Created by federico on 07/05/22.
//
#include "task1/odometry/RungeKutta4.h"

Pose Rungekutta4::updatePose(Pose startingPose, Control control, double deltaTime) {
    // Runge kutta 4th order integration

    // First point evaluation
    auto k1_p = deltaTime * computeRotationMatrix(startingPose.getTheta()) * control.getLinearSpeed();
    auto k1_r = deltaTime * control.getAngularSpeed();

    // Second point evaluation
    auto k2_p = deltaTime * computeRotationMatrix(startingPose.getTheta() + k1_r / 2.0) * control.getLinearSpeed();
    auto k2_r = deltaTime * control.getAngularSpeed();

    // Third point evaluation
    auto k3_p = deltaTime * computeRotationMatrix(startingPose.getTheta() + k2_r / 2.0) * control.getLinearSpeed();
    auto k3_r = deltaTime * control.getAngularSpeed();

    // Fourth point evaluation
    auto k4_p = deltaTime * computeRotationMatrix(startingPose.getTheta() + k3_r) * control.getLinearSpeed();
    auto k4_r = deltaTime * control.getAngularSpeed();

    // Compute integral: combining
    auto position = startingPose.getPosition() + (k1_p + 2 * k2_p + 2 * k3_p + k4_p) / 6.0;
    double theta =
            startingPose.getTheta() + (k1_r + 2 * k2_r + 2 * k3_r + k4_r) / 6.0; // rk4 is useless for theta (exact)

    return {position, theta};
}

Eigen::Matrix2d Rungekutta4::computeRotationMatrix(double theta) {
    Eigen::Matrix2d rotation;
    rotation << cos(theta), -sin(theta),
            sin(theta), cos(theta);
    return rotation;
}

