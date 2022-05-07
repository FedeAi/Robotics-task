//
// Created by federico on 04/05/22.
//

#include "task1/odometry/Euler.h"

Pose Euler::updatePose(Pose startingPose, Control control, double deltaTime) {
    Eigen::Matrix<double, 2, 2> motion;
    motion << cos(startingPose.getTheta()), -sin(startingPose.getTheta()), // TODO rename
            sin(startingPose.getTheta()), cos(startingPose.getTheta());

    double theta = startingPose.getTheta() + control.getAngularSpeed() * deltaTime;
    theta =  atan2(sin(theta),cos(theta)); // pi2pi function
    Eigen::Vector2d position = startingPose.getPosition() + deltaTime * motion * control.getLinearSpeed();
    return {position, theta};
}
