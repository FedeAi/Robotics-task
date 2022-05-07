//
// Created by federico on 04/05/22.
//

#include <iostream>
#include "task1/odometry/Odometry.h"
#include "task1/odometry/Euler.h"


void Odometry::updatePosition(const Control &control, double timestamp) {
    double deltaTime = timestamp - previousTime_;
    previousTime_ = timestamp;
    if (deltaTime > .1) {
        return; // unfeasible time
    }
    actualPose_ = integrationMethod_->updatePose(actualPose_, control, deltaTime);
}

void Odometry::updatePosition(const Control &control) {
    auto now = (double) std::chrono::time_point_cast<std::chrono::nanoseconds>(
            std::chrono::high_resolution_clock::now()).time_since_epoch().count() * 1e-9;
    updatePosition(control, now);
}

Odometry::Odometry(std::unique_ptr<Integrator> integrationMethod) : actualPose_(Pose({0, 0}, 0)) {
    previousTime_ = 0.0;
    integrationMethod_ = std::move(integrationMethod);
}

Odometry::Odometry() : actualPose_(Pose({0, 0}, 0)) {
    previousTime_ = 0.0;
    integrationMethod_ = std::make_unique<Euler>(Euler());
}


const Pose &Odometry::getPose() const {
    return actualPose_;
}

void Odometry::resetPose(Pose pose) {
    actualPose_ = pose;
}





