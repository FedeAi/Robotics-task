//
// Created by federico on 04/05/22.
//

#ifndef TASK1_MECANUMMODEL_H
#define TASK1_MECANUMMODEL_H

#include <eigen3/Eigen/Core>
#include <memory>
#include <chrono>
#include "Integrator.h"

class Odometry {
public:

    Odometry();

    explicit Odometry(std::unique_ptr<Integrator> integrationMethod);

    void updatePosition(const Control &control);

    void updatePosition(const Control &control, double timestamp);


    const Pose &getPose() const;

    void resetPose(Pose pose);

private:
    Pose actualPose_;


private:
    std::unique_ptr<Integrator> integrationMethod_;
    double previousTime_;
};


#endif //TASK1_MECANUMMODEL_H
