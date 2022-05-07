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

    Odometry(std::unique_ptr<Integrator> integrationMethod, double theta0, double x0, double y0);

    void updatePosition(const Control &control);

    void updatePosition(const Control &control, double timestamp);


    const Pose &getPose() const;

    void resetPose(Pose pose);
    void setIntegrationMethod(std::unique_ptr<Integrator> integrationMethod);

private:
    Pose actualPose_;

    std::unique_ptr<Integrator> integrationMethod_;
    double previousTime_;
};


#endif //TASK1_MECANUMMODEL_H
