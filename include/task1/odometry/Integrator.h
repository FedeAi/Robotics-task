//
// Created by federico on 06/05/22.
//

#ifndef TASK1_INTEGRATOR_H
#define TASK1_INTEGRATOR_H

#include <eigen3/Eigen/Core>

class Pose {
public:
    Pose(Eigen::Vector2d position, double theta) : position_(position), theta_(theta) {}

    const Eigen::Vector2d &getPosition() const {
        return position_;
    }

    double getTheta() const {
        return theta_;
    };
private:
    Eigen::Vector2d position_;
    double theta_;

};

class Control {
public:
    Control(Eigen::Vector2d linearSpeed, const double angularSpeed) : linearSpeed_(linearSpeed),
                                                                      angularSpeed_(angularSpeed) {}

    const Eigen::Vector2d &getLinearSpeed() const {
        return linearSpeed_;
    }

    double getAngularSpeed() const {
        return angularSpeed_;
    };

private:
    Eigen::Vector2d linearSpeed_;
    const double angularSpeed_;
};


class Integrator {
public:
    virtual Pose updatePose(Pose startingPose, Control control, double deltaTime) {};
};

#endif //TASK1_INTEGRATOR_H
