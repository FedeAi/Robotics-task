//
// Created by federico on 05/05/22.
//

#ifndef TASK1_KINEMATIC_H
#define TASK1_KINEMATIC_H

#include <eigen3/Eigen/Core>
#include "task1/odometry/Integrator.h"

/**
 * Kinematic is an interface for building different types of kinematic models
 * @author Federico Sarrocco
 */
class Kinematic {
    /**
     * getter
     * @return last computed linear speed
     */
    virtual Eigen::Vector2d getLinearSpeed() {};

    /**
     * getter
     * @return last computed angular speed
     */
    virtual double getAngularSpeed() {};

    virtual Control getControl() {}
};


#endif //TASK1_KINEMATIC_H
