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
     * getLinearSpeed available after having called computeKinematic
     * @see computeKinematic
     * @return linear robot speed [vx,vy]
     */
    virtual Eigen::Vector2d getLinearSpeed() {};

    /**
     * getAngularSpeed available after having called computeKinematic
     * @see computeKinematic
     * @return angular robot speed [w]
     */
    virtual double getAngularSpeed() {};

    /**
     * getControl available after having called computeKinematic
     * @see computeKinematic
     * @return both linear speed and angular speed
     */
    virtual Control getControl() {}
};


#endif //TASK1_KINEMATIC_H
