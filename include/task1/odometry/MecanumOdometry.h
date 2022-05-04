//
// Created by federico on 04/05/22.
//

#ifndef TASK1_MECANUMODOMETRY_H
#define TASK1_MECANUMODOMETRY_H

#include "task1/odometry/OdometryHelper.h"

class MecanumOdometry {
public:

    /**
     * updateOdometry method updates the internal state, so it computes angular and linear speed, TODO and position?
     * @param encoderFL
     * @param encoderFR
     * @param encoderRL
     * @param encoderRR
     * @param timestamp
     */
    void updateOdometry(int encoderFL, int encoderFR, int encoderRL, int encoderRR, double timestamp);

    /**
     * getW returns the last computed angular speed
     */
    double getW();

    /**
     * getV returns the last computed linear speed
     * @return
     */
    double getV();
    // getPosition?

private:
    WheelSpeedCalculator encoderFL_, encoderFR_, encoderRL, encoderRR;
    double W;
    double V;

};


#endif //TASK1_MECANUMODOMETRY_H
