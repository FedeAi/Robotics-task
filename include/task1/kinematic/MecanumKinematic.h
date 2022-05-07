//
// Created by federico on 04/05/22.
//

#ifndef TASK1_MECANUMKINEMATIC_H
#define TASK1_MECANUMKINEMATIC_H

#include <eigen3/Eigen/Core>

#include "task1/kinematic/WheelEncoderHelper.h"
#include "task1/kinematic/Kinematic.h"

class MecanumKinematic : public Kinematic {
public:

    /**
     * MecanumKinematic constructor
     * @param wheelRadius
     * @param gearRatio
     * @param wheelX Wheel position along x (±𝑙)
     * @param wheelY Wheel position along y (±w)
     */
    MecanumKinematic(double wheelRadius, double gearRatio, double wheelX, double wheelY);

    MecanumKinematic();

    /**
     * computeKinematic method updates the internal state, so it computes angular and linear speed
     *  double speedFL, double speedFR, double speedRR, double speedRL  TODO FIX description
     */
    void computeKinematic(const Eigen::Vector4d &wheelSpeeds);


    Eigen::Vector2d getLinearSpeed() override;

    double getAngularSpeed() override;

    Control getControl() override;

private:
    Eigen::Matrix<double, 3, 4> H_0p;
    Eigen::Vector3d V_b;
    double wheelRadius_, wheelX_, wheelY_, gearRatio_;

    void buildH_0();

};


#endif //TASK1_MECANUMKINEMATIC_H
