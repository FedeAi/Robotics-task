//
// Created by federico on 04/05/22.
//

#ifndef TASK1_MECANUMKINEMATIC_H
#define TASK1_MECANUMKINEMATIC_H

#include <eigen3/Eigen/Core>

#include "task1/kinematic/WheelEncoderHelper.h"
#include "task1/kinematic/Kinematic.h"

/**
 * MecanumKinematic class describes the kinematic of a mecanum like robot. It can be used to compute from
 *  wheelSpeeds[w, vx, vy] -> robotSpeed
 * or from robotSpeed[w, vx, vy] -> wheelsSpeed
 * @author Federico Sarrocco
 */
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

    /**
     * MecanumKinematic constructor
     */
    MecanumKinematic();

    /**
     * forward kinematic, computeKinematic starts from wheel state and computes robot angular and linear speed
     * @param wheelSpeeds [speedFL, speedFR, speedRR, speedRL]
     */
    void computeKinematic(const Eigen::Vector4d &wheelSpeeds);

    /**
     * reverse kinematic, starting from desired control angular and linear speed, it computes wheel speeds
     * @param control
     */
    void computeWheelsSpeed(Control control);

    /**
     * getLinearSpeed available after having called computeKinematic
     * @see computeKinematic
     * @return linear robot speed [vx,vy]
     */
    Eigen::Vector2d getLinearSpeed() override;

    /**
     * getAngularSpeed available after having called computeKinematic
     * @see computeKinematic
     * @return angular robot speed [w]
     */
    double getAngularSpeed() override;

    /**
     * getControl available after having called computeKinematic
     * @see computeKinematic
     * @return both linear speed and angular speed
     */
    Control getControl() override;

    /**
     * getWheelsControlSpeed available after having called computeWheelsSpeed
     * @see computeWheelsSpeed
     * @return wheels speed  [speedFL, speedFR, speedRR, speedRL]
     */
    Eigen::Vector4d getWheelsControlSpeed();

private:
    //! pseudo-inverse matrix used for wheelsSpeed[ w, vx, vy] -> robotSpeed conversion
    Eigen::Matrix<double, 3, 4> H_0p;
    //! matrix used for robotSpeed[ w, vx, vy] -> wheelsSpeed conversion
    Eigen::Matrix<double, 4, 3> H_0;

    //! robot speed [w, vx, vy]
    Eigen::Vector3d V_b;
    //! robot parameters
    double wheelRadius_, wheelX_, wheelY_, gearRatio_;
    //! wheels speed, computed starting from control in [w ,vx ,vy]
    Eigen::Vector4d wheelSpeeds_;
    void buildH_0p();
    void buildH_0();

};


#endif //TASK1_MECANUMKINEMATIC_H
