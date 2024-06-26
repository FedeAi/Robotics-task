//
// Created by federico on 04/05/22.
//

#include "task1/kinematic/MecanumKinematic.h"

MecanumKinematic::MecanumKinematic(double wheelRadius, double gearRatio, double wheelX, double wheelY) :
        wheelRadius_(wheelRadius),
        gearRatio_(gearRatio),
        wheelX_(wheelX),
        wheelY_(wheelY) {
    buildH_0p();
    buildH_0();
}


void MecanumKinematic::computeKinematic(const Eigen::Vector4d &wheelSpeeds) {
    V_b = H_0p * wheelSpeeds * wheelRadius_ / gearRatio_ / 4.0;
}

void MecanumKinematic::computeWheelsSpeed(Control control) {
    Eigen::Vector3d controlV;
    controlV << control.getAngularSpeed(), control.getLinearSpeed().x(), control.getLinearSpeed().y();
    wheelSpeeds_ = H_0 * controlV / wheelRadius_ * gearRatio_;
}


void MecanumKinematic::buildH_0p() {
    double fact = 1.0 / (wheelX_ + wheelY_);
    H_0p <<
         -fact, fact, fact, -fact,
            1, 1, 1, 1,
            -1, 1, -1, 1;
}

void MecanumKinematic::buildH_0() {
    double fact = wheelY_ + wheelX_;
    H_0 <<
        -fact,  1,  -1,
        fact,   1,  1,
        fact,   1,  -1,
        -fact,  1,  1;
}

Eigen::Vector2d MecanumKinematic::getLinearSpeed() {
    return V_b.tail(2);
}

double MecanumKinematic::getAngularSpeed() {
    return V_b(0);
}

MecanumKinematic::MecanumKinematic() {
    wheelRadius_ = 0;
    wheelX_ = 0;
    wheelY_ = 0;
    gearRatio_ = 0;
}

Control MecanumKinematic::getControl() {
    return Control(getLinearSpeed(), getAngularSpeed());
}

Eigen::Vector4d MecanumKinematic::getWheelsControlSpeed() {
    return wheelSpeeds_;
}


