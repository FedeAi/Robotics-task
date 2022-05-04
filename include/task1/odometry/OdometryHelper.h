//
// Created by federico on 04/05/22.
//

#ifndef TASK1_ODOMETRYHELPER_H
#define TASK1_ODOMETRYHELPER_H


/**
 * EncoderDelta is an helper class to keep track of elapsed time between encoder measurements and compute the delta in
 * number of steps
 */
class EncoderDelta{
public:
    void updateEncoder(int encoderValue, double timestamp);
    int getDeltaSteps();
    double getDeltaTime();

private:
    int lastEncoderValue_;
    double lastTime_;

    int deltaStep_;
    double deltaTime_;

};


/**
 * WheelSpeedCalculator is an helper class to compute wheel speed starting from encoder readings
 */
class WheelSpeedCalculator{
public:
    WheelSpeedCalculator(double gearRatio, double wheelRadius);
    double computeWheelSpeed(int encoderValue, double timestamp);

private:
    double gearRatio_, wheelRadius_;
    EncoderDelta encoderState;
};



#endif //TASK1_ODOMETRYHELPER_H
