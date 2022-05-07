//
// Created by federico on 04/05/22.
//

#ifndef TASK1_WHEELENCODERHELPER_H
#define TASK1_WHEELENCODERHELPER_H

#include <iostream>
#include <memory>

#include "task1/filtering/Filter.h"

/**
 * EncoderDelta is an helper class to keep track of elapsed time between encoder measurements and compute the delta in
 * number of steps
 */
class EncoderDelta {
public:
    EncoderDelta() = default;;

    void updateEncoder(double encoderValue, double timestamp);

    int getDeltaSteps() const;

    double getDeltaTime() const;

private:
    double lastEncoderValue_;
    double lastTime_;

    double deltaStep_;
    double deltaTime_;

};


/**
 * WheelSpeedCalculator is an helper class to compute wheel speed starting from encoder readings
 */
class WheelSpeedCalculator {
public:
    /**
     * WheelSpeedCalculator constructor
     * @param countsPerRevolution  encoder parameter
     */
    explicit WheelSpeedCalculator(double countsPerRevolution);

    /**
     * WheelSpeedCalculator constructor
     * @param countsPerRevolution encoder parameter
     * @param filter it's an optional parameter to filter encoder reading @see Filter
     */
    WheelSpeedCalculator(double countsPerRevolution, std::unique_ptr<Filter> filter);

    WheelSpeedCalculator() = default;;

    void computeWheelSpeed(double encoderValue, double timestamp);

    void setFilter(std::unique_ptr<Filter> filter);

    double getAngularSpeed() const;

private:
    double countsPerRevolution_, wheelAngularSpeed_{};
    EncoderDelta encoderState{};

    //! filter applied to the output
    std::unique_ptr<Filter> filter_;
};


#endif //TASK1_WHEELENCODERHELPER_H
