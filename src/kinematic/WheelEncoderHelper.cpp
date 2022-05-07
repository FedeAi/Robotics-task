//
// Created by federico on 04/05/22.
//
#include <cmath>
#include "task1/kinematic/WheelEncoderHelper.h"

WheelSpeedCalculator::WheelSpeedCalculator(double countsPerRevolution) : countsPerRevolution_(countsPerRevolution) {
    filter_ = std::make_unique<Filter>(Filter());
}

WheelSpeedCalculator::WheelSpeedCalculator(double countsPerRevolution, std::unique_ptr<Filter> filter)
        : countsPerRevolution_(countsPerRevolution) {
    filter_ = std::move(filter);
}

void WheelSpeedCalculator::computeWheelSpeed(double encoderValue, double timestamp) {
    encoderState.updateEncoder(encoderValue, timestamp);
    wheelAngularSpeed_ = encoderState.getDeltaSteps() / countsPerRevolution_ / encoderState.getDeltaTime() * 2 * M_PI;
    wheelAngularSpeed_ = filter_->update(wheelAngularSpeed_, timestamp);
}

double WheelSpeedCalculator::getAngularSpeed() const {
    return wheelAngularSpeed_;
}

void WheelSpeedCalculator::setFilter(std::unique_ptr<Filter> filter) {
    filter_ = std::move(filter);
}


void EncoderDelta::updateEncoder(double encoderValue, double timestamp) {

    deltaStep_ = encoderValue - lastEncoderValue_;
    deltaTime_ = timestamp - lastTime_;

    lastEncoderValue_ = encoderValue;
    lastTime_ = timestamp;

    if (deltaTime_ == 0.0) {
        // this can happen the first iteration
        deltaTime_ = 1e10;
    }
}

int EncoderDelta::getDeltaSteps() const {
    return deltaStep_;
}

double EncoderDelta::getDeltaTime() const {
    return deltaTime_;
}
