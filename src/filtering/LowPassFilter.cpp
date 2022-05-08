//
// Created by federico on 06/05/22.
//

#include <iostream>
#include "task1/filtering/LowPassFilter.h"

LowPassFilter::LowPassFilter(double K, double T, double u0) : K_(K), T_(T), state_(u0) {
    previousTime_ = 0.0;
}

double LowPassFilter::update(double input) {
    auto time = (double) std::chrono::time_point_cast<std::chrono::nanoseconds>(
            std::chrono::high_resolution_clock::now()).time_since_epoch().count() * 1e-9;
    return update(input, time);
}

double LowPassFilter::update(double input, double timestamp) {
    auto Ts_ = timestamp - previousTime_;

    auto a = Ts_ / (T_ + Ts_);
    double output = input * a + state_ * (1 - a);
    state_ = output;

    previousTime_ = timestamp;
    return output;

}
