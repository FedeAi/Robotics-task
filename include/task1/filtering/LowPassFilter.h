//
// Created by federico on 06/05/22.
//

#ifndef TASK1_LOWPASSFILTER_H
#define TASK1_LOWPASSFILTER_H

#include <chrono>  // for high_resolution_clock
#include "Filter.h"

/**
 * Derivative discrete filter G(s)=Ks/(Ts+1)
 * @author Federico Sarrocco
 */
class LowPassFilter : public Filter {
public:
    /**
     *
     * @param K gain
     * @param T time constant
     * @param u0 initial input to the block
     */
    LowPassFilter(double K, double T, double u0);

    /**
     *
     * @param input
     * @return filter output
     */
    double update(double input) override;

    /**
     *
     * @param input
     * @param timestamp
     * @return filter output
     */
    double update(double input, double timestamp) override;

private:
    double state_, K_, T_;
    double previousTime_;

};


#endif //TASK1_LOWPASSFILTER_H
