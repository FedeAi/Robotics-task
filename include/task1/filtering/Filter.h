//
// Created by federico on 06/05/22.
//

#ifndef TASK1_FILTER_H
#define TASK1_FILTER_H

class Filter {
public:
    virtual double update(double input) { return input; }

    virtual double update(double input, double timestamp) { return input; }
};

#endif //TASK1_FILTER_H
