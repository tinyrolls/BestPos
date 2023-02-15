#ifndef POSFILTER_H
#define POSFILTER_H

#include <iostream>
#include <vector>
#include <algorithm>
#include "Eigen/Dense"

using namespace std;

class PosFilter {
public:
    PosFilter(int inputTag);
    ~PosFilter();
    void predict();
    void update(Eigen::VectorXd inputPos);
    Eigen::VectorXd getCurrentPos();

    static int tagId_;
    static double dt_;
    static double processNoise_;
    static double measureNoise_;
    static double symmetryThreshold_;

    static int step_;
    static int posDimension_;

};


#endif