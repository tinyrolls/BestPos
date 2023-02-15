#include "PosFilter.h"

int PosFilter::tagId_ = -1;
double PosFilter::dt_ = 0.5;
double PosFilter::processNoise_ = 0.1;
double PosFilter::measureNoise_ = 0.3;
double PosFilter::symmetryThreshold_ = 1e-11;

int PosFilter::step_= 0;
int PosFilter::posDimension_ = 3;

PosFilter::PosFilter(int inputTag) {
    tagId_ = inputTag;
}

PosFilter::~PosFilter() {}

void PosFilter::predict() {}

void PosFilter::update(Eigen::VectorXd inputPos) {
    step_ += 1;
}

Eigen::VectorXd PosFilter::getCurrentPos() {
    return Eigen::VectorXd::Zero(this->posDimension_);
}