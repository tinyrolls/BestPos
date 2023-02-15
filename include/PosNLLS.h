#ifndef POSNLLS_H
#define POSNLLS_H

#include "PosFilter.h"
#include "PosLM.hpp"

#include "unsupported/Eigen/LevenbergMarquardt"

class PosNLLS:public PosFilter {
public:
    PosNLLS(int inputTag, Eigen::MatrixXd inputPos, Eigen::VectorXd inputDis);
    ~PosNLLS();
    Eigen::VectorXd getCurrentPos();
    Eigen::VectorXd getPosResult();

private:
    // 非零判断
    static double epsilon_;

    // 迭代次数
    static int MAXITERATIONS_;
    static int stateDimension_;
    static int measureDimension_;

    Eigen::MatrixXd anchorsPos;
    Eigen::VectorXd anchorsDis;
};


#endif