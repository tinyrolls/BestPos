#include "PosNLLS.h"

// 迭代次数
double PosNLLS::epsilon_ = 1e-7;
int PosNLLS::MAXITERATIONS_ = 1000;
int PosNLLS::stateDimension_ = 6;
int PosNLLS::measureDimension_ = 4;


PosNLLS::PosNLLS(int inputTag, Eigen::MatrixXd inputPos, Eigen::VectorXd inputDis):PosFilter(inputTag) {
    measureDimension_ = inputDis.size();
    stateDimension_ = this->posDimension_;

    if (inputDis.size() < 2 || inputPos.rows() != inputDis.size()) {
        throw "Argument Error";
    }

    for (int i = 0; i < inputDis.size(); i++) {
        inputDis(i) = max(inputDis(i), epsilon_);
    }

    this->anchorsPos = inputPos;
    this->anchorsDis = inputDis;
}

PosNLLS::~PosNLLS() {}

Eigen::VectorXd PosNLLS::getCurrentPos() {
    return this->getPosResult();
}
Eigen::VectorXd PosNLLS::getPosResult() {
    Eigen::VectorXd initialStart = Eigen::VectorXd::Zero(stateDimension_);
    
    // 构建的坐标数组，距离数组，初始计算点
    for (int i = 0; i < measureDimension_; i++) {
        for (int j = 0; j < stateDimension_; j++) {
            initialStart(j) += this->anchorsPos(i,j);
        }
    }
    for (int i = 0; i < stateDimension_; i++) {
        initialStart(i) /= measureDimension_;
    } 

    lmder_functor functor(this->anchorsPos, this->anchorsDis);
    Eigen::LevenbergMarquardt<lmder_functor> lm(functor);
    int info = lm.minimize(initialStart);
    
    return initialStart;
}

