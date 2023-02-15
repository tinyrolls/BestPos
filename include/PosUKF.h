#ifndef POSUKF_H
#define POSUKF_H

#include "PosFilter.h"

class PosUKF:public PosFilter {
public:
    PosUKF(int inputTag, Eigen::VectorXd postPos);
    ~PosUKF();

    void predict();
    void update(Eigen::VectorXd inputZ);
    Eigen::VectorXd getCurrentPos();

    pair<Eigen::VectorXd, Eigen::MatrixXd> unscentedTransform(Eigen::MatrixXd inputSigma, Eigen::MatrixXd errMatrix);
    Eigen::MatrixXd processModel(Eigen::MatrixXd inputSigma);
    Eigen::MatrixXd sigmaFunction(Eigen::VectorXd inputX, Eigen::MatrixXd inputP);
    void weightFunction();
    Eigen::MatrixXd stateToMeasure(Eigen::MatrixXd inputSigma);

    Eigen::MatrixXd A, Q, R;
    Eigen::MatrixXd P, Pp;
    Eigen::VectorXd x, xp;
    Eigen::MatrixXd sigmaF, sigmaH;
    Eigen::VectorXd Wc, Wm;

    static double lambda;
    static double alpha, beta, kappa;
    static int stateDimension;
    static int measureDimension;
};

#endif