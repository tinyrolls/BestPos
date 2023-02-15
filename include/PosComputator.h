#ifndef POSCOMPUTATOR_H
#define POSCOMPUTATOR_H

#include "PosUKF.h"
#include "PosNLLS.h"
#include <unordered_map>
#include <pthread.h>

class PosComputator{
public:
    PosComputator(double inputFrames, bool inputHeightProcess);
    ~PosComputator();

    Eigen::VectorXd posCalculateFrequency(int inputTag, Eigen::MatrixXd AnchorsPos, Eigen::VectorXd AnchorsDis, double rates);

    Eigen::VectorXd posCalculate(int inputTag, Eigen::MatrixXd AnchorsPos, Eigen::VectorXd AnchorsDis);

    Eigen::VectorXd posUKFwithPos(int inputTag, Eigen::VectorXd posFirst);

    Eigen::VectorXd posNLLSwithAnchor(int inputTag, Eigen::MatrixXd AnchorsPos, Eigen::VectorXd AnnchorsDis);

    Eigen::VectorXd heightProcess(Eigen::VectorXd postPos, Eigen::MatrixXd AnchorsPos, Eigen::VectorXd AnchorsDis);

    void setOffset(double inputxoffset, double inputyoffset, double inputzoffset);

    void setHeightProcess(bool inputHeightProcess);

    static unordered_map<int, PosUKF> filtermap;
    static unordered_map<int, Eigen::VectorXd> lastPosMap;
    static double frames;
    static bool heightprocess;

    static double xoffset, yoffset, zoffset;

    pthread_mutex_t mutex;

};

#endif