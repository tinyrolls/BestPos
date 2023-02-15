#include "PosComputator.h"

unordered_map<int, PosUKF> PosComputator::filtermap;
unordered_map<int, Eigen::VectorXd> PosComputator::lastPosMap;
double PosComputator::frames = 2;
bool PosComputator::heightprocess = true;

double PosComputator::xoffset = 0;
double PosComputator::yoffset = 0;
double PosComputator::zoffset = 0;

PosComputator::PosComputator(double inputFrames, bool inputHeightProcess){
    PosComputator::frames = inputFrames;
    PosComputator::heightprocess = inputHeightProcess;
    pthread_mutex_init(&(PosComputator::mutex), NULL);
}
PosComputator::~PosComputator(){}

Eigen::VectorXd PosComputator::posCalculateFrequency(int inputTag, Eigen::MatrixXd AnchorsPos, Eigen::VectorXd AnchorsDis, double rates) {
    
    PosFilter::dt_ = PosFilter::dt_/rates;
    PosFilter::symmetryThreshold_ = 1e-10;
    PosFilter::posDimension_ = AnchorsPos.row(0).size();

    Eigen::VectorXd currentPos = posNLLSwithAnchor(inputTag, AnchorsPos, AnchorsDis);
    Eigen::VectorXd lastPos(PosFilter::posDimension_);
    Eigen::VectorXd eachFrameResultPos(PosFilter::posDimension_);


    if (lastPosMap.find(inputTag) == lastPosMap.end()) {
        pthread_mutex_lock(&(PosComputator::mutex));
        lastPosMap.insert({inputTag, currentPos});
        pthread_mutex_unlock(&(PosComputator::mutex));
        lastPos = currentPos;
    } else {
        lastPos = lastPosMap.find(inputTag)->second;
    }

    for (int i = 0; i < rates; i++) {
        Eigen::VectorXd processPos(PosFilter::posDimension_);
        for (int j = 0; j < PosFilter::posDimension_; j++) {
            processPos(j) = lastPos(j) + (currentPos(j) - lastPos(j))/(rates * 1.0) * (i + 1);
        }
        eachFrameResultPos = posUKFwithPos(inputTag, processPos);
    }

    pthread_mutex_lock(&(PosComputator::mutex));
    lastPosMap.insert({inputTag, currentPos});
    pthread_mutex_unlock(&(PosComputator::mutex));

    return eachFrameResultPos;
}

Eigen::VectorXd PosComputator::posCalculate(int inputTag, Eigen::MatrixXd AnchorsPos, Eigen::VectorXd AnchorsDis) {
    
    // 插帧参数
    Eigen::VectorXd posResult = posCalculateFrequency(inputTag, AnchorsPos, AnchorsDis, frames);
    
    // 高度处理
    if (heightprocess) {
        posResult = heightProcess(posResult, AnchorsPos, AnchorsDis);
    }

    // 基准校正
    posResult(0) += xoffset;
    posResult(1) += yoffset;
    
    if (AnchorsDis.size() == 3) {
        posResult(2) = 0 + zoffset;
    } else {
        posResult(2) += zoffset;
    }
    
    return posResult;
}

Eigen::VectorXd PosComputator::posUKFwithPos(int inputTag, Eigen::VectorXd posFirst) {
    if (filtermap.find(inputTag) == filtermap.end()) {
        PosUKF newfilter(inputTag, posFirst);
        newfilter.predict();
        newfilter.update(posFirst);
        filtermap.insert({inputTag, newfilter});
    } else {
        filtermap.find(inputTag)->second.predict();
        filtermap.find(inputTag)->second.update(posFirst);
    }

    return filtermap.find(inputTag)->second.getCurrentPos();
}

Eigen::VectorXd PosComputator::posNLLSwithAnchor(int inputTag, Eigen::MatrixXd AnchorsPos, Eigen::VectorXd AnchorsDis) {
    return PosNLLS(inputTag, AnchorsPos, AnchorsDis).getCurrentPos();
}

Eigen::VectorXd PosComputator::heightProcess(Eigen::VectorXd postPos, Eigen::MatrixXd AnchorsPos, Eigen::VectorXd AnchorsDis) {
    double heightSum = 0;
    for (int i = 0; i < AnchorsDis.size(); i++) {
        double deltaX = pow(postPos(0) - AnchorsPos(i,0), 2);
        double deltaY = pow(postPos(1) - AnchorsPos(i,1), 2);

        double heightdelta = sqrt(abs(pow(AnchorsDis(i), 2) - deltaX - deltaY));
        heightSum += AnchorsPos(i, 2) - heightdelta;
    }
    double height = heightSum / AnchorsDis.size();
    Eigen::VectorXd processedResult(3);
    processedResult << postPos(0), postPos(1), height;
    return processedResult;
}

void PosComputator::setOffset(double inputxoffset, double inputyoffset, double inputzoffset){
    xoffset = inputxoffset;
    yoffset = inputyoffset;
    zoffset = inputzoffset;
}

void PosComputator::setHeightProcess(bool inputHeightProcess) {
    heightprocess = inputHeightProcess;
}