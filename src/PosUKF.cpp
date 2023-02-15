#include "PosUKF.h"

double PosUKF::alpha = 0.1;
double PosUKF::beta = 2;
double PosUKF::kappa = 0;
double PosUKF::lambda = 0;

int PosUKF::stateDimension = 6;
int PosUKF::measureDimension = 3;

PosUKF::PosUKF(int inputTag, Eigen::VectorXd postPos):PosFilter(inputTag) {        
        // x = new ArrayEigen::VectorXd(new double[]{1,1,1,1,1,1});
        this->stateDimension = this->posDimension_*2;
        this->measureDimension = postPos.size();

        Eigen::VectorXd InitialX(stateDimension);
        InitialX << postPos(0),0.0,postPos(1),0.0,postPos(2),0.0;
        this->x = InitialX;
        this->Q = Eigen::MatrixXd::Identity(stateDimension,stateDimension) * pow(processNoise_, 2);
        this->R = Eigen::MatrixXd::Identity(measureDimension, measureDimension) * pow(measureNoise_, 2);
        this->P = Eigen::MatrixXd::Identity(stateDimension,stateDimension);

        weightFunction();
        
}

PosUKF::~PosUKF() {}

void PosUKF::predict() {

    Eigen::MatrixXd sigma = sigmaFunction(x, P);
    sigmaF = processModel(sigma);
    pair<Eigen::VectorXd, Eigen::MatrixXd> result = unscentedTransform(sigmaF, Q);  
    xp = result.first;
    Pp = result.second;
    
}

void PosUKF::update(Eigen::VectorXd inputZ) {

    PosFilter::update(inputZ);

    Eigen::VectorXd z = inputZ;
    sigmaH = stateToMeasure(sigmaF);

    pair<Eigen::VectorXd, Eigen::MatrixXd> result = unscentedTransform(sigmaH, R); 
    Eigen::VectorXd zp = result.first;
    Eigen::MatrixXd Pz = result.second;

    Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(stateDimension,measureDimension);
    for (int i = 0; i < sigmaH.rows(); i++){
        Eigen::MatrixXd outResult = (sigmaF.row(i).transpose() - xp) * (sigmaH.row(i).transpose() - zp).transpose();
        Pxz = Pxz + outResult * Wc(i);
    }

    Eigen::MatrixXd K = Pxz * Pz.inverse();

    x = xp + K * (z - zp);
    P = Pp - K * Pz * K.transpose();
}

Eigen::VectorXd PosUKF::getCurrentPos(){
    Eigen::VectorXd zerobase(posDimension_);
    zerobase << x(0),x(2),x(4);
    return zerobase;
}


pair<Eigen::VectorXd, Eigen::MatrixXd> PosUKF::unscentedTransform(Eigen::MatrixXd inputSigma, Eigen::MatrixXd errMatrix){
    
    Eigen::VectorXd newX = Wm.transpose() * inputSigma;

    Eigen::MatrixXd newP = Eigen::MatrixXd::Zero(inputSigma.cols(), inputSigma.cols());

    for (int i = 0; i < inputSigma.rows(); i++) {
        Eigen::VectorXd y = inputSigma.row(i).transpose() - newX;
        newP = newP + y * y.transpose() * Wc(i);
    }

    newP = newP + errMatrix;

    return make_pair(newX,newP);
}

Eigen::MatrixXd PosUKF::processModel(Eigen::MatrixXd inputSigma) {
    Eigen::MatrixXd resultSigma(inputSigma.rows(),inputSigma.cols());

    for (int i = 0; i < inputSigma.rows(); i++) {
        Eigen::VectorXd currentState = inputSigma.row(i);
        currentState(0) = currentState(0)+currentState(1)*dt_;
        currentState(2) = currentState(2)+currentState(3)*dt_;
        currentState(4) = currentState(4)+currentState(5)*dt_;
        
        resultSigma.row(i) = currentState;
    }
    return resultSigma;
}

Eigen::MatrixXd PosUKF::sigmaFunction(Eigen::VectorXd inputX, Eigen::MatrixXd inputP) {
    Eigen::MatrixXd sigmas(2*inputX.size()+1, inputX.size());
    Eigen::MatrixXd U = (inputP * (inputX.size()+lambda)).llt().matrixL();

    for (int i = 0; i < inputX.size(); i++) {
        if (i == 0) sigmas.row(0) = inputX;

        sigmas.row(i+1) = inputX.transpose() + U.row(i);
        sigmas.row(stateDimension+i+1) = inputX.transpose() - U.row(i);
    }

    return sigmas;
}

void PosUKF::weightFunction() {
    lambda = pow(alpha, 2) * (stateDimension+kappa) - stateDimension;
    Wc = Eigen::VectorXd::Zero(2*stateDimension+1).array() +  1/(2*(stateDimension+lambda)); 
    Wm = Eigen::VectorXd::Zero(2*stateDimension+1).array() + 1/(2*(stateDimension+lambda)); 
    Wc(0) = lambda/(stateDimension+lambda) + (1 - pow(alpha, 2) + beta);
    Wm(0) = lambda/(stateDimension+lambda);
} 

Eigen::MatrixXd PosUKF::stateToMeasure(Eigen::MatrixXd inputSigma) {

    Eigen::MatrixXd resultH = Eigen::MatrixXd::Zero(inputSigma.rows(), measureDimension);
    for (int i = 0; i < inputSigma.rows(); i++) {
        Eigen::VectorXd posState = inputSigma.row(i);
        Eigen::VectorXd currentDis(posDimension_);
        currentDis << posState(0), posState(2), posState(4);
        resultH.row(i) = currentDis;
    }
    return resultH;
}

