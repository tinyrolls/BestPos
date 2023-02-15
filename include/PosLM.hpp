#include <iostream>
#include "unsupported/Eigen/LevenbergMarquardt"

struct lmder_functor : Eigen::DenseFunctor<double>
{
    lmder_functor(Eigen::MatrixXd inputPos, Eigen::VectorXd inputDis): DenseFunctor<double>(inputPos.cols(), inputDis.size()) {
        this->anchorsPos = inputPos;
        this->anchorsDis = inputDis;
    }
    /**
     * @brief 雅可比矩阵迭代计算
     * 
     * @param x tempPos，中间坐标
     * @param fvec function vecotr
     * @return int 
     */
    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
    {
        // inputs = 3, values = 4
        Eigen::VectorXd posVector = x;
        for (int i = 0; i < values(); i++)
        {
            fvec[i] = 0;
            for (int j = 0; j < inputs(); j++) {
                fvec[i] += pow(posVector[j] - this->anchorsPos(i,j), 2);
            }
            fvec[i] -= pow(this->anchorsDis[i], 2);
        }
        return 0;
    }

    /**
     * @brief 从单向量求雅可比矩阵
     * 
     * @param x 单向量
     * @param fjac 雅可比矩阵
     * @return int 
     */
    int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const
    {
        Eigen::VectorXd posVector = x;
        for (int i = 0; i < values(); i++) {
            for (int j = 0; j < inputs(); j++) {
                fjac(i,j) = 2 * posVector[j] - 2 * this->anchorsPos(i,j);
            }
        }
        return 0;
    }

    Eigen::MatrixXd anchorsPos;
    Eigen::VectorXd anchorsDis;
};

