#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "factorgraph.h"
//#include "/home/valerio/SLAM_Project/SparseMatrix/SparseMatrix.h"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/SparseQR>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

typedef Eigen::Triplet<double> T;

class Optimizer
{
public:
    Optimizer();
    Optimizer(FactorGraph *g);
    double getGlobalError(){ return _globalError; }
    void writeOptGraph(MatrixXd newmeans);
     void run(MatrixXd vmeans, int n);

protected:
    int _iteration;
    std::vector<T> tripletList;
    double _globalError;
    FactorGraph _graph;
    VectorXd _b;
//    MatrixXd _H;
    SparseMatrix<double> _H;
    Matrix3d A;
    Matrix3d B;

    //SparseMatrix<double> createSparse();
//    Vector3d linear_factors(Edge* e);
    Vector3d linear_factors(Edge* e, MatrixXd vmeans, MatrixXd emeans, int k);
    Pose retrieveVertex(int id);
    double normalizeTheta(double theta);
    MatrixXd linearize_and_solve(MatrixXd vmeans, int k);
    void updateMeans(MatrixXd means);
    Matrix3d v2t(Vector3d v);
    Vector3d t2v(Matrix3d A);

};

#endif // OPTIMIZER_H
