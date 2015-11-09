#ifndef EDGE_H
#define EDGE_H

#include "pose.h"
#include "Eigen"

using namespace Eigen;

class Edge
{
    int _id1;
    int _id2;
    Pose _transf;
    Matrix3d _inform;
public:
    Edge(){ }
    Edge(int id1, int id2, Pose p, Matrix3d inf){
        _id1=id1,_id2 = id2, _transf=p, _inform=inf;
    }
    ~Edge(){}
    int id1(){ return _id1; }
    int id2(){ return _id2; }
    Pose getTransf(){ return _transf; }
    Matrix3d getInf(){ return _inform; }
};

#endif // EDGE_H
