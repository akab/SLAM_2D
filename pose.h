#ifndef POSE_H
#define POSE_H

#include "Eigen"
#include <cmath>
#include <ctgmath>
#include <iostream>

#define M_PI 3.14159265358979323846

using namespace Eigen;

class Pose
{
private:
    Rotation2Dd _R;
    Vector2d _t;
public:
    Pose():_R(0),_t(0,0){}
    Pose(Rotation2Dd R, Vector2d t) : _R(R), _t(t){}
    ~Pose(){}
    Pose(double x, double y, double yaw):_R(yaw),_t(x,y){}
    Vector2d& translation(){ return _t; }
    Rotation2D<double>& rotation(){ return _R; }
    Pose operator *(const Pose& p);
    Pose& operator *=(const Pose& tr2);
    Pose inverse();
    void print(){ std::cout << _t(0) << " " << _t(1) << " " << _R.angle() << std::endl;}
    Vector3d toVector(){ return Vector3d(_t.x(), _t.y(), _R.angle()); }
    void fromVector (Vector3d& v){ *this=Pose(v[0], v[1], v[2]); }
    void setRotation(Rotation2Dd R){ _R = R; }
    void setAngle(double theta){ _R.angle() = theta; }
    void setTranslation(Vector2d t){ _t = t; }
//    Pose boxplus(Pose state, Vector3d incr);
//    Matrix3d constructTransform(Pose state);
//    Matrix3d measurementFunction(Pose i, Pose j);
//    Vector3d errorFunction(Pose state1, Pose state2, Pose meas);
//    Matrix3d v2t(Vector3d v);
//    Vector3d t2v(Matrix3d A);
};

#endif // POSE_H
