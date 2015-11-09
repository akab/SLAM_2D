#include "pose.h"

inline double normalize_angle(double yaw)
{
  if (yaw >= -M_PI && yaw < M_PI) return yaw;

  double multiplier = floor(yaw / (2*M_PI));
  yaw = yaw - multiplier*2*M_PI;
  if (yaw >= M_PI) yaw -= 2*M_PI;
  if (yaw < -M_PI) yaw += 2*M_PI;

  return yaw;
}

//Pose Pose::boxplus(Pose state, Vector3d incr){
//    Matrix3d s, res;
//    s = constructTransform(state);
//    Matrix3d A = v2t(incr);
//    res = s*A;
//}

//Matrix3d Pose::constructTransform(Pose state){
//    Matrix3d s;
//    Matrix2d r = state.rotation().toRotationMatrix();
//    Vector2d v = state.translation();
//    s(1,1) = r(1,1);
//    s(1,2) = r(1,2);
//    s(2,1) = r(2,1);
//    s(2,2) = r(2,2);
//    s(1,3) = v(1);
//    s(2,3) = v(2);
//    s(3,1) = 0;
//    s(3,2) = 0;
//    s(3,3) = 1;

//    return s;
//}

//Matrix3d Pose::measurementFunction(Pose i, Pose j){
//    Matrix3d Xi = constructTransform(i);
//    Matrix3d Xj = constructTransform(j);

//    return Xi.inverse()*Xj;
//}

//Vector3d Pose::errorFunction(Pose state1, Pose state2, Pose meas){
//    Matrix3d M = measurementFunction(state1,state2);
//    Matrix3d Z = constructTransform(meas);

//    Matrix3d res = Z.inverse()*M;
//    return t2v(res);
//}

Pose Pose::operator* (const Pose& p){
    Pose result(*this);
    result._t += _R*p._t;
    result._R.angle() += p._R.angle();
    result._R.angle() = normalize_angle(result._R.angle());
    return result;
}

Pose& Pose::operator*= (const Pose& tr2){
  _t += _R*tr2._t;
  _R.angle()+=tr2._R.angle();
  _R.angle()=normalize_angle(_R.angle());
  return *this;
}



Pose Pose::inverse(){
  Pose ret;
  ret.setRotation(_R.inverse());
  ret.setAngle(normalize_angle(ret._R.angle()));
//#ifdef _MSC_VER
//  ret._t=ret._R*(Vector2d(_t*-1.));
//#else
  ret.setTranslation(ret._R*(_t*-1.));
//#endif
  return ret;
}

//Vector3d Pose::t2v(Matrix3d A){
//    Vector3d v;
//    for(int i = 1; i < 3; ++i)
//        v(i) = A(i,3);
//    v(3) = atan2(A(2,1),A(1,1));

//    return v;
//}

//Matrix3d Pose::v2t(Vector3d v){
//    Matrix3d A;
//    double c = cos(v(3));
//    double s = sin(v(3));

//    A << c, -s,  v(1),
//         s,  c,  v(2),
//         0,  0,  1;

//    return A;
//}
