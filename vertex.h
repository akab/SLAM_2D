#ifndef VERTEX_H
#define VERTEX_H

#include "pose.h"
#include <stack>

using namespace std;

class Vertex
{
    int _id;
    Pose _pose;
    stack<Pose> _backup;
public:
    Vertex();
    ~Vertex();
    Vertex(int id, Pose p);
    void push(){ _backup.push(_pose); }
    void pop(){ _pose = _backup.top(); _backup.pop(); }
    void setPose(Pose p){ _pose = p; }
    int getId(){ return _id; }
    Pose getPose(){ return _pose; }
};

#endif // VERTEX_H
