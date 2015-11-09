#include "vertex.h"

Vertex::Vertex()
{

}

Vertex::~Vertex(){

}

Vertex::Vertex(int id, Pose p){
    _id = id;
    _pose = p;
}
