#include "factorgraph.h"

FactorGraph::FactorGraph()
{

}

FactorGraph::~FactorGraph(){

}

bool FactorGraph::insertVertex(Vertex v){
    if(exsist(v.getId())) return false;
    _vertices.push_back(new Vertex(v.getId(),v.getPose()));
    return true;
}

bool FactorGraph::exsist(int id){
    if(_vertices.size() == 0) return false;
    for(std::vector<Vertex*>::iterator it = _vertices.begin(); it != _vertices.end(); it++){
        Vertex* v = *it;
        if(v->getId() == id) return true;
    }
    return false;
}

bool FactorGraph::insertEdge(Edge e){
    if(&e == NULL) return false;
//    std::cout << e.id1() << " " << e.id2() << std::endl;
    _edges.insert(new Edge(e.id1(),e.id2(),e.getTransf(),e.getInf()));
    return true;
}

Vertex* FactorGraph::at(int id){
    if(exsist(id)){
        for(std::vector<Vertex*>::iterator it = _vertices.begin(); it != _vertices.end(); it++){
            Vertex* v = *it;
            if(v->getId() == id) return v;
        }
    }
    assert(0 && "at: vertex not present!");
}
