#ifndef FACTORGRAPH_H
#define FACTORGRAPH_H

#include <map>
#include <unordered_map>
#include <set>
#include <cassert>
#include <utility>
#include <iostream>

#include "vertex.h"
#include "edge.h"

class FactorGraph
{
private:

//    std::unordered_map<int, Vertex*>  _verticesMap;
    std::vector<Vertex*> _vertices;
    std::set<Edge*>  _edges;
//    std::vector<Edge*> _edges;


public:
    FactorGraph();
    ~FactorGraph();
    bool insertVertex(Vertex v);
    bool insertEdge(Edge e);
    bool exsist(int id);
    Vertex* at(int id);
    std::set<Edge*> edges(){ return _edges; }
    std::vector<Vertex*> vetices(){ return _vertices;}
    int nVertex(){ return _vertices.size(); }
    int nEdges(){ return _edges.size(); }
};

#endif // FACTORGRAPH_H
