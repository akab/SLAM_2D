#ifndef LOADG2O_H
#define LOADG2O_H

#include "factorgraph.h"
#include "vertex.h"
#include "pose.h"
#include "Eigen/Core"

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <stdexcept>

using namespace std;
using namespace Eigen;

#define LINESIZE 81920

class loadG2O
{
private:
    FactorGraph _graph;
public:
    loadG2O();
    loadG2O(const string& filename);
    ~loadG2O();
    FactorGraph createGraph(const string& filename);
    FactorGraph getGraph(){ return _graph; }
};

#endif // LOADG2O_H
