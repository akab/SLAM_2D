#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "loadg2o.h"
#include "optimizer.h"
#include "factorgraph.h"

using namespace std;

MatrixXd v2m(FactorGraph graph);

int main()
{
    char m;
    cout << "maps: " << endl;
    cout << "1 - killian.g2o " << endl;
    cout << "2 - intel.g2o " << endl;
    cout << "3 - manhattanOlson3500.g2o" << endl;
    cout << "choose: ";
    cin >> m;

    int nmap = m - '0';

    loadG2O g;
    switch(nmap){
    case 1:
        g = loadG2O("/home/valerio/SLAM_Project/killian.g2o");
        break;
    case 2:
        g = loadG2O("/home/valerio/SLAM_Project/intel.g2o");
        break;
    case 3:
        g = loadG2O("/home/valerio/SLAM_Project/manhattanOlson3500.g2o");
        break;
    default:
        cout << "wrong number!" << endl;
        exit(0);
    }

    cin.clear();
    FactorGraph graph = g.getGraph();


//    cout << " ************** MAIN ************** " << endl;
    Optimizer opt(&graph);
    cout << "Graph acquired! ";
    cout << "n# of vertices " << graph.vetices().size() << " n# of edges " << graph.edges().size() << endl;
    char n;
    cout << "insert the number of iteration for the algorithm: ";
    cin >> n;
    MatrixXd vmeans = v2m(graph);
    int it = n - '0';
    opt.run(vmeans,it);


}

MatrixXd v2m(FactorGraph graph){
    vector<Vertex*> vertices = graph.vetices();
    MatrixXd vmeans(3,vertices.size());
    for(int i = 0; i < vertices.size(); ++i){
//    for(int i = 0; i < 10; ++i){
        Vertex* v = vertices.at(i);
        Vector3d vv = v->getPose().toVector();
        for(int j = 0; j < 3; ++j){
            vmeans.col(i)[j] = vv[j];
        }
    }

    return vmeans;
}


