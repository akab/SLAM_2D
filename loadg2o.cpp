#include "loadg2o.h"

loadG2O::loadG2O()
{

}

loadG2O::loadG2O(const string &filename)
{
    _graph = createGraph(filename);
}

loadG2O::~loadG2O(){

}

FactorGraph loadG2O::createGraph(const string &filename){
    ifstream is(filename.c_str());
    if (!is) throw invalid_argument("load2D: can not find file " + filename);

    FactorGraph graph = FactorGraph();

    string tag;

    //load the poses
    while(!is.eof()){
        if(!(is >> tag)) break;

        if((tag == "VERTEX2") || (tag == "VERTEX_SE2") || (tag == "VERTEX")){
            int id;
            double x, y, theta;
            is >> id >> x >> y >> theta;
//            cout << "id " << id << " x " << x << " y " << y << " theta " << theta << endl;

            Pose p(x,y,theta);
            Vertex v(id,p);
            if(!(graph.insertVertex(v))) assert(0 && "error in insertVertex: vertex present");
        }
        is.ignore(LINESIZE, '\n');
    }
    is.clear();
    is.seekg(0, ios::beg);

    //parse the pose constraints
    int id1, id2, id3;
    while (!is.eof()){
      if (!(is >> tag)) break;
      if ((tag == "EDGE2") || (tag == "EDGE") || (tag == "EDGE_SE2") || (tag == "ODOMETRY")){
          //Read transformation
          double x, y, theta;
          is >> id1 >> id2;
          is >> x >> y >> theta;
          Pose p(x, y, theta);

          if(!graph.exsist(id1)){
              Vertex v(id1,Pose());
              graph.insertVertex(v);
          }
          if(!graph.exsist(id2)){
              Vertex v(id2,graph.at(id1)->getPose()*p);
              graph.insertVertex(v);
          }

          double v1, v2, v3, v4, v5, v6;
          is >> v1 >> v2 >> v3 >> v4 >> v5 >> v6;
          Matrix3d inf;
//          inf(0,0) = v1;
//          inf(0,1) = v2;
//          inf(0,2) = v3;
//          inf(1,0) = - inf(0,1);
//          inf(1,1) = v4;
//          inf(1,2) = v5;
//          inf(2,0) = -inf(0,2);
//          inf(2,1) = -inf(1,2);
//          inf(2,2) = v6;
          inf(0,0) = v1;
          inf(0,1) = 0;
          inf(0,2) = 0;
          inf(1,0) = 0;
          inf(1,1) = v3;
          inf(1,2) = 0;
          inf(2,0) = 0;
          inf(2,1) = 0;
          inf(2,2) = v4;

          Edge e(id1,id2,p,inf);
//          cout << id1 << " " << id2 << endl;
          graph.insertEdge(e);
      }
    }
//    for(std::vector<Vertex*>::iterator it = graph.vetices().begin(); it != graph.vetices().end();it++){
//        Vertex *v = *it;
//        cout << v->getId() << endl;
//    }
//    for(std::set<Edge*>::iterator it = graph.edges().begin(); it != graph.edges().end();it++){
//        Edge *e = *it;
//        cout << e->id1() << " " << e->id2() << endl;
//    }

    return graph;

}
