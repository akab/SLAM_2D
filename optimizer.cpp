#include "optimizer.h"
#include <algorithm>    // std::sort

IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

Optimizer::Optimizer()
{
}

Optimizer::Optimizer(FactorGraph *g){
    _graph = *g;
//    _H = MatrixXd(_graph.vetices().size()*3,_graph.vetices().size()*3);
    _H = SparseMatrix<double>(_graph.vetices().size()*3,_graph.vetices().size()*3);
    _b = VectorXd(_graph.vetices().size()*3);
//    _globalError = 0;
}

void printVmeans(MatrixXd vmeans, int i){

    stringstream sstm;
    sstm << i << "_vmeans.xls";
    string name = sstm.str();
    string m = "mv " + name + " ~/SLAM_Project";
    const char* move = m.c_str();

    ofstream h;
    h.open(name);
    h << vmeans.format(OctaveFmt);
    h.close();

    system(move);
}

void Optimizer::run(MatrixXd vmeans, int n){

    cout << " ************** OPTIMIZATION ************** " << endl;
    MatrixXd newmeans(3,_graph.vetices().size());
    for(int i = 0; i < n; i++){
        printVmeans(vmeans,i);
        cout << "ITERATION N# " << i << endl;
        vmeans = linearize_and_solve(vmeans,i);
//        newmeans = linearize_and_solve(vmeans);
//        vmeans = newmeans;
    }
    newmeans = vmeans;
    cout << " *********** OPTIMIZATION DONE! *********** " << endl;

    writeOptGraph(newmeans);
}

void printHessian(MatrixXd H, int i){

    stringstream sstm;
    sstm << i << "_H.xls";
    string name = sstm.str();
    string app = "mv " + name + " ~/SLAM_Project";
    const char* move = app.c_str();


    ofstream h;
    h.open(name);
    h << H.format(OctaveFmt);
    h.close();

    system(move);
}

void printB(MatrixXd b, int i){

    stringstream sstm;
    sstm << i << "_b.xls";
    string name = sstm.str();
    string app = "mv " + name + " ~/SLAM_Project";
    const char* move = app.c_str();


    ofstream h;
    h.open(name);
    h << b.format(OctaveFmt);
    h.close();

    system(move);
}

void sparsePrint(SparseMatrix<double> mat){
    for (int k=0; k<mat.outerSize(); ++k)
      for (SparseMatrix<double>::InnerIterator it(mat,k); it; ++it)
      {
        cout << "value " << it.value() << endl;
        cout << "row index " << it.row() << " ";   // row index
        cout << "col index " << it.col() << endl;   // col index (here it is equal to k)
      }
}

SparseMatrix<double> Optimizer::createSparse(){

//    typedef Triplet<double> T;
//    vector<T> tripletList;
//    tripletList.reserve(_H.cols());
//    for(int i = 0; i < _H.rows(); i++)
//    {
//        for(int j = 0; j < _H.cols(); j++){
//            double vij = _H(i,j);
//            tripletList.push_back(T(i,j,vij));
//        }
//    }
//    SparseMatrix<double> H(_H.rows(),_H.cols());
//    H.setFromTriplets(tripletList.begin(), tripletList.end());
//    H.makeCompressed();

//    return H;
}

void Optimizer::updateMeans(MatrixXd means){

    for(int i = 0; i < means.cols(); i++){
        Pose p(means.col(i)[0],means.col(i)[1],means.col(i)[2]);
        _graph.vetices().at(i)->setPose(p);
    }

}

MatrixXd constructEMeans(set<Edge*> edges){
    MatrixXd emeans(3,edges.size());
    int i = 0;
    for(set<Edge*>::iterator it = edges.begin(); it != edges.end(); ++it, ++i){
        Edge* e = *it;
        Pose p = e->getTransf();
        emeans.col(i) = p.toVector();
    }

    return emeans;
}

/**
 * @brief Optimizer::linearize_and_solve
 * @param vmeans: vertices positions at the linearization point (X = matrix of vertices <x y theta>)
 * @param eids: edge ids
 * @param emeans: edge means
 * @param einfs: edge information matrices
 * @return newmeans: new solution computed from the initial guess in vmeans
 */
MatrixXd Optimizer::linearize_and_solve(MatrixXd vmeans,int k){
    cout << "- Initialize the system matrix and the system vector (H and b)" << endl;
    _H = SparseMatrix<double>(vmeans.cols()*3, vmeans.cols()*3);
    _b = VectorXd::Zero(vmeans.cols()*3);


    cout << "- Global system loop, accumulates in H and b all edges contributions..." << endl;
    cout << "...Accumulate the block in H and b..." << endl;
//    pause();
    Matrix3d H_ii, H_jj, H_ij, H_ji;
    Vector3d b_i, b_j;
    Matrix3d id = Matrix3d::Identity();

    //Resolve the Gauge Ambiguity
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++)
            tripletList.push_back(T(i,j,id(i,j)));
    }

    //Define quantities
    int id_i, id_j;
    double s,c;
    set<Edge*> edges = _graph.edges();
    Matrix3d omega;
    MatrixXd emeans = constructEMeans(edges);
    Vector3d error;
    Edge* e;
    int i = 0;

    for(set<Edge*>::iterator it = edges.cbegin(); it != edges.cend();it++){
        e = *it;
        id_i = e->id1();
        id_j = e->id2();
        omega = e->getInf();
//        error = linear_factors(e);
//        error = linear_factors(e,vmeans);
        error = linear_factors(e,vmeans,emeans,i);

//        cout << "Compute the blocks of H^k" << endl;
//        Vector3d b_i = -A.transpose()*omega*error;
//        Vector3d b_j = -B.transpose()*omega*error;
//        Matrix3d H_ii = A.transpose()*omega*A;
//        Matrix3d H_ij = A.transpose()*omega*B;
//        Matrix3d H_jj = B.transpose()*omega*B;
          b_i += -A.transpose()*omega*error;
          b_j += -B.transpose()*omega*error;
          H_ii += A.transpose()*omega*A;
          H_jj += B.transpose()*omega*B;
          H_ij += A.transpose()*omega*B;
          H_ji += H_ij.transpose();

//        cout << "Accumulate the block in H and b" << endl;
        cout << "edge n# " << i << ": " << id_i << " -> " << id_j << endl;
//        system("clear");
        ++id_i;
        ++id_j;
//        _H.block((id_i-1)*3,(id_i-1)*3,3,3) += H_ii;
//        _H.block((id_j-1)*3,(id_j-1)*3,3,3) += H_jj;
//        _H.block((id_i-1)*3,(id_j-1)*3,3,3) += H_ij;
//        _H.block((id_j-1)*3,(id_i-1)*3,3,3) += H_ij.transpose();
        for(int i = (id_i-1)*3; i < 3; i++){
            for(int j = (id_i-1)*3; j < 3; j++)
               tripletList.push_back(T(i,j,H_ii(i,j)));
        }

        for(int i = (id_j-1)*3; i < 3; i++){
            for(int j = (id_j-1)*3; j < 3; j++)
                tripletList.push_back(T(i,j,H_jj(i,j)));
        }

        for(int i = (id_i-1)*3; i < 3; i++){
            for(int j = (id_j-1)*3; j < 3; j++)
                tripletList.push_back(T(i,j,H_ij(i,j)));
        }

        for(int i = (id_j-1)*3; i < 3; i++){
            for(int j = (id_i-1)*3; j < 3; j++)
                tripletList.push_back(T(i,j,H_ji(i,j)));
        }


//        cout << "H's computed! " << endl;
//        for(int i = (id_i-1)*3; i < (id_i-1)*3+3; i++) _b(i) += b_i(i);
        _b.segment((id_i-1)*3,3) += b_i;
//        for(int j = (id_j-1)*3; j < (id_j-1)*3+3; j++) _b(j) += b_j(j);
        _b.segment((id_j-1)*3,3) += b_j;
//        cout << "b's computed! " << endl;


        i++;

    }
    cout << "...Done!" << endl;

//    _H.block(0,0,3,3) += Matrix<double, 3, 3>::Identity();
//    cout << "- Gauge ambiguity resolved! " << endl;

    _H.setFromTriplets(tripletList.begin(), tripletList.end());
    _H.makeCompressed();

    printHessian(_H,k);
    printB(_b,k);

    VectorXd dX;
//    SparseMatrix<double> H = createSparse();
    cout << "- Sparse matrix H created! " << endl;
    cout << "- Solving the Linear System..." << endl;
    SparseQR<SparseMatrix<double>, COLAMDOrdering<int>> solver;
    solver.compute(_H);
    if(solver.info() != Success){
        cout << " decomposition failed!!! " << endl;
        exit(0);
    }
    dX = solver.solve(_b);
    if(solver.info() != Success){
        cout << " solving failed!!! " << endl;
        exit(0);
    }
    cout << "...Done!" << endl;

    cout << "split the increments in 3x1 vectors and sum" << endl;
    MatrixXd dXp(3,vmeans.cols());
    dXp = Map<MatrixXd>(dX.data(),3,vmeans.cols());
    MatrixXd newmeans(3,_graph.vetices().size());
    newmeans = vmeans + dXp;

    cout << "normalize the angle between -pi and pi" << endl;
    for(int i = 0; i < newmeans.cols(); i++){
        s = sin(newmeans.col(i)[2]);
        c = cos(newmeans.col(i)[2]);
        newmeans.col(i)[2] = atan2(s,c);
    }

//    updateMeans(newmeans);

    return newmeans;
}

Vector3d Optimizer::linear_factors(Edge* e, MatrixXd vmeans, MatrixXd emeans, int k){
    int idi = e->id1();
    int idj = e->id2();
    Vector3d vi(vmeans.col(idi)[0],vmeans.col(idi)[1],vmeans.col(idi)[2]);
    Vector3d vj(vmeans.col(idj)[0],vmeans.col(idj)[1],vmeans.col(idj)[2]);
    Vector3d zij = emeans.col(k);

    //homogeneous transf of the previous solutions
    Matrix3d vt_i = v2t(vi);
    Matrix3d vt_j = v2t(vj);
    Matrix3d zt_ij = v2t(zij);

    //displacement betw xi and xj
    Matrix3d f_ij = vt_i.inverse()*vt_j;
    double theta_i = vi[2];
    Vector2d ti,tj;
    ti(0) = vi(0);
    ti(1) = vi(1);
    tj(0) = vj(0);
    tj(1) = vj(1);
    Vector2d dt_ij = tj - ti;

    double si = sin(theta_i);
    double ci = cos(theta_i);
    Vector2d R1(-si,ci);
    Vector2d R2(-ci,-si);
    double Rdt1 = R1.transpose()*dt_ij;
    double Rdt2 = R2.transpose()*dt_ij;

    A << -ci, -si, Rdt1,
          si, -ci, Rdt2,
           0,   0,   -1;
    B <<  ci, si, 0,
         -si, ci, 0,
           0, 0, 1;
    Matrix3d ztinv = zt_ij.inverse();
    Vector3d err = t2v(ztinv*f_ij);
    ztinv(0,2) = 0;
    ztinv(1,2) = 0;
    A = ztinv*A;
    B = ztinv*B;

    return err;
}

void Optimizer::writeOptGraph(MatrixXd newmeans){

    string name = "solution.g2o";

    if (ifstream(name))
    {
         system("rm solution.g2o");
    }

    ofstream map;
    map.open(name);
    for(int i = 0; i < newmeans.cols(); ++i){
            Vector2d t(newmeans(0,i),newmeans(1,i));
            Rotation2Dd r(newmeans(2,i));
            map << "VERTEX_SE2 " << i << " " << t(0) << " " << t(1) << " " << r.angle() << endl;
    }

    for(std::set<Edge*>::iterator it = _graph.edges().begin(); it != _graph.edges().end(); it++){
        Edge* e = *it;
        Vector2d t = e->getTransf().translation();
        Rotation2Dd R = e->getTransf().rotation();
        Matrix3d inf = e->getInf();
        map << "EDGE_SE2 " << e->id1() << " " << e->id2() << " " << t(0) << " " << t(1) << " " << R.angle() << " " <<
                inf(0,0) << " " << inf(0,1) << " " << inf(0,2) << " " << inf(1,1) << " " << inf(1,2) << " " << inf(2,2) <<  endl;
    }

    map.close();

    system("mv solution.g2o ~/SLAM_Project/");
}

Vector3d Optimizer::t2v(Matrix3d A){
    Vector3d v;
    for(int i = 0; i < 2; ++i)
        v(i) = A(i,2);
    v(2) = atan2(A(1,0),A(0,0));

    return v;
}

Matrix3d Optimizer::v2t(Vector3d v){
    Matrix3d A;
    double c = cos(v(2));
    double s = sin(v(2));

    A << c, -s,  v(0),
         s,  c,  v(1),
         0,  0,  1;

    return A;
}

//Vector3d Optimizer::linear_factors(Edge* e){
//    Pose state1 = retrieveVertex(e->id1());
//    Pose state2 = retrieveVertex(e->id2());
//    Pose meas = e->getTransf();

//    //homogeneous transf of the previous solutions
//    Matrix3d vt_i = v2t(Vector3d(state1.translation()(0),
//                                 state1.translation()(1),
//                                 state1.rotation().angle()));
//    Matrix3d vt_j = v2t(Vector3d(state2.translation()(0),
//                                 state2.translation()(1),
//                                 state2.rotation().angle()));
//    Matrix3d zt_ij = v2t(Vector3d(meas.translation()(0),
//                                  meas.translation()(1),
//                                  meas.rotation().angle()));

//    //displacement betw xi and xj
//    Matrix3d f_ij = vt_i.inverse()*vt_j;
//    double theta_i = state1.rotation().angle();
//    Vector2d ti,tj;
//    ti(0) = state1.translation()(0);
//    ti(1) = state1.translation()(1);
//    tj(0) = state2.translation()(0);
//    tj(1) = state2.translation()(1);
//    Vector2d dt_ij = tj - ti;

//    double si = sin(theta_i);
//    double ci = cos(theta_i);
//    Vector2d R1(-si,ci);
//    Vector2d R2(-ci,-si);
//    double Rdt1 = R1.transpose()*dt_ij;
//    double Rdt2 = R2.transpose()*dt_ij;

//    A << -ci, -si, Rdt1,
//          si, -ci, Rdt2,
//           0,   0,   -1;
//    B <<  ci, si, 0,
//         -si, ci, 0,
//           0, 0, 1;
//    Matrix3d ztinv = zt_ij.inverse();
//    Vector3d err = t2v(ztinv*f_ij);
//    ztinv(0,2) = 0;
//    ztinv(1,2) = 0;
//    A = ztinv*A;
//    B = ztinv*B;

//    return err;
//}

double Optimizer::normalizeTheta(double theta){
    if (theta >= -M_PI && theta < M_PI)
      return theta;

    double multiplier = floor(theta / (2*M_PI));
    theta = theta - multiplier*2*M_PI;
    if (theta >= M_PI)
      theta -= 2*M_PI;
    if (theta < -M_PI)
      theta += 2*M_PI;

    return theta;

}

Pose Optimizer::retrieveVertex(int id){
    if(!(_graph.exsist(id))) assert(0 && "retrieveVertex: Vertex not present!");
    return _graph.at(id)->getPose();
}
