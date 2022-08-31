#include "SearchOptCoefficients.h"
#include "ros/ros.h"
#include "time.h"

using namespace std;

SearchOptCoefficients::SearchOptCoefficients(int nSegment, int iterationsBetweenSEG, double tfk, SupportPolygon* polydata,UserParameter<double> & param):
    _iterationsBetweenSEG(iterationsBetweenSEG),
    _nSpline(nSegment),
    _tfk(tfk),
    _poly_data(polydata)
    {
        _Xmax = param.Xmax;
        _Xmin = param.Xmin;
        _Ymax = param.Ymax;
        _Ymin = param.Ymin;
        _margin = param.margin;

        n  = 12*_nSpline;

        c_int P_i_k[21] = {0, 
                           0,1,
                           0,1,2,
                           0,1,2,3,
                           0,1,2,3,4,
                           0,1,2,3,4,5,};
        c_int P_p_k[6] = {0,1,3,6,10,15,};

        P_i_ = new c_int[42*_nSpline];
        P_p_ = new c_int[n+1];
        P_nnz_ = 42*_nSpline;

        for(int i(0); i<_nSpline; i++){
            //P_i_
            for(int j(0);j<21;j++){
                P_i_[42*i+j] = P_i_k[j]+12*i;
                P_i_[42*i+j+21] = P_i_k[j]+12*i+6;
            }

            //P_p_
            for(int j(0); j<6; j++){
                P_p_[12*i+j] = P_p_k[j]+42*i;
                P_p_[12*i+j+6] = P_p_k[j]+42*i+21;
            }
        }
        //P_p_
        P_p_[n] = P_nnz_;

        // settings = new OSQPSettings;
        // data = new OSQPData;
}

SearchOptCoefficients::~SearchOptCoefficients(){
    delete [] P_i_;
    delete [] P_p_;
    // delete settings;
    // delete data;
}

void SearchOptCoefficients::run(c_float * P_x, c_float * q,
                                const Vec2<float>& initPoint, const Vec2<float>& initVel, const Vec2<float>& initAcc,
                                double zcom, const Vec2<float> & finalPoint,
                                DVec<double> & solution, bool & if_solved){
    zcom_ = zcom;
    //Resize Equality Constraint_____________________________________
    //Polygons Intersection Check
    std::vector<supportPolygonData> polySet; 
    std::vector<int> NumOfNoCross;
    int noCross(0);
    polySet = _poly_data->getSupportPolySet();
    // for(int i(0); i<polySet.size()-1;i++){
    //     if(!_poly_data->IntersectCheck(polySet[i+1].vertices, polySet[i].vertices)){//两多边形没有重叠时,
    //         noCross ++;
    //         NumOfNoCross.push_back(polySet[i+1].timeStart/_iterationsBetweenSEG);//表示此时的SPLINE前端发生了NoCross
    //     }
    // }

    _dimEq = 6 * _nSpline - 2*noCross;

    //Resize InEquation Constraints____________________________________
    _dimInEq = 2;//final point soft constraints
    int num_poly(polySet.size());
    for(int i(0); i<num_poly-1; i++){
        if(i==0){
            _dimInEq += (polySet[i+1].timeStart/_iterationsBetweenSEG) * _getNumOfLine(polySet[i].vertices.size());
        }
        else{
            _dimInEq += ((polySet[i+1].timeStart - polySet[i].timeStart)/_iterationsBetweenSEG) * _getNumOfLine(polySet[i].vertices.size());
        }
    }
    _dimInEq += ((_nSpline *_iterationsBetweenSEG - polySet[num_poly-1].timeStart)/_iterationsBetweenSEG)* _getNumOfLine(polySet[num_poly-1].vertices.size());
    _dimInEq -= _getNumOfLine(polySet[0].vertices.size());

    //Initialize constraints____________________________________________
    const c_float kInfinity = std::numeric_limits<c_float>::infinity();
    m = _dimEq + _dimInEq;
    A_nnz_ = 36 + 60*(_nSpline-2) + 30 - 16*noCross
                           + (_dimInEq - 2)*12 + 12;
    c_float A_x_[A_nnz_];
    c_int A_i_[A_nnz_];
    c_int A_p_[n+1];

    c_float l_[m];
    c_float u_[m];

    l_vec.resize(m);
    u_vec.resize(m);

    l_vec.head(_dimEq).setZero(); 
    u_vec.head(_dimEq).setZero();
    for(int i(_dimEq); i<(m-2);i++){
        u_vec[i] = kInfinity;
        //ROS_INFO_STREAM("INF: "<<u_vec[i]);
    }

    //Set constraints________________________________________________
    Index_Spline = 0;
    Index_Eq = 0;
    Index_InEq = 0;
    Index_A_x_ = 0;
    Index_A_i_ = 0;
    Index_A_p_ = 0;
    bool isNoCross(false);

   //ROS_INFO("___________________SetConstraints DONE!_________________");

    //Spline 0
    for(int j(0); j<noCross; j++){
        if(1==NumOfNoCross[j])
            isNoCross = true;
    }

    if(isNoCross){
        A_nnz_0 = 28;
        A_nnz_0_half = 14;
        A_x_0.resize(14);
        A_x_0 << pow(_tfk,5), 5*pow(_tfk,4), 
                            pow(_tfk,4), 4*pow(_tfk, 3),
                            pow(_tfk,3), 3*pow(_tfk,2),
                            2, pow(_tfk,2), 2*_tfk,
                            1, _tfk, 1,
                            1,1;
        A_i_0.resize(28);
        A_i_0 << 6,8,6,8,6,8,4,6,8,2,6,8,0,6,
                            7,9,7,9,7,9,5,7,9,3,7,9,1,7;
        A_p_0.resize(12);
        A_p_0 << 0,2,4,6,9,12,14,16,18,20,23,26;
    }
    else{
        A_nnz_0 = 36;
        A_nnz_0_half = 18;
        A_x_0.resize(18);
        A_x_0 << pow(_tfk,5), 5*pow(_tfk,4), 20*pow(_tfk,3), 
                            pow(_tfk,4), 4*pow(_tfk, 3), 12*pow(_tfk, 2),
                            pow(_tfk,3), 3*pow(_tfk,2), 6*_tfk,
                            2, pow(_tfk,2), 2*_tfk, 2,
                            1, _tfk, 1,
                            1,1;
        A_i_0.resize(36);
        A_i_0 << 6,8,10,6,8,10,6,8,10,4,6,8,10,2,6,8,0,6,
                            7,9,11,7,9,11,7,9,11,5,7,9,11,3,7,9,1,7;
        A_p_0.resize(12);
        A_p_0 << 0,3,6,9,13,16,18,21,24,27,31,34;
    }

    for(int i(0); i<2;i++){
        for(int j(0); j<A_nnz_0_half; j++)
            A_x_[A_nnz_0_half*i+j] = A_x_0[j];
    }
    for(int i(0); i<A_nnz_0; i++){
        A_i_[i] = A_i_0[i];
    }
    for(int i(0); i<12; i++){
        A_p_[i] = A_p_0[i]; 
    }
    for(int i(0); i<2; i++){
        l_vec[i] = u_vec[i] = initPoint[i];
        l_vec[i+2] = u_vec[i+2] = initVel[i];
        l_vec[i+4] = u_vec[i+4] = initAcc[i];
    }
    
    Index_Spline++;
    Index_Eq +=6;
    Index_InEq += _dimEq;
    Index_A_x_ += A_nnz_0;
    Index_A_i_ +=  A_nnz_0;
    Index_A_p_+=12;

    //Spline 1 TO Spline (n-2)
    for(int i(0); i<polySet.size()-1; i++){
        //ROS_INFO_STREAM("Set_Constraints_Spline :" << Index_Spline);
        num_line = _getNumOfLine(polySet[i].vertices.size());
        line.clear();
        line = _poly_data -> getLineCoefficient(polySet[i].vertices, _margin);

        int Spline(polySet[i+1].timeStart/_iterationsBetweenSEG);
        for(int j(Index_Spline); j<Spline; j++){
            bool isNoCrossLeft(false);
            bool isNoCrossRight(false);
            for(int k(0); k<noCross; k++){
                if(j==NumOfNoCross[k])
                    isNoCrossLeft = true;
                if((j+1)== NumOfNoCross[k])
                    isNoCrossRight = true;
            }

            if((isNoCrossLeft==false)&&(isNoCrossRight==false)){
                LeftCrossRightCross(j);
            }
            else if((isNoCrossLeft==false)&&(isNoCrossRight==true)){
                LeftCrossRightNoCross(j);
            }
            else if((isNoCrossLeft==true)&&(isNoCrossRight==false)){
                LeftNoCrossRightCross(j);
            }
            else{
                LeftNoCrossRightNoCross(j);
            }

            for(int k(0); k<A_nnz_k; k++){
                A_x_[Index_A_x_+k] = A_x_k[k];
                A_i_[Index_A_i_ + k] = A_i_k[k];
            }
            for(int k(0); k<12; k++){
                A_p_[Index_A_p_+k] = A_p_k[k] + Index_A_x_; 
            }

            if((isNoCrossLeft==false)&&(isNoCrossRight==false)){
                Index_Eq += 6;
                Index_A_x_ += 60+12*num_line;
                Index_A_i_ += 60+12*num_line;
                Index_A_p_+= 12;
            }
            else if((isNoCrossLeft==false)&&(isNoCrossRight==true)){
                Index_Eq += 6;
                Index_A_x_ += 52+12*num_line;
                Index_A_i_ += 52+12*num_line;
                Index_A_p_+= 12;
            }
            else if((isNoCrossLeft==true)&&(isNoCrossRight==false)){
                Index_Eq += 4;
                Index_A_x_ += 52+12*num_line;
                Index_A_i_ += 52+12*num_line;
                Index_A_p_+= 12;
            }
            else{
                Index_Eq += 4;
                Index_A_x_ += 44+12*num_line;
                Index_A_i_ += 44+12*num_line;
                Index_A_p_+= 12;
            }
            Index_Spline++;
            Index_InEq += num_line;
        }
    }

    //last polySet
    num_line = _getNumOfLine(polySet[num_poly-1].vertices.size());
    line.clear();
    line = _poly_data -> getLineCoefficient(polySet[num_poly-1].vertices, _margin);
    for(int j(Index_Spline); j<(_nSpline-1); j++){
        bool isNoCrossLeft(false);
        bool isNoCrossRight(false);
        for(int k(0); k<noCross; k++){
            if(j==NumOfNoCross[k])
                isNoCrossLeft = true;
            if((j+1)== NumOfNoCross[k])
                isNoCrossRight = true;
        }

        if((isNoCrossLeft==false)&&(isNoCrossRight==false)){
            LeftCrossRightCross(j);
        }
        else if((isNoCrossLeft==false)&&(isNoCrossRight==true)){
            LeftCrossRightNoCross(j);
        }
        else if((isNoCrossLeft==true)&&(isNoCrossRight==false)){
            LeftNoCrossRightCross(j);
        }
        else{
            LeftNoCrossRightNoCross(j);
        }

        for(int k(0); k<A_nnz_k; k++){
            A_x_[Index_A_x_+k] = A_x_k[k];
            A_i_[Index_A_i_ + k] = A_i_k[k];
        }
        for(int k(0); k<12; k++){
            A_p_[Index_A_p_+k] = A_p_k[k] + Index_A_x_; 
        }

        if((isNoCrossLeft==false)&&(isNoCrossRight==false)){
            Index_Eq += 6;
            Index_A_x_ += 60+12*num_line;
            Index_A_i_ += 60+12*num_line;
            Index_A_p_+= 12;
        }
        else if((isNoCrossLeft==false)&&(isNoCrossRight==true)){
            Index_Eq += 6;
            Index_A_x_ += 52+12*num_line;
            Index_A_i_ += 52+12*num_line;
            Index_A_p_+= 12;
        }
        else if((isNoCrossLeft==true)&&(isNoCrossRight==false)){
            Index_Eq += 4;
            Index_A_x_ += 52+12*num_line;
            Index_A_i_ += 52+12*num_line;
            Index_A_p_+= 12;
        }
        else{
            Index_Eq += 4;
            Index_A_x_ += 44+12*num_line;
            Index_A_i_ += 44+12*num_line;
            Index_A_p_+= 12;
        }
        Index_Spline++;
        Index_InEq += num_line;
    }

    //Spline (n-1)
    isNoCross = false;
    for(int j(0); j<noCross; j++){ 
        if((_nSpline-1)==NumOfNoCross[j])
            isNoCross = true;
    }

    num_line = _getNumOfLine(polySet[num_poly -1].vertices.size());
    line.clear();
    line = _poly_data -> getLineCoefficient(polySet[num_poly -1].vertices, _margin);

    if(isNoCross){
        LastSplineNoCross();
    }
    else{
        LastSplineCross();
    }

    for(int k(0); k<A_nnz_k; k++){
        A_x_[Index_A_x_+k] = A_x_k[k];
        A_i_[Index_A_i_ + k] = A_i_k[k];
    }
    for(int k(0); k<12; k++){
        A_p_[Index_A_p_+k] = A_p_k[k] + Index_A_x_; 
    }

    l_vec[m-2] = _Xmin + finalPoint[0];
    u_vec[m-2] =  _Xmax + finalPoint[0];
    l_vec[m-1] = _Ymin + finalPoint[1];
    u_vec[m-1] =  _Ymax + finalPoint[1];

    for(int i(0); i<m;i++){
        l_[i] = l_vec[i];
        u_[i] = u_vec[i];
    }

    // ROS_INFO_STREAM("LOWBOUND: \n"<<l_vec);
    // ROS_INFO_STREAM("UPBOUND: \n"<<u_vec);

    //A_p_(n)
    A_p_[n] = A_nnz_;
    
    // Exitflag
    c_int exitflag = 0;

    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

    //Populate data
    if(data){
        data->n = n;
        data->m = m;
        data->P = csc_matrix(data->n, data->n, P_nnz_, P_x, P_i_, P_p_);
        data->q = q;
        data->A = csc_matrix(data->m, data->n, A_nnz_, A_x_, A_i_, A_p_);
        data->l = l_;
        data->u = u_;
    }
    
    //Define solver settings as default

    if (settings) {
        osqp_set_default_settings(settings);
    }
    
    settings->max_iter = 400;

    // Setup workspace
    exitflag = osqp_setup(&work, data, settings);

    // Solve Problem
    osqp_solve(work);
    cout << "Solve QP INFO: " << endl;
    cout << work->info->status << endl;
    cout << work->info->iter << endl;
    if((work->info->iter > 3000)||(work->info->status[0] != 's')){
        if_solved = false;  
        cout << "SOLVE QP FALSE!!!!" << endl; 
    }
    else{
        if_solved = true;
        cout << "SOLVE QP TRUE!!!!" << endl; 
    }
    
    solution.resize(12*_nSpline);
    for(int i(0); i<n;i++){
        solution[i] = (work->solution->x)[i];
    }

    // Cleanup
    osqp_cleanup(work);
    if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
    }
    if (settings) c_free(settings);
}

void SearchOptCoefficients::LeftCrossRightCross(int SplineNum){
    double ta(_tfk*SplineNum);
    double tb(ta+_tfk);

    vector<vector<c_float> > xk(6);
    vector<vector<c_int> > ik(6);
    //0
    xk[0].push_back(-pow(ta,5));
    xk[0].push_back(-5*pow(ta,4));
    xk[0].push_back(-20*pow(ta, 3));
    xk[0].push_back(pow(tb,5));
    xk[0].push_back(5*pow(tb,4));
    xk[0].push_back(20*pow(tb, 3));

    ik[0].push_back(0);
    ik[0].push_back(2);
    ik[0].push_back(4);
    ik[0].push_back(6);
    ik[0].push_back(8);
    ik[0].push_back(10);

    //1
    xk[1].push_back(-pow(ta,4));
    xk[1].push_back(-4*pow(ta,3));
    xk[1].push_back(-12*pow(ta,2));
    xk[1].push_back(pow(tb,4));
    xk[1].push_back(4*pow(tb,3));
    xk[1].push_back(12*pow(tb,2));

    ik[1].push_back(0);
    ik[1].push_back(2);
    ik[1].push_back(4);
    ik[1].push_back(6);
    ik[1].push_back(8);
    ik[1].push_back(10);

    //2
    xk[2].push_back(-pow(ta,3));
    xk[2].push_back(-3*pow(ta,2));
    xk[2].push_back(-6*ta);
    xk[2].push_back(pow(tb,3));
    xk[2].push_back(3*pow(tb,2));
    xk[2].push_back(6*tb);

    ik[2].push_back(0);
    ik[2].push_back(2);
    ik[2].push_back(4);
    ik[2].push_back(6);
    ik[2].push_back(8);
    ik[2].push_back(10);

    //3
    xk[3].push_back(-pow(ta,2));
    xk[3].push_back(-2*ta);
    xk[3].push_back(-2);
    xk[3].push_back(pow(tb,2));
    xk[3].push_back(2*tb);
    xk[3].push_back(2);

    ik[3].push_back(0);
    ik[3].push_back(2);
    ik[3].push_back(4);
    ik[3].push_back(6);
    ik[3].push_back(8);
    ik[3].push_back(10);

    //4
    xk[4].push_back(-ta);
    xk[4].push_back(-1);
    xk[4].push_back(tb);
    xk[4].push_back(1);

    ik[4].push_back(0);
    ik[4].push_back(2);
    ik[4].push_back(6);
    ik[4].push_back(8);

    //5
    xk[5].push_back(-1);
    xk[5].push_back(1);

    ik[5].push_back(0);
    ik[5].push_back(6);

    //InEqConstraints Part
    Mat16<c_float> w_ = _Eta(ta) - zcom_/9.81*_Eta_dd(ta);

    int Index_xk(0);
    A_x_k.resize(60+num_line*12);
    A_i_k.resize(60+num_line*12);
    //x
    for(int i(0); i<6; i++){
        for(int j(0); j< xk[i].size(); j++){
            A_x_k[Index_xk + j] = xk[i][j];
            A_i_k[Index_xk + j] = ik[i][j] +Index_Eq;
        }
        Index_xk += xk[i].size();
        for(int k(0); k<num_line; k++){
            A_x_k[Index_xk + k] = line[k][0]*w_[i];
            A_i_k[Index_xk + k] = k + Index_InEq;
            l_vec[Index_InEq+k] =- line[k][2];
        }
        Index_xk += num_line;
    }
    //y
    for(int i(0); i<6; i++){
        for(int j(0); j<xk[i].size();j++){
            A_x_k[Index_xk+j] = xk[i][j];
            A_i_k[Index_xk + j] = ik[i][j] + 1 + Index_Eq;
        }
        Index_xk += xk[i].size();
        for(int k(0); k<num_line; k++){
            A_x_k[Index_xk + k] = line[k][1]*w_[i];
            A_i_k[Index_xk + k] = k + Index_InEq;
        }
        Index_xk += num_line;
    }

    //A_nnz_k
    A_nnz_k = 60 + num_line*12;

    //A_p_k
    A_p_k[0] = 0;
    A_p_k[1] = 6+num_line;
    A_p_k[2] = A_p_k[1] + 6 + num_line;
    A_p_k[3] = A_p_k[2] + 6 + num_line;
    A_p_k[4] = A_p_k[3] + 6 + num_line;
    A_p_k[5] = A_p_k[4] + 4 + num_line; 

    A_p_k[6] = A_p_k[5]+ 2 + num_line;
    A_p_k[7] = A_p_k[6]+ 6 + num_line;
    A_p_k[8] = A_p_k[7]+ 6 + num_line;
    A_p_k[9] = A_p_k[8]+ 6 + num_line;
    A_p_k[10] = A_p_k[9]+ 6 + num_line;
    A_p_k[11] = A_p_k[10]+ 4 + num_line;
}

void SearchOptCoefficients::LeftCrossRightNoCross(int SplineNum){
    double ta(_tfk*SplineNum);
    double tb(ta+_tfk);

    vector<vector<c_float>> xk(6);
    vector<vector<c_int>> ik(6);

    //0
    xk[0].push_back(-pow(ta,5));
    xk[0].push_back(-5*pow(ta,4));
    xk[0].push_back(-20*pow(ta, 3));
    xk[0].push_back(pow(tb,5));
    xk[0].push_back(5*pow(tb,4));

    ik[0].push_back(0);
    ik[0].push_back(2);
    ik[0].push_back(4);
    ik[0].push_back(6);
    ik[0].push_back(8);

    //1
    xk[1].push_back(-pow(ta,4));
    xk[1].push_back(-4*pow(ta,3));
    xk[1].push_back(-12*pow(ta,2));
    xk[1].push_back(pow(tb,4));
    xk[1].push_back(4*pow(tb,3));

    ik[1].push_back(0);
    ik[1].push_back(2);
    ik[1].push_back(4);
    ik[1].push_back(6);
    ik[1].push_back(8);

    //2
    xk[2].push_back(-pow(ta,3));
    xk[2].push_back(-3*pow(ta,2));
    xk[2].push_back(-6*ta);
    xk[2].push_back(pow(tb,3));
    xk[2].push_back(3*pow(tb,2));

    ik[2].push_back(0);
    ik[2].push_back(2);
    ik[2].push_back(4);
    ik[2].push_back(6);
    ik[2].push_back(8);

    //3
    xk[3].push_back(-pow(ta,2));
    xk[3].push_back(-2*ta);
    xk[3].push_back(-2);
    xk[3].push_back(pow(tb,2));
    xk[3].push_back(2*tb);

    ik[3].push_back(0);
    ik[3].push_back(2);
    ik[3].push_back(4);
    ik[3].push_back(6);
    ik[3].push_back(8);

    //4
    xk[4].push_back(-ta);
    xk[4].push_back(-1);
    xk[4].push_back(tb);
    xk[4].push_back(1);

    ik[4].push_back(0);
    ik[4].push_back(2);
    ik[4].push_back(6);
    ik[4].push_back(8);

    //5
    xk[5].push_back(-1);
    xk[5].push_back(1);

    ik[5].push_back(0);
    ik[5].push_back(6);

    //InEqConstraints Part
    Mat16<c_float> w_ = _Eta(ta) - zcom_/9.81*_Eta_dd(ta);

    int Index_xk(0);
    A_x_k.resize(52+num_line*12);
    A_i_k.resize(52+num_line*12);
    //x
    for(int i(0); i<6; i++){
        for(int j(0); j< xk[i].size(); j++){
            A_x_k[Index_xk + j] = xk[i][j];
            A_i_k[Index_xk + j] = ik[i][j] +Index_Eq;
        }
        Index_xk += xk[i].size();
        for(int k(0); k<num_line; k++){
            A_x_k[Index_xk + k] = line[k][0]*w_[i];
            A_i_k[Index_xk + k] = k + Index_InEq;
            l_vec[Index_InEq+k] =- line[k][2];
        }
        Index_xk += num_line;
    }
    //y
    for(int i(0); i<6; i++){
        for(int j(0); j<xk[i].size();j++){
            A_x_k[Index_xk+j] = xk[i][j];
            A_i_k[Index_xk + j] = ik[i][j] + 1 + Index_Eq;
        }
        Index_xk += xk[i].size();
        for(int k(0); k<num_line; k++){
            A_x_k[Index_xk + k] = line[k][1]*w_[i];
            A_i_k[Index_xk + k] = k + Index_InEq;
        }
        Index_xk += num_line;
    }

    //A_nnz_k
    A_nnz_k = 52 + num_line*12;

    //A_p_k
    A_p_k[0] = 0;
    A_p_k[1] = 5+num_line;
    A_p_k[2] = A_p_k[1] + 5 + num_line;
    A_p_k[3] = A_p_k[2] + 5 + num_line;
    A_p_k[4] = A_p_k[3] + 5 + num_line;
    A_p_k[5] = A_p_k[4] + 4 + num_line; 

    A_p_k[6] = A_p_k[5]+ 2 + num_line;
    A_p_k[7] = A_p_k[6]+ 5 + num_line;
    A_p_k[8] = A_p_k[7]+ 5 + num_line;
    A_p_k[9] = A_p_k[8]+ 5 + num_line;
    A_p_k[10] = A_p_k[9]+ 5 + num_line;
    A_p_k[11] = A_p_k[10]+ 4 + num_line;
}

void SearchOptCoefficients::LeftNoCrossRightCross(int SplineNum){
    double ta(_tfk*SplineNum);
    double tb(ta+_tfk);

    vector<vector<c_float>> xk(6);
    vector<vector<c_int>> ik(6);

    //0
    xk[0].push_back(-pow(ta,5));
    xk[0].push_back(-5*pow(ta,4));
    xk[0].push_back(pow(tb,5));
    xk[0].push_back(5*pow(tb,4));
    xk[0].push_back(20*pow(tb, 3));

    ik[0].push_back(0);
    ik[0].push_back(2);
    ik[0].push_back(4);
    ik[0].push_back(6);
    ik[0].push_back(8);

    //1
    xk[1].push_back(-pow(ta,4));
    xk[1].push_back(-4*pow(ta,3));
    xk[1].push_back(pow(tb,4));
    xk[1].push_back(4*pow(tb,3));
    xk[1].push_back(12*pow(tb,2));

    ik[1].push_back(0);
    ik[1].push_back(2);
    ik[1].push_back(4);
    ik[1].push_back(6);
    ik[1].push_back(8);

    //2
    xk[2].push_back(-pow(ta,3));
    xk[2].push_back(-3*pow(ta,2));
    xk[2].push_back(pow(tb,3));
    xk[2].push_back(3*pow(tb,2));
    xk[2].push_back(6*tb);

    ik[2].push_back(0);
    ik[2].push_back(2);
    ik[2].push_back(4);
    ik[2].push_back(6);
    ik[2].push_back(8);

    //3
    xk[3].push_back(-pow(ta,2));
    xk[3].push_back(-2*ta);
    xk[3].push_back(pow(tb,2));
    xk[3].push_back(2*tb);
    xk[3].push_back(2);

    ik[3].push_back(0);
    ik[3].push_back(2);
    ik[3].push_back(4);
    ik[3].push_back(6);
    ik[3].push_back(8);

    //4
    xk[4].push_back(-ta);
    xk[4].push_back(-1);
    xk[4].push_back(tb);
    xk[4].push_back(1);

    ik[4].push_back(0);
    ik[4].push_back(2);
    ik[4].push_back(4);
    ik[4].push_back(6);

    //5
    xk[5].push_back(-1);
    xk[5].push_back(1);

    ik[5].push_back(0);
    ik[5].push_back(4);   

    //InEqConstraints Part
    Mat16<c_float> w_ = _Eta(ta) - zcom_/9.81*_Eta_dd(ta);

    int Index_xk(0);
    A_x_k.resize(52+num_line*12);
    A_i_k.resize(52+num_line*12);
    //x
    for(int i(0); i<6; i++){
        for(int j(0); j< xk[i].size(); j++){
            A_x_k[Index_xk + j] = xk[i][j];
            A_i_k[Index_xk + j] = ik[i][j] +Index_Eq;
        }
        Index_xk += xk[i].size();
        for(int k(0); k<num_line; k++){
            A_x_k[Index_xk + k] = line[k][0]*w_[i];
            A_i_k[Index_xk + k] = k + Index_InEq;
            l_vec[Index_InEq+k] =- line[k][2];
        }
        Index_xk += num_line;
    }
    //y
    for(int i(0); i<6; i++){
        for(int j(0); j<xk[i].size();j++){
            A_x_k[Index_xk+j] = xk[i][j];
            A_i_k[Index_xk + j] = ik[i][j] + 1 + Index_Eq;
        }
        Index_xk += xk[i].size();
        for(int k(0); k<num_line; k++){
            A_x_k[Index_xk + k] = line[k][1]*w_[i];
            A_i_k[Index_xk + k] = k + Index_InEq;
        }
        Index_xk += num_line;
    }
    //A_nnz_k
    A_nnz_k = 52 + num_line*12;

    //A_p_k
    A_p_k[0] = 0;
    A_p_k[1] = 5+num_line;
    A_p_k[2] = A_p_k[1] + 5 + num_line;
    A_p_k[3] = A_p_k[2] + 5 + num_line;
    A_p_k[4] = A_p_k[3] + 5 + num_line;
    A_p_k[5] = A_p_k[4] + 4 + num_line; 

    A_p_k[6] = A_p_k[5]+ 2 + num_line;
    A_p_k[7] = A_p_k[6]+ 5 + num_line;
    A_p_k[8] = A_p_k[7]+ 5 + num_line;
    A_p_k[9] = A_p_k[8]+ 5 + num_line;
    A_p_k[10] = A_p_k[9]+ 5 + num_line;
    A_p_k[11] = A_p_k[10]+ 4 + num_line;
}

void SearchOptCoefficients::LeftNoCrossRightNoCross(int SplineNum){
    double ta(_tfk*SplineNum);
    double tb(ta+_tfk);

    vector<vector<c_float>> xk(6);
    vector<vector<c_int>> ik(6);

    //0
    xk[0].push_back(-pow(ta,5));
    xk[0].push_back(-5*pow(ta,4));
    xk[0].push_back(pow(tb,5));
    xk[0].push_back(5*pow(tb,4));

    ik[0].push_back(0);
    ik[0].push_back(2);
    ik[0].push_back(4);
    ik[0].push_back(6);

    //1
    xk[1].push_back(-pow(ta,4));
    xk[1].push_back(-4*pow(ta,3));
    xk[1].push_back(pow(tb,4));
    xk[1].push_back(4*pow(tb,3));

    ik[1].push_back(0);
    ik[1].push_back(2);
    ik[1].push_back(4);
    ik[1].push_back(6);

    //2
    xk[2].push_back(-pow(ta,3));
    xk[2].push_back(-3*pow(ta,2));
    xk[2].push_back(pow(tb,3));
    xk[2].push_back(3*pow(tb,2));

    ik[2].push_back(0);
    ik[2].push_back(2);
    ik[2].push_back(4);
    ik[2].push_back(6);

    //3
    xk[3].push_back(-pow(ta,2));
    xk[3].push_back(-2*ta);
    xk[3].push_back(pow(tb,2));
    xk[3].push_back(2*tb);

    ik[3].push_back(0);
    ik[3].push_back(2);
    ik[3].push_back(4);
    ik[3].push_back(6);

    //4
    xk[4].push_back(-ta);
    xk[4].push_back(-1);
    xk[4].push_back(tb);
    xk[4].push_back(1);

    ik[4].push_back(0);
    ik[4].push_back(2);
    ik[4].push_back(4);
    ik[4].push_back(6);

    //5
    xk[5].push_back(-1);
    xk[5].push_back(1);

    ik[5].push_back(0);
    ik[5].push_back(4);

    //InEqConstraints Part
    Mat16<c_float> w_ = _Eta(ta) - zcom_/9.81*_Eta_dd(ta);

    int Index_xk(0);
    A_x_k.resize(44 +num_line*12);
    A_i_k.resize(44+num_line*12);
    //x
    for(int i(0); i<6; i++){
        for(int j(0); j< xk[i].size(); j++){
            A_x_k[Index_xk + j] = xk[i][j];
            A_i_k[Index_xk + j] = ik[i][j] +Index_Eq;
        }
        Index_xk += xk[i].size();
        for(int k(0); k<num_line; k++){
            A_x_k[Index_xk + k] = line[k][0]*w_[i];
            A_i_k[Index_xk + k] = k + Index_InEq;
            l_vec[Index_InEq+k] =- line[k][2];
        }
        Index_xk += num_line;
    }
    //y
    for(int i(0); i<6; i++){
        for(int j(0); j<xk[i].size();j++){
            A_x_k[Index_xk+j] = xk[i][j];
            A_i_k[Index_xk + j] = ik[i][j] + 1 + Index_Eq;
        }
        Index_xk += xk[i].size();
        for(int k(0); k<num_line; k++){
            A_x_k[Index_xk + k] = line[k][1]*w_[i];
            A_i_k[Index_xk + k] = k + Index_InEq;
        }
        Index_xk += num_line;
    }

    //A_nnz_k
    A_nnz_k = 44 + num_line*12;

    //A_p_k
    A_p_k[0] = 0;
    A_p_k[1] = 4+num_line;
    A_p_k[2] = A_p_k[1] + 4 + num_line;
    A_p_k[3] = A_p_k[2] + 4 + num_line;
    A_p_k[4] = A_p_k[3] + 4 + num_line;
    A_p_k[5] = A_p_k[4] + 4 + num_line; 

    A_p_k[6] = A_p_k[5]+ 2 + num_line;
    A_p_k[7] = A_p_k[6]+ 4 + num_line;
    A_p_k[8] = A_p_k[7]+ 4 + num_line;
    A_p_k[9] = A_p_k[8]+ 4 + num_line;
    A_p_k[10] = A_p_k[9]+ 4 + num_line;
    A_p_k[11] = A_p_k[10]+ 4 + num_line;
}

void SearchOptCoefficients::LastSplineCross(){
    double ta(_tfk*(_nSpline - 1));
    double tb(ta+_tfk);

    vector<vector<c_float>> xk(6);
    vector<vector<c_int>> ik(6);

    //0
    xk[0].push_back(-pow(ta,5));
    xk[0].push_back(-5*pow(ta,4));
    xk[0].push_back(-20*pow(ta, 3));

    ik[0].push_back(0);
    ik[0].push_back(2);
    ik[0].push_back(4);

    //1
    xk[1].push_back(-pow(ta,4));
    xk[1].push_back(-4*pow(ta,3));
    xk[1].push_back(-12*pow(ta,2));

    ik[1].push_back(0);
    ik[1].push_back(2);
    ik[1].push_back(4);

    //2
    xk[2].push_back(-pow(ta,3));
    xk[2].push_back(-3*pow(ta,2));
    xk[2].push_back(-6*ta);

    ik[2].push_back(0);
    ik[2].push_back(2);
    ik[2].push_back(4);

    //3
    xk[3].push_back(-pow(ta,2));
    xk[3].push_back(-2*ta);
    xk[3].push_back(-2);

    ik[3].push_back(0);
    ik[3].push_back(2);
    ik[3].push_back(4);

    //4
    xk[4].push_back(-ta);
    xk[4].push_back(-1);

    ik[4].push_back(0);
    ik[4].push_back(2);

    //5
    xk[5].push_back(-1);

    ik[5].push_back(0);

    //InEqConstraints Part
    Mat16<c_float> w_ = _Eta(ta) - zcom_/9.81*_Eta_dd(ta);
    Mat16<c_float> eta_ = _Eta(tb);

    int Index_xk(0);
    A_x_k.resize(30 + num_line*12 + 12);
    A_i_k.resize(30 + num_line*12 + 12);
    //x
    for(int i(0); i<6; i++){
        for(int j(0); j< xk[i].size(); j++){
            A_x_k[Index_xk + j] = xk[i][j];
            A_i_k[Index_xk + j] = ik[i][j] +Index_Eq;
        }
        Index_xk += xk[i].size();
        for(int k(0); k<num_line; k++){
            A_x_k[Index_xk + k] = line[k][0]*w_[i];
            A_i_k[Index_xk + k] = k + Index_InEq;
            l_vec[Index_InEq+k] = -line[k][2];
        }
        A_x_k[Index_xk + num_line] = eta_[i];
        A_i_k[Index_xk + num_line] = num_line;

        Index_xk += num_line;
        Index_xk++;
    }
    //y
    for(int i(0); i<6; i++){
        for(int j(0); j<xk[i].size();j++){
            A_x_k[Index_xk+j] = xk[i][j];
            A_i_k[Index_xk + j] = ik[i][j] + 1 + Index_Eq;
        }
        Index_xk += xk[i].size();
        for(int k(0); k<num_line; k++){
            A_x_k[Index_xk + k] = line[k][1]*w_[i];
            A_i_k[Index_xk + k] = k + Index_InEq;
        }
        A_x_k[Index_xk + num_line] = eta_[i];
        A_i_k[Index_xk + num_line] = num_line+1;
        Index_xk += num_line;
        Index_xk ++;
    }

    //A_nnz_k
    A_nnz_k = 30 + num_line*12 + 12;

    //A_p_k
    A_p_k[0] = 0;
    A_p_k[1] = 3+num_line +1;
    A_p_k[2] = A_p_k[1] + 3 + num_line+1;
    A_p_k[3] = A_p_k[2] + 3 + num_line+1;
    A_p_k[4] = A_p_k[3] + 3 + num_line+1;
    A_p_k[5] = A_p_k[4] + 2 + num_line+1; 

    A_p_k[6] = A_p_k[5]+ 1 + num_line+1;
    A_p_k[7] = A_p_k[6]+ 3 + num_line+1;
    A_p_k[8] = A_p_k[7]+ 3 + num_line+1;
    A_p_k[9] = A_p_k[8]+ 3 + num_line+1;
    A_p_k[10] = A_p_k[9]+ 3 + num_line+1;
    A_p_k[11] = A_p_k[10]+ 2 + num_line+1;

}

void SearchOptCoefficients::LastSplineNoCross(){
    double ta(_tfk*(_nSpline - 1));
    double tb(ta+_tfk);

    vector<vector<c_float>> xk(6);
    vector<vector<c_int>> ik(6);

    //0
    xk[0].push_back(-pow(ta,5));
    xk[0].push_back(-5*pow(ta,4));

    ik[0].push_back(0);
    ik[0].push_back(2);

    //1
    xk[1].push_back(-pow(ta,4));
    xk[1].push_back(-4*pow(ta,3));

    ik[1].push_back(0);
    ik[1].push_back(2);

    //2
    xk[2].push_back(-pow(ta,3));
    xk[2].push_back(-3*pow(ta,2));

    ik[2].push_back(0);
    ik[2].push_back(2);

    //3
    xk[3].push_back(-pow(ta,2));
    xk[3].push_back(-2*ta);

    ik[3].push_back(0);
    ik[3].push_back(2);

    //4
    xk[4].push_back(-ta);
    xk[4].push_back(-1);

    ik[4].push_back(0);
    ik[4].push_back(2);

    //5
    xk[5].push_back(-1);

    ik[5].push_back(0);

    //InEqConstraints Part
    Mat16<c_float> w_ = _Eta(ta) - zcom_/9.81*_Eta_dd(ta);
    Mat16<c_float> eta_ = _Eta(tb);

    int Index_xk(0);
    A_x_k.resize(22 + num_line*12 + 12);
    A_i_k.resize(22 + num_line*12 + 12);
    //x
    for(int i(0); i<6; i++){
        for(int j(0); j< xk[i].size(); j++){
            A_x_k[Index_xk + j] = xk[i][j];
            A_i_k[Index_xk + j] = ik[i][j] +Index_Eq;
        }
        Index_xk += xk[i].size();
        for(int k(0); k<num_line; k++){
            A_x_k[Index_xk + k] = line[k][0]*w_[i];
            A_i_k[Index_xk + k] = k + Index_InEq;
            l_vec[Index_InEq+k] = -line[k][2];
        }
        A_x_k[Index_xk + num_line] = eta_[i];
        A_i_k[Index_xk + num_line] = num_line;

        Index_xk += num_line;
        Index_xk++;
    }
    //y
    for(int i(0); i<6; i++){
        for(int j(0); j<xk[i].size();j++){
            A_x_k[Index_xk+j] = xk[i][j];
            A_i_k[Index_xk + j] = ik[i][j] + 1 + Index_Eq;
        }
        Index_xk += xk[i].size();
        for(int k(0); k<num_line; k++){
            A_x_k[Index_xk + k] = line[k][1]*w_[i];
            A_i_k[Index_xk + k] = k + Index_InEq;
        }
        A_x_k[Index_xk + num_line] = eta_[i];
        A_i_k[Index_xk + num_line] = num_line+1;
        Index_xk += num_line;
        Index_xk ++;
    }

    //A_nnz_k
    A_nnz_k = 22 + num_line*12 + 12;

    //A_p_k
    A_p_k[0] = 0;
    A_p_k[1] = 2+num_line +1;
    A_p_k[2] = A_p_k[1] + 2 + num_line+1;
    A_p_k[3] = A_p_k[2] + 2 + num_line+1;
    A_p_k[4] = A_p_k[3] + 2 + num_line+1;
    A_p_k[5] = A_p_k[4] + 2 + num_line+1; 

    A_p_k[6] = A_p_k[5]+ 1 + num_line+1;
    A_p_k[7] = A_p_k[6]+ 2 + num_line+1;
    A_p_k[8] = A_p_k[7]+ 2 + num_line+1;
    A_p_k[9] = A_p_k[8]+ 2 + num_line+1;
    A_p_k[10] = A_p_k[9]+ 2 + num_line+1;
    A_p_k[11] = A_p_k[10]+ 2 + num_line+1;

}

Mat2_12<double>   SearchOptCoefficients::_timePosMat(double time){
    Mat16<double> eta;
    eta << pow(time,5), pow(time,4), pow(time,3), pow(time,2), time, 1;

    Mat2_12<double> timeMat;
    timeMat.setZero();
    timeMat.topLeftCorner(1,6) = eta;
    timeMat.bottomRightCorner(1,6) = eta;

    return timeMat;
}

Mat2_12<double>   SearchOptCoefficients::_timeVelMat(double time){
    Mat16<double> eta_d;
    eta_d << 5*pow(time,4), 4*pow(time,3), 3*pow(time,2), 2*time, 1, 0;
   
    Mat2_12<double> timeMat;
    timeMat.setZero();
    timeMat.topLeftCorner(1,6) = eta_d;
    timeMat.bottomRightCorner(1,6) = eta_d;

    return timeMat;
}

Mat2_12<double>   SearchOptCoefficients::_timeAccMat(double time){
    Mat16<double> eta_dd;
    eta_dd << 20*pow(time,3), 12*pow(time,2), 6*time, 2,0,0;
   
    Mat2_12<double> timeMat;
    timeMat.setZero();
    timeMat.topLeftCorner(1,6) = eta_dd;
    timeMat.bottomRightCorner(1,6) = eta_dd;

    return timeMat;
}


Mat16<double>  SearchOptCoefficients::_Eta(double time){
    Mat16<double> eta;
    eta << pow(time,5), pow(time,4), pow(time,3), pow(time,2), time, 1;

    return eta;
} 

Mat16<double>  SearchOptCoefficients::_Eta_dd(double time){
    Mat16<double> eta_dd;
    eta_dd << 20*pow(time,3), 12*pow(time,2), 6*time, 2,0,0;

    return eta_dd;
}

int SearchOptCoefficients::_getNumOfLine(int numVertice){
    int num;
    if(numVertice != 2)
        num = numVertice;
    else
        num = 4;
    return num;
}