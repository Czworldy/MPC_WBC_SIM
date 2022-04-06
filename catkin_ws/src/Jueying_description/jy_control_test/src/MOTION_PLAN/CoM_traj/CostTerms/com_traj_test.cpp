#include "ros/ros.h"

#include "Gait.h"
#include "SupportPolygon.h"
#include "AccelerationMin.h"
#include "UserParameter.h"
#include "SoftFinalConstraints.h"
#include "DeviationFromPre.h"
//#include "PathRegularization.h"
#include "SearchOptCoefficients.h"

#include "osqp.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "jy_control_test_node");
    ros::NodeHandle n;

    //Class AccelerationMin______________________________
    int nSegment(20);
    int iterationsBetweenSEG(2);
    int nSpline(nSegment*iterationsBetweenSEG);
    double tf(1);
    AccelerationMin accelerationMin(nSegment, iterationsBetweenSEG, tf);

    c_float P_Acc[42*nSegment*iterationsBetweenSEG];
    accelerationMin.UpdateCostFunctionAcc(P_Acc);
    ROS_INFO("________________ACCELERATIONMIN:______________________");
    // for(int i(0); i<42*nSpline; i++)
    //     ROS_INFO_STREAM("P_rows:   " << P_Acc[i]);

    //Class SoftFinalConstraints___________________________
    UserParameter<double> paramd;
    SoftFinalConstraints softFinalConstraints(nSegment, iterationsBetweenSEG, tf,  paramd);

    Vec2<float> pf;
    pf << 1,2;
    c_float P_Final[42];
    c_float q_Final[12];
    softFinalConstraints.UpdateCostFunction(pf, P_Final, q_Final);
    ROS_INFO("________________SOFTFINALCONSTRAINTS:______________________");
    for(int j(0); j<42; j++){
        ROS_INFO_STREAM("FINAL: "<<P_Final[j]);
    }
    for(int j(0); j<12; j++){
        ROS_INFO_STREAM("F: "<<q_Final[j]);
    }

    // //Class DeviationFromPre_____________________________
    // DeviationFromPre deviationFromPre(nSegment, iterationsBetweenSEG, tf, paramd);
    // DVec<double> preSolution(24);
    // preSolution.setOnes();
    // c_float P_Devia[42*nSpline];
    // c_float q_Devia[42*nSpline];
    
    // deviationFromPre.UpdateCostFunction(preSolution, P_Devia, q_Devia);
    // ROS_INFO("DEVIATIONFROMPRE___________________________________");

    // ROS_INFO_STREAM("Q2:\n"<<Q2);
    // ROS_INFO_STREAM("Q2_d:\n"<<Q2_d);
    // ROS_INFO_STREAM("Q2_dd:\n"<<Q2_dd);
    // ROS_INFO_STREAM("c2:\n"<<c2);
    // ROS_INFO_STREAM("c2_d:\n"<<c2_d);
    // ROS_INFO_STREAM("c2_dd:\n"<<c2_dd);

    //Class PathRegularization
    // PathRegularization path(nSegment, iterationsBetweenSEG, tf, paramd);
    // Vec2<float> initPoint_, initVel_, initAcc_;
    // Vec2<float> finalPoint_, finalVel_, finalAcc_;
    // initPoint_ << 2,2;
    // initVel_ << 1,0;
    // initAcc_ << 0,0;
    // finalPoint_ << 3,2;
    // finalVel_ <<1,0;
    // finalAcc_ <<0,0;

    // DMat<double> Qf, Qfdot, Qfddot;
    // DVec<double> cf, cfdot, cfddot;

    // Mat6_12<double> timeMat;
    // timeMat = path._timeMat(1);
    // ROS_INFO_STREAM("timeMat:\n  "<< timeMat);
    // path._UpdateAccMin();
    // path._Update_EqConstraint(initPoint_, initVel_, initAcc_, 
    //                                                             finalPoint_, finalVel_, finalAcc_);
    // ROS_INFO("PATH_EQCONSTRAINT: ");
    // ROS_INFO_STREAM(path.A_.bottomRightCorner(12,36));

    // path._SolveQP();

    // path.UpdateCostFunction(initPoint_, initVel_, initAcc_, 
    //                                                         finalPoint_, finalVel_, finalAcc_,
    //                                                         Qf, cf, Qfdot,cfdot, Qfddot, cfddot);
    // ROS_INFO("________________Solve QP DONE!______________________");



    //Class SearchOptCoefficients_______________________________
    Vec41<int> offsets_walk, durations_walk;
    offsets_walk << 4,19,14,9;
    durations_walk << 16,16,16,16;
    int cur(20);
    DMat<int> segTable;

    Gait walk(nSegment, offsets_walk, durations_walk, "walk");
    walk.setIterations(iterationsBetweenSEG, cur);
    segTable = walk.getGaitPatternMat();
    ROS_INFO_STREAM("GAITPATTERN_:\n" << walk.getGaitPatternMat());

    FootStateData contactCur;
    contactCur.footLocation[0]<<5,3;
    contactCur.footLocation[1]<<1,3;
    contactCur.footLocation[2]<<1,0;
    contactCur.footLocation[3]<<5,0;
    contactCur.isContact[0] = true;
    contactCur.isContact[1] = true;
    contactCur.isContact[2] = false;
    contactCur.isContact[3] = true;

    Vec2<float> contactDes[4];
    contactDes[0]<< 6,3;
    contactDes[1]<< 2,3;
    contactDes[2]<< 2,0;
    contactDes[3]<< 6,0;

    SupportPolygon supPoly(nSegment, iterationsBetweenSEG);   
    std::vector<supportPolygonData> data;
    supPoly.SearchPolygon(segTable, contactCur, contactDes, cur);
    data = supPoly.getSupportPolySet();
    ROS_INFO("________________SearchPolygon DONE!______________________");

    SearchOptCoefficients searchOpt(nSegment, iterationsBetweenSEG, tf, &supPoly, paramd);
    ROS_INFO("________________searchOpt DONE!______________________");
    // ROS_INFO_STREAM("_timePosMat:\n" << searchOpt._timePosMat(2.));
    // ROS_INFO_STREAM("_timeVelMat:\n" << searchOpt._timeVelMat(2.));
    // ROS_INFO_STREAM("_timeAccMat:\n" << searchOpt._timeAccMat(2.));
    // ROS_INFO_STREAM("_Eta:\n" << searchOpt._Eta(2.));
    // ROS_INFO_STREAM("_Eta_dd:\n" << searchOpt._Eta_dd(2.));
    // ROS_INFO_STREAM("_getNumOfLine:\n" << searchOpt._getNumOfLine(2));

    c_float q_[12*nSpline];
    for(int i(0); i<12*nSpline; i++){
        q_[i] = 0; 
    }
    // searchOpt._Update_CostFunction(Q, c);
    // ROS_INFO_STREAM("cost function: Q_rows:  "<<searchOpt.Q_.rows());
    // ROS_INFO_STREAM("cost function: Q_cols:  "<<searchOpt.Q_.cols());
    // ROS_INFO_STREAM("cost function: C_rows:  "<<searchOpt.c_.rows());
    // ROS_INFO_STREAM("cost function: C_cols:  "<<searchOpt.c_.cols());
    // ROS_INFO("________________SET COST FUNCTION DONE!______________________");


    Vec2<float> initPoint;
    Vec2<float> initVel;
    Vec2<float> initAcc;
    Vec2<float> finalPoint;

    initPoint << 2,2;
    finalPoint << 3,2;
    initVel << 1,0;
    initAcc << 0,0;

    // searchOpt._Update_EqConstraint(initPoint, initVel, initAcc);
    // // ROS_INFO_STREAM("Equality Constraints: rows: " << searchOpt._dimEq);
    // // ROS_INFO_STREAM("Equality Constraints: \n: " << searchOpt.A_.bottomRightCorner(30, 24));
    // // ROS_INFO_STREAM("Equality Constraints: \n: " << searchOpt.b_.tail(30));

    double zcom(0.5);
    // searchOpt._Update_InEqConstraint(zcom);
    // // ROS_INFO_STREAM("InEquality Constraints: rows: " << searchOpt._dimInEq);
    // // ROS_INFO_STREAM("InEquality Constraints: \n: " << searchOpt.D_.bottomRightCorner(11, 24));
    // // ROS_INFO_STREAM("InEquality Constraints: \n: " << searchOpt.f_.tail(11));

    DVec<double> solution;
    solution.resize(12*nSpline);
    // searchOpt._SolveQP(solution);
    // ROS_INFO("________________Solve QP DONE!______________________");

    searchOpt.run(P_Acc, q_, initPoint, initVel, initAcc, zcom,finalPoint , solution);
    ROS_INFO("________________Solve QP DONE!______________________");

    return 0;
}