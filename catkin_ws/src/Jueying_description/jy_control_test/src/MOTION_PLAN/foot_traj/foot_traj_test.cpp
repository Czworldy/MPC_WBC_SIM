#include "ros/ros.h"

#include "Gait.h"
#include "SupportPolygon.h"
#include "foothold.h"
#include "FootSwingTrajectory.h"
#include "UserParameter.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "jy_control_test_node");
    ros::NodeHandle n;

    //Class Gait Test____________________________________
    Vec41<int> offsets_walk, durations_walk;
    offsets_walk << 36,16,36,16;
    durations_walk << 24,24,24,24;
    Gait walk(40, offsets_walk, durations_walk, "walk");

    Vec41<float> contactState, swingState;
    int* gaitPattern;
    float time_StanceCur, time_SwingCur;
    float phase;
    int iterBetSEG(4), curIter(65);
    DMat<int> gaitPatternMat;

    walk.setIterations(iterBetSEG, curIter);
    phase = walk.getCurrentGaitPhase();
    time_StanceCur = walk.getCurrentStanceTime(0.2,1);
    time_SwingCur = walk.getCurrentSwingTime(0.2,1);
    contactState = walk.getContactState();
    swingState = walk.getSwingState();
    gaitPatternMat = walk.getGaitPatternMat();

    ROS_INFO_STREAM("PHASE:\n" << phase);
    ROS_INFO_STREAM("TIME_CURRENT_STANCE:\n" << time_StanceCur);
    ROS_INFO_STREAM("TIME_CURRENT_SWING:\n" << time_SwingCur);
    ROS_INFO_STREAM("CONTACT_STATE:\n" << contactState);
    ROS_INFO_STREAM("SWING_STATE:\n" << swingState);
    ROS_INFO_STREAM("GAITPATTERN:\n" << *gaitPattern);
    ROS_INFO_STREAM("GAITPATTERN_MAT:\n" << gaitPatternMat);
    
    
    // curIter = 16;
    // walk.setIterations(iterBetSEG, curIter);
    // ROS_INFO_STREAM("GAITPATTERN_16:\n" << walk.getGaitPatternMat());
    // curIter = 17;
    // walk.setIterations(iterBetSEG, curIter);
    // ROS_INFO_STREAM("GAITPATTERN_17:\n" << walk.getGaitPatternMat());
    // curIter = 18;
    // walk.setIterations(iterBetSEG, curIter);
    // ROS_INFO_STREAM("GAITPATTERN_18:\n" << walk.getGaitPatternMat());
    // curIter = 19;
    // walk.setIterations(iterBetSEG, curIter);
    // ROS_INFO_STREAM("GAITPATTERN_19:\n" << walk.getGaitPatternMat());
    // curIter = 20;
    // walk.setIterations(iterBetSEG, curIter);
    // ROS_INFO_STREAM("GAITPATTERN_20:\n" << walk.getGaitPatternMat());


    //Class FootHold______________________________________________
    // UserParameter<float> paramf;
    // FootHold footHold(&walk, 0.01, paramf);

    // Vec2<float> desVel;
    // desVel << 2,4;

    // Vec2<float> hipVel[4];
    // hipVel[0] << 2,3;
    // hipVel[1] << 2,3;
    // hipVel[2] << 2,3;
    // hipVel[3] << 2,3;

    // Vec31<float> hipPos[4];
    // hipPos[0] << 20,30,11;
    // hipPos[1] << 20,-30,11;
    // hipPos[2] << -20,30,11;
    // hipPos[3] <<-20,-30,11;

    // Vec2<float>* desHold;
    // desHold = footHold.FootHoldPlan(desVel, hipVel, hipPos);
    // ROS_INFO("_______________OKOKOKOKOKOKOKOKOKO_______");
    // ROS_INFO_STREAM("DES_FOOTHOLD:\n" << desHold[0]);
    // ROS_INFO_STREAM("DES_FOOTHOLD:\n" << desHold[1]);
    // ROS_INFO_STREAM("DES_FOOTHOLD:\n" << desHold[2]);
    // ROS_INFO_STREAM("DES_FOOTHOLD:\n" << desHold[3]);


    //Class SupportPolygon_______________________________________
    // int nSeg(20), iterPerSeg(2);
    // SupportPolygon supPoly(nSeg, iterPerSeg);

    // //PolygonCenter_____
    // vectorAligned<Vec2<float>>  polygon;
    // Vec2<float> point, center;
    // point << 0,0;
    // polygon.push_back(point);
    // point << 2,0;
    // polygon.push_back(point);
    // point <<4,2;
    // polygon.push_back(point);
    // point << 2,2;
    // polygon.push_back(point);
    
    // center = supPoly.getPolygonCenter(polygon);
    // ROS_INFO_STREAM("polygon center:\n"<< center);


    // DMat<int> segTable;
    // int cur(17);
    // walk.setIterations(iterPerSeg, cur);
    // segTable = walk.getGaitPatternMat();
    // ROS_INFO_STREAM("GAITPATTERN_18:\n" << walk.getGaitPatternMat());

    // FootStateData contactCur;
    // contactCur.footLocation[0]<<0,0;
    // contactCur.footLocation[1]<<1,1;
    // contactCur.footLocation[2]<<2,2;
    // contactCur.footLocation[3]<<3,3;
    // contactCur.isContact[0] = true;
    // contactCur.isContact[1] = true;
    // contactCur.isContact[2] = true;
    // contactCur.isContact[3] = false;

    // Vec2<float> contactDes[4];
    // contactDes[0]<< 4,4;
    // contactDes[1]<< 5,5;
    // contactDes[2]<< 6,6;
    // contactDes[3]<< 7,7;

    // supPoly.SearchPolygon(segTable, contactCur, contactDes, cur);
    // supPoly.SearchPolygon(segTable, contactCur, contactDes, cur);
    // std::vector<supportPolygonData>* data;
    // data = supPoly.getSupportPolySet();
    // for (int i(0); i<data->size(); i++){
    //     ROS_INFO_STREAM("MAIN_POLYGON:\n"<<(*data)[i].timeStart);
    // }
    // //ROS_INFO("____________________DONE__________________________");

    // Vec2<float> point;
    // vectorAligned<Vec2<float>> polygon1, polygon2;
    // point << 0.,2.;
    // polygon1.push_back(point);
    // point << 2.,0.;
    // polygon1.push_back(point);
    // point <<4.,2.;
    // polygon1.push_back(point);

    // vectorAligned<Vec31<double>>* lineCoeff;
    // lineCoeff = supPoly.getLineCoefficient(polygon1, paramf);
    // lineCoeff = supPoly.getLineCoefficient(polygon1, paramf);

    // for (int i(0);i<lineCoeff->size();i++)
    //     ROS_INFO_STREAM("LINECOEFF1: \n"<< (*lineCoeff)[i]);

    // point << 3.,2.;
    // polygon2.push_back(point);
    // point << 1.,0.;
    // polygon2.push_back(point);
    // point << 5.,0.;
    // polygon2.push_back(point);

    // lineCoeff = supPoly.getLineCoefficient(polygon2, paramf);

    // for (int i(0);i<lineCoeff->size();i++)
    //     ROS_INFO_STREAM("LINECOEFF2: \n"<< (*lineCoeff)[i]);

    // ROS_INFO_STREAM("INTERSECT_CHECK: "<< supPoly.IntersectCheck(polygon1,polygon2));

    //Class FootSwingTrajectory_____________________________________
    // FootSwingTrajectory<float> footSwingTrajectory;
    // Vec31<float> p0, pf, p, v, a;

    // float h(0.5);
    // p0 << 0,0,0;
    // pf << 1,1,0;

    // footSwingTrajectory.setInitialPosition(p0);
    // footSwingTrajectory.setFinalPosition(pf);
    // footSwingTrajectory.setHeight(h);
    // footSwingTrajectory.computeSwingTrajectoryBezier(0.5, 2);
    // p = footSwingTrajectory.getPosition();
    // v = footSwingTrajectory.getVelocity();
    // a = footSwingTrajectory.getAcceleration();
    // ROS_INFO_STREAM("FOOTSWINGTRAJ_p: "<<p);
    // ROS_INFO_STREAM("FOOTSWINGTRAJ_v: "<<v);
    // ROS_INFO_STREAM("FOOTSWINGTRAJ_a: "<<a);
    return 0;
}