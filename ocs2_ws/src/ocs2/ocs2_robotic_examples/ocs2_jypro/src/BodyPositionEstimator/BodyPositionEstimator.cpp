// #include "BodyPositionEstimator.h"
// #include "ros/ros.h"

// namespace ocs2 {
// Vec31<double> BodyPositionEst::bodyPositionEst(const Vec31<double> & qleg_lf,
//                                               const Vec31<double> & qleg_lb,
//                                               const Vec31<double> & qleg_rf,
//                                               const Vec31<double> & qleg_rb,
//                                               const Vec41<double> & contactState){
//     //Foot position in Base Frame
//     _foot_lf = _sysParam.getFootLocationLF(qleg_lf);
//     _foot_lb = _sysParam.getFootLocationLB(qleg_lb);
//     _foot_rf = _sysParam.getFootLocationRF(qleg_rf);
//     _foot_rb = _sysParam.getFootLocationRB(qleg_rb);

//     // ROS_INFO_STREAM("_FOOT_LF: \n"<<_foot_lf);
//     // ROS_INFO_STREAM("_FOOT_LB: \n"<<_foot_lb);
//     // ROS_INFO_STREAM("_FOOT_RF: \n"<<_foot_rf);
//     // ROS_INFO_STREAM("_FOOT_RB: \n"<<_foot_rb);

//     vectorAligned<Vec2<double>> foot_print;
//     foot_print.push_back(_foot_lf.head(2));
//     foot_print.push_back(_foot_lb.head(2));
//     foot_print.push_back(_foot_rb.head(2));
//     foot_print.push_back(_foot_rf.head(2));

//     _foot_print_center = this->getPolygonCenter(foot_print);

//     int num_contact=0;
//     _body_height = 0;

//     if(contactState[legID::LF]){
//         num_contact++;
//         _body_height += _foot_lf[2];
//         // ROS_INFO("LF IS CONTACT!");
//     }
//     if(contactState[legID::LB]){
//         num_contact++;
//         _body_height += _foot_lb[2];
//         // ROS_INFO("LB IS CONTACT!");
//     }
//     if(contactState[legID::RF]){
//         num_contact++;
//         _body_height += _foot_rf[2];
//         // ROS_INFO("RF IS CONTACT!");
//     }
//     if(contactState[legID::RB]){
//         num_contact++;
//         _body_height += _foot_rb[2];
//         // ROS_INFO("RB IS CONTACT!");
//     }
//     // ROS_INFO_STREAM("body_height: "<<_body_height);
//     _body_height /= (num_contact+0.01);
//     // ROS_INFO_STREAM("NUM_CONTACT: "<<num_contact);

//     if(num_contact == 0){
//         _body_height = _body_height_pre;
//     }
//     _body_height_pre = _body_height;


//     _pos_body.head(2) = -_foot_print_center;
//     _pos_body[2] = -_body_height;
//     // ROS_INFO_STREAM("rb_foot_in_foot_print_center: "<<(_foot_rb.head(2) - _foot_print_center ));
//     // ROS_INFO_STREAM("rb_foot_height: "<<(_foot_rb[2] + _pos_body[2]));

//     _foot_lf_in_center.head(2) = _foot_lf.head(2) - _foot_print_center;
//     _foot_lf_in_center[2] = _foot_lf[2] + _pos_body[2];

//     _foot_lb_in_center.head(2) = _foot_lb.head(2) - _foot_print_center;
//     _foot_lb_in_center[2] = _foot_lb[2] + _pos_body[2];

//     _foot_rf_in_center.head(2) = _foot_rf.head(2) - _foot_print_center;
//     _foot_rf_in_center[2] = _foot_rf[2] + _pos_body[2];

//     _foot_rb_in_center.head(2) = _foot_rb.head(2) - _foot_print_center;
//     _foot_rb_in_center[2] = _foot_rb[2] + _pos_body[2];

//     return _pos_body;

// }

// Vec31<double> BodyPositionEst::bodyPositionEst_P(const Vec31<double> & qleg_lf_P,
//                                                 const Vec31<double> & qleg_lb_P,
//                                                 const Vec31<double> & qleg_rb_P,
//                                                 const Vec31<double> & qleg_rf_P,
//                                                 const Vec41<double>& contactState_P){
//     //Foot position in Base Frame
//     _foot_lf = _sysParam.getFootLocationLF(qleg_lf_P);
//     _foot_lb = _sysParam.getFootLocationLB(qleg_lb_P);
//     _foot_rf = _sysParam.getFootLocationRF(qleg_rf_P);
//     _foot_rb = _sysParam.getFootLocationRB(qleg_rb_P);

//     // ROS_INFO_STREAM("_FOOT_LF: \n"<<_foot_lf);
//     // ROS_INFO_STREAM("_FOOT_LB: \n"<<_foot_lb);
//     // ROS_INFO_STREAM("_FOOT_RF: \n"<<_foot_rf);
//     // ROS_INFO_STREAM("_FOOT_RB: \n"<<_foot_rb);

//     vectorAligned<Vec2<double>> foot_print;
//     foot_print.push_back(_foot_lf.head(2));
//     foot_print.push_back(_foot_lb.head(2));
//     foot_print.push_back(_foot_rb.head(2));
//     foot_print.push_back(_foot_rf.head(2));

//     _foot_print_center = this->getPolygonCenter(foot_print);

//     int num_contact=0;
//     _body_height = 0;

//     if(contactState_P[legID_P::LF]){
//         num_contact++;
//         _body_height += _foot_lf[2];
//         // ROS_INFO("LF IS CONTACT!");
//     }
//     if(contactState_P[legID_P::LB]){
//         num_contact++;
//         _body_height += _foot_lb[2];
//         // ROS_INFO("LB IS CONTACT!");
//     }
//     if(contactState_P[legID_P::RF]){
//         num_contact++;
//         _body_height += _foot_rf[2];
//         // ROS_INFO("RF IS CONTACT!");
//     }
//     if(contactState_P[legID_P::RB]){
//         num_contact++;
//         _body_height += _foot_rb[2];
//         // ROS_INFO("RB IS CONTACT!");
//     }
//     // ROS_INFO_STREAM("body_height: "<<_body_height);
//     _body_height /= (num_contact+0.01);
//     // ROS_INFO_STREAM("NUM_CONTACT: "<<num_contact);


//     _pos_body.head(2) = -_foot_print_center;
//     _pos_body[2] = -_body_height;
//     // ROS_INFO_STREAM("rb_foot_in_foot_print_center: "<<(_foot_rb.head(2) - _foot_print_center ));
//     // ROS_INFO_STREAM("rb_foot_height: "<<(_foot_rb[2] + _pos_body[2]));

//     _foot_lf_in_center.head(2) = _foot_lf.head(2) - _foot_print_center;
//     _foot_lf_in_center[2] = _foot_lf[2] + _pos_body[2];

//     _foot_lb_in_center.head(2) = _foot_lb.head(2) - _foot_print_center;
//     _foot_lb_in_center[2] = _foot_lb[2] + _pos_body[2];

//     _foot_rf_in_center.head(2) = _foot_rf.head(2) - _foot_print_center;
//     _foot_rf_in_center[2] = _foot_rf[2] + _pos_body[2];

//     _foot_rb_in_center.head(2) = _foot_rb.head(2) - _foot_print_center;
//     _foot_rb_in_center[2] = _foot_rb[2] + _pos_body[2];

//     return _pos_body;

// }

// Vec2<double> BodyPositionEst::getPolygonCenter(const vectorAligned<Vec2<double>>&  polygon){
//     Vec2<double> center(0,0);
//     if(polygon.size()>3){
//         double det(0), tempDet(0);
//         int j = 0, nVertice = polygon.size();
//         for(int i(0); i<nVertice;i++){
//             if(i+1 == nVertice)
//                 j = 0;
//             else
//                 j = i+1;
            
//             tempDet = polygon[i][0]*polygon[j][1] - polygon[j][0]*polygon[i][1];
//             det += tempDet;

//             center[0] +=(polygon[i][0] + polygon[j][0])*tempDet;
//             center[1] +=(polygon[i][1] + polygon[j][1])*tempDet;
//         }

//         center[0] /= 3*det;
//         center[1] /= 3*det;
//     }
//     else if(polygon.size()==3){
//         for(int i(0); i<polygon.size(); i++){
//             center[0] += 1/3. * polygon[i][0];
//             center[1] += 1/3. * polygon[i][1];
//         }
//     }
//     return center;
// }
// } //namespace ocs2