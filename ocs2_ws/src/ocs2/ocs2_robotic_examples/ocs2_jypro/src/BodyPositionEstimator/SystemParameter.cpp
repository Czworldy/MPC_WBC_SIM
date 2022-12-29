// #include "SystemParameter.h"

// namespace ocs2 {
// template<typename T>
// SystemParameter<T>::SystemParameter(){
//     hipLocation[0] << -0.135, 0.33, 0; //LF
//     hipLocation[1] << -0.135, -0.33, 0; //LB
//     hipLocation[2] << 0.135, -0.33, 0; //RB
//     hipLocation[3] << 0.135, 0.33, 0; //RF

//     deviation = 0.042;
//     thigh = 0.33;
//     shank = 0.33;

//     footLocation << shank,0,0; //Not in urdf , but in D-H;
//     footLocation_q << shank,0,0,1; //Not in urdf , but in D-H;
// } 

// template<typename T>
// SystemParameter<T>::~SystemParameter(){}

// template<typename T>
// Vec31<T> SystemParameter<T>::getFootLocation(const Vec31<T>& q, size_t legID){
//     Vec31<T> foot;
//     if(legID == legID_P::LF)
//         foot = getFootLocationLF(q);
//     else if (legID == legID_P::LB)
//         foot = getFootLocationLB(q);
//     else if (legID == legID_P::RB)
//         foot = getFootLocationRB(q);
//     else if (legID == legID_P::RF)
//         foot = getFootLocationRF(q);

//     return foot;
// }

// //q_LF[0]---------LF_HipX
// //q_LF[1]---------LF_HipY
// //q_LF[2]---------LF_Knee
// template<typename T>
// Vec31<T> SystemParameter<T>::getFootLocationLF(const Vec31<T>& q_LF){
//     Mat4<T> T10, T21, T32, T_LF0;
//     T10 << cos(q_LF[0]), -sin(q_LF[0]), 0., 0.,
//                    sin(q_LF[0]),  cos(q_LF[0]), 0., 0.,
//                          0.,                           0.,             1., 0.,
//                          0.,                           0.,             0., 1.;

//     T21 << -cos(q_LF[1]), sin(q_LF[1]), 0., 0.,
//                           0.,                         0.,              1., -deviation,
//                     sin(q_LF[1]), cos(q_LF[1]), 0., 0.,
//                           0.,                         0.,              0., 1.;
    
//     T32 << cos(q_LF[2]), -sin(q_LF[2]), 0., thigh,
//                    sin(q_LF[2]),  cos(q_LF[2]), 0.,      0.,
//                              0.,                      0.,              1.,      0.,
//                              0.,                      0.,              0.,      1.;

//     T_LF0 << 0., 1., 0., -0.135,
//                         0., 0., 1.,  0.33,
//                         1., 0., 0.,   0.,
//                         0., 0., 0.,   1.;
    
//     return (T_LF0*T10*T21*T32*footLocation_q).head(3);
// }

// template<typename T>
// Vec31<T> SystemParameter<T>::getFootLocationLB(const Vec31<T>& q_LB){
//     Mat4<T> T10, T21, T32, T_LB0;
//     T10 << cos(q_LB[0]), -sin(q_LB[0]), 0., 0.,
//                    sin(q_LB[0]),  cos(q_LB[0]), 0., 0.,
//                          0.,                           0.,             1., 0.,
//                          0.,                           0.,             0., 1.;

//     T21 << -cos(q_LB[1]), sin(q_LB[1]), 0., 0.,
//                           0.,                         0.,              1., -deviation,
//                     sin(q_LB[1]), cos(q_LB[1]), 0., 0.,
//                           0.,                         0.,              0., 1.;
    
//     T32 << cos(q_LB[2]), -sin(q_LB[2]), 0., thigh,
//                    sin(q_LB[2]),  cos(q_LB[2]), 0.,      0.,
//                              0.,                      0.,              1.,      0.,
//                              0.,                      0.,              0.,      1.;

//     T_LB0 << 0., 1., 0., -0.135,
//                         0., 0., 1.,  -0.33,
//                         1., 0., 0.,   0.,
//                         0., 0., 0.,   1.;
    
//     return (T_LB0*T10*T21*T32*footLocation_q).head(3);
// }

// template<typename T>
// Vec31<T> SystemParameter<T>::getFootLocationRB(const Vec31<T>& q_RB){
//     Mat4<T> T10, T21, T32, T_RB0;
//     T10 << cos(q_RB[0]), -sin(q_RB[0]), 0., 0.,
//                    sin(q_RB[0]),  cos(q_RB[0]), 0., 0.,
//                          0.,                           0.,             1., 0.,
//                          0.,                           0.,             0., 1.;

//     T21 << -cos(q_RB[1]), sin(q_RB[1]), 0., 0.,
//                           0.,                         0.,              -1., -deviation,
//                     -sin(q_RB[1]), -cos(q_RB[1]), 0., 0.,
//                           0.,                         0.,              0., 1.;
    
//     T32 << cos(q_RB[2]), -sin(q_RB[2]), 0., thigh,
//                    sin(q_RB[2]),  cos(q_RB[2]), 0.,      0.,
//                              0.,                      0.,              1.,      0.,
//                              0.,                      0.,              0.,      1.;

//     T_RB0 << 0., -1., 0., 0.135,
//                         0., 0., -1.,  -0.33,
//                         1., 0., 0.,   0.,
//                         0., 0., 0.,   1.;
    
//     return (T_RB0*T10*T21*T32*footLocation_q).head(3);
// }

// template<typename T>
// Vec31<T> SystemParameter<T>::getFootLocationRF(const Vec31<T>& q_RF){
//     Mat4<T> T10, T21, T32, T_RF0;
//     T10 << cos(q_RF[0]), -sin(q_RF[0]), 0., 0.,
//                    sin(q_RF[0]),  cos(q_RF[0]), 0., 0.,
//                          0.,                           0.,             1., 0.,
//                          0.,                           0.,             0., 1.;

//     T21 << -cos(q_RF[1]), sin(q_RF[1]), 0., 0.,
//                           0.,                         0.,              -1., -deviation,
//                     -sin(q_RF[1]), -cos(q_RF[1]), 0., 0.,
//                           0.,                         0.,              0., 1.;
    
//     T32 << cos(q_RF[2]), -sin(q_RF[2]), 0., thigh,
//                    sin(q_RF[2]),  cos(q_RF[2]), 0.,      0.,
//                              0.,                      0.,              1.,      0.,
//                              0.,                      0.,              0.,      1.;

//     T_RF0 << 0., -1., 0., 0.135,
//                         0., 0., -1.,  0.33,
//                         1., 0., 0.,   0.,
//                         0., 0., 0.,   1.;
    
//     return (T_RF0*T10*T21*T32*footLocation_q).head(3);
// }

// template<typename T>
// Mat3<T> SystemParameter<T>::getFootJacobianLF(const Vec31<T>& q_LF){
//     Mat3<T> jaco_hip, jaco_body;
//     Mat4<T> T10, T21, T32, T_LF0, T3bar_3;
//     T10 << cos(q_LF[0]), -sin(q_LF[0]), 0., 0.,
//                    sin(q_LF[0]),  cos(q_LF[0]), 0., 0.,
//                          0.,                           0.,             1., 0.,
//                          0.,                           0.,             0., 1.;

//     T21 << -cos(q_LF[1]), sin(q_LF[1]), 0., 0.,
//                           0.,                         0.,              1., -deviation,
//                     sin(q_LF[1]), cos(q_LF[1]), 0., 0.,
//                           0.,                         0.,              0., 1.;
    
//     T32 << cos(q_LF[2]), -sin(q_LF[2]), 0., thigh,
//                    sin(q_LF[2]),  cos(q_LF[2]), 0.,      0.,
//                              0.,                      0.,              1.,      0.,
//                              0.,                      0.,              0.,      1.;

//     T_LF0 << 0., 1., 0., -0.135,
//                         0., 0., 1.,  0.33,
//                         1., 0., 0.,   0.,
//                         0., 0., 0.,   1.;
    
//     T3bar_3 <<  1., 0., 0., shank,
//                              0., 1., 0., 0.,
//                              0., 0., 1., 0.,
//                              0., 0., 0., 1.;

//     Mat4<T> T3bar_2, T3bar_1;
//     T3bar_2 = T32 * T3bar_3;
//     T3bar_1 =  T21 * T3bar_2;

//     //HIP_X
//     jaco_hip(0,0) = T3bar_1(0,3)*T3bar_1(1,0) - T3bar_1(1,3)*T3bar_1(0,0);
//     jaco_hip(1,0) = T3bar_1(0,3)*T3bar_1(1,1) - T3bar_1(1,3)*T3bar_1(0,1);
//     jaco_hip(2,0) = T3bar_1(0,3)*T3bar_1(1,2) - T3bar_1(1,3)*T3bar_1(0,2);
//     //HIP_Y
//     jaco_hip(0,1) = T3bar_2(0,3)*T3bar_2(1,0) - T3bar_2(1,3)*T3bar_2(0,0);
//     jaco_hip(1,1) = T3bar_2(0,3)*T3bar_2(1,1) - T3bar_2(1,3)*T3bar_2(0,1);
//     jaco_hip(2,1) = T3bar_2(0,3)*T3bar_2(1,2) - T3bar_2(1,3)*T3bar_2(0,2);
//     //KNEE
//     jaco_hip(0,2) = 0;
//     jaco_hip(1,2) = shank;
//     jaco_hip(2,2) = 0;
     
//     jaco_body = T_LF0.block(0,0,3,3) * jaco_hip;
//     return jaco_body;
// }

// template<typename T>
// Mat3<T> SystemParameter<T>::getFootJacobianLB(const Vec31<T>& q_LB){
//     Mat3<T> jaco_hip, jaco_body;
//     Mat4<T> T10, T21, T32, T_LB0, T3bar_3;
//     T10 << cos(q_LB[0]), -sin(q_LB[0]), 0., 0.,
//                    sin(q_LB[0]),  cos(q_LB[0]), 0., 0.,
//                          0.,                           0.,             1., 0.,
//                          0.,                           0.,             0., 1.;

//     T21 << -cos(q_LB[1]), sin(q_LB[1]), 0., 0.,
//                           0.,                         0.,              1., -deviation,
//                     sin(q_LB[1]), cos(q_LB[1]), 0., 0.,
//                           0.,                         0.,              0., 1.;
    
//     T32 << cos(q_LB[2]), -sin(q_LB[2]), 0., thigh,
//                    sin(q_LB[2]),  cos(q_LB[2]), 0.,      0.,
//                              0.,                      0.,              1.,      0.,
//                              0.,                      0.,              0.,      1.;

//     T_LB0 << 0., 1., 0., -0.135,
//                         0., 0., 1.,  -0.33,
//                         1., 0., 0.,   0.,
//                         0., 0., 0.,   1.;
    
//     T3bar_3 <<  1., 0., 0., shank,
//                              0., 1., 0., 0.,
//                              0., 0., 1., 0.,
//                              0., 0., 0., 1.;

//     Mat4<T> T3bar_2, T3bar_1;
//     T3bar_2 = T32 * T3bar_3;
//     T3bar_1 =  T21 * T3bar_2;

//     //HIP_X
//     jaco_hip(0,0) = T3bar_1(0,3)*T3bar_1(1,0) - T3bar_1(1,3)*T3bar_1(0,0);
//     jaco_hip(1,0) = T3bar_1(0,3)*T3bar_1(1,1) - T3bar_1(1,3)*T3bar_1(0,1);
//     jaco_hip(2,0) = T3bar_1(0,3)*T3bar_1(1,2) - T3bar_1(1,3)*T3bar_1(0,2);
//     //HIP_Y
//     jaco_hip(0,1) = T3bar_2(0,3)*T3bar_2(1,0) - T3bar_2(1,3)*T3bar_2(0,0);
//     jaco_hip(1,1) = T3bar_2(0,3)*T3bar_2(1,1) - T3bar_2(1,3)*T3bar_2(0,1);
//     jaco_hip(2,1) = T3bar_2(0,3)*T3bar_2(1,2) - T3bar_2(1,3)*T3bar_2(0,2);
//     //KNEE
//     jaco_hip(0,2) = 0;
//     jaco_hip(1,2) = shank;
//     jaco_hip(2,2) = 0;
     
//     jaco_body = T_LB0.block(0,0,3,3) * jaco_hip;
//     return jaco_body;
// }

// template<typename T>
// Mat3<T> SystemParameter<T>::getFootJacobianRB(const Vec31<T>& q_RB){
//     Mat3<T> jaco_hip, jaco_body;
//     Mat4<T> T10, T21, T32, T_RB0, T3bar_3;
//     T10 << cos(q_RB[0]), -sin(q_RB[0]), 0., 0.,
//                    sin(q_RB[0]),  cos(q_RB[0]), 0., 0.,
//                          0.,                           0.,             1., 0.,
//                          0.,                           0.,             0., 1.;

//     T21 << -cos(q_RB[1]), sin(q_RB[1]), 0., 0.,
//                           0.,                         0.,              -1., -deviation,
//                     -sin(q_RB[1]), -cos(q_RB[1]), 0., 0.,
//                           0.,                         0.,              0., 1.;
    
//     T32 << cos(q_RB[2]), -sin(q_RB[2]), 0., thigh,
//                    sin(q_RB[2]),  cos(q_RB[2]), 0.,      0.,
//                              0.,                      0.,              1.,      0.,
//                              0.,                      0.,              0.,      1.;

//     T_RB0 << 0., -1., 0., 0.135,
//                         0., 0., -1.,  -0.33,
//                         1., 0., 0.,   0.,
//                         0., 0., 0.,   1.;

//     T3bar_3 <<  1., 0., 0., shank,
//                              0., 1., 0., 0.,
//                              0., 0., 1., 0.,
//                              0., 0., 0., 1.;

//     Mat4<T> T3bar_2, T3bar_1;
//     T3bar_2 = T32 * T3bar_3;
//     T3bar_1 =  T21 * T3bar_2;

//     //HIP_X
//     jaco_hip(0,0) = T3bar_1(0,3)*T3bar_1(1,0) - T3bar_1(1,3)*T3bar_1(0,0);
//     jaco_hip(1,0) = T3bar_1(0,3)*T3bar_1(1,1) - T3bar_1(1,3)*T3bar_1(0,1);
//     jaco_hip(2,0) = T3bar_1(0,3)*T3bar_1(1,2) - T3bar_1(1,3)*T3bar_1(0,2);
//     //HIP_Y
//     jaco_hip(0,1) = T3bar_2(0,3)*T3bar_2(1,0) - T3bar_2(1,3)*T3bar_2(0,0);
//     jaco_hip(1,1) = T3bar_2(0,3)*T3bar_2(1,1) - T3bar_2(1,3)*T3bar_2(0,1);
//     jaco_hip(2,1) = T3bar_2(0,3)*T3bar_2(1,2) - T3bar_2(1,3)*T3bar_2(0,2);
//     //KNEE
//     jaco_hip(0,2) = 0;
//     jaco_hip(1,2) = shank;
//     jaco_hip(2,2) = 0;
     
//     jaco_body = T_RB0.block(0,0,3,3) * jaco_hip;
//     return jaco_body;
// }

// template<typename T>
// Mat3<T> SystemParameter<T>::getFootJacobianRF(const Vec31<T>& q_RF){
//     Mat3<T> jaco_hip, jaco_body;
//     Mat4<T> T10, T21, T32, T_RF0, T3bar_3;
//     T10 << cos(q_RF[0]), -sin(q_RF[0]), 0., 0.,
//                    sin(q_RF[0]),  cos(q_RF[0]), 0., 0.,
//                          0.,                           0.,             1., 0.,
//                          0.,                           0.,             0., 1.;

//     T21 << -cos(q_RF[1]), sin(q_RF[1]), 0., 0.,
//                           0.,                         0.,              -1., -deviation,
//                     -sin(q_RF[1]), -cos(q_RF[1]), 0., 0.,
//                           0.,                         0.,              0., 1.;
    
//     T32 << cos(q_RF[2]), -sin(q_RF[2]), 0., thigh,
//                    sin(q_RF[2]),  cos(q_RF[2]), 0.,      0.,
//                              0.,                      0.,              1.,      0.,
//                              0.,                      0.,              0.,      1.;

//     T_RF0 << 0., -1., 0., 0.135,
//                         0., 0., -1.,  0.33,
//                         1., 0., 0.,   0.,
//                         0., 0., 0.,   1.;
    
//     T3bar_3 <<  1., 0., 0., shank,
//                              0., 1., 0., 0.,
//                              0., 0., 1., 0.,
//                              0., 0., 0., 1.;

//     Mat4<T> T3bar_2, T3bar_1;
//     T3bar_2 = T32 * T3bar_3;
//     T3bar_1 =  T21 * T3bar_2;

//     //HIP_X
//     jaco_hip(0,0) = T3bar_1(0,3)*T3bar_1(1,0) - T3bar_1(1,3)*T3bar_1(0,0);
//     jaco_hip(1,0) = T3bar_1(0,3)*T3bar_1(1,1) - T3bar_1(1,3)*T3bar_1(0,1);
//     jaco_hip(2,0) = T3bar_1(0,3)*T3bar_1(1,2) - T3bar_1(1,3)*T3bar_1(0,2);
//     //HIP_Y
//     jaco_hip(0,1) = T3bar_2(0,3)*T3bar_2(1,0) - T3bar_2(1,3)*T3bar_2(0,0);
//     jaco_hip(1,1) = T3bar_2(0,3)*T3bar_2(1,1) - T3bar_2(1,3)*T3bar_2(0,1);
//     jaco_hip(2,1) = T3bar_2(0,3)*T3bar_2(1,2) - T3bar_2(1,3)*T3bar_2(0,2);
//     //KNEE
//     jaco_hip(0,2) = 0;
//     jaco_hip(1,2) = shank;
//     jaco_hip(2,2) = 0;
     
//     jaco_body = T_RF0.block(0,0,3,3) * jaco_hip;
//     return jaco_body;
// }


// template class SystemParameter<double>;
// template class SystemParameter<float>;
// }