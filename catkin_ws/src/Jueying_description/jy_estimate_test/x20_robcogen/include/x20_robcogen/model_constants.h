#ifndef RCG_X20_MODEL_CONSTANTS_H_
#define RCG_X20_MODEL_CONSTANTS_H_

#include "rbd_types.h"

/**
 * \file
 * This file contains the definitions of all the non-zero numerical
 * constants of the robot model (i.e. the numbers appearing in the
 * .kindsl file).
 *
 * Varying these values (and recompiling) is a quick & dirty
 * way to vary the kinematics/dynamics model. For a much more
 * flexible way of exploring variations of the model, consider
 * using the parametrization feature of RobCoGen (see the wiki).
 *
 * Beware of inconsistencies when changing any of the inertia
 * properties.
 */

namespace X20 {
namespace rcg {

// Do not use 'constexpr' to allow for non-literal scalar types

const Scalar tx_LF_HAA = 0.2919999957084656;
const Scalar ty_LF_HAA = 0.07999999821186066;
const Scalar ty_LF_HFE = 0.12325000017881393;
const Scalar ty_LF_KFE = -0.30000001192092896;
const Scalar tx_RF_HAA = 0.2919999957084656;
const Scalar ty_RF_HAA = -0.07999999821186066;
const Scalar ty_RF_HFE = -0.12325000017881393;
const Scalar ty_RF_KFE = -0.30000001192092896;
const Scalar tx_LH_HAA = -0.2919999957084656;
const Scalar ty_LH_HAA = 0.07999999821186066;
const Scalar ty_LH_HFE = 0.12325000017881393;
const Scalar ty_LH_KFE = -0.30000001192092896;
const Scalar tx_RH_HAA = -0.2919999957084656;
const Scalar ty_RH_HAA = -0.07999999821186066;
const Scalar ty_RH_HFE = -0.12325000017881393;
const Scalar ty_RH_KFE = -0.30000001192092896;
const Scalar tx_fr_BASE_COM = 0.004999999888241291;
const Scalar ty_fr_BASE_COM = 4.130000015720725E-4;
const Scalar tz_fr_BASE_COM = 0.004000000189989805;
const Scalar tx_fr_LF_HIP_COM = 5.052800042903982E-5;
const Scalar ty_fr_LF_HIP_COM = -0.00891290046274662;
const Scalar tz_fr_LF_HIP_COM = -7.568899891339242E-4;
const Scalar tx_fr_LF_THIGH_COM = -0.002526399912312627;
const Scalar ty_fr_LF_THIGH_COM = -0.029200000688433647;
const Scalar tz_fr_LF_THIGH_COM = 0.02211800031363964;
const Scalar ty_fr_LF_FOOT = -0.33000001311302185;
const Scalar tx_fr_LF_SHANK_COM = 0.007522400002926588;
const Scalar ty_fr_LF_SHANK_COM = -0.17586000263690948;
const Scalar tz_fr_LF_SHANK_COM = -2.588800089142751E-5;
const Scalar tx_fr_RF_HIP_COM = 5.052800042903982E-5;
const Scalar ty_fr_RF_HIP_COM = 0.008999999612569809;
const Scalar tz_fr_RF_HIP_COM = 7.570000016130507E-4;
const Scalar tx_fr_RF_THIGH_COM = -0.0025265999138355255;
const Scalar ty_fr_RF_THIGH_COM = -0.029193999245762825;
const Scalar tz_fr_RF_THIGH_COM = -0.022119000554084778;
const Scalar ty_fr_RF_FOOT = -0.33000001311302185;
const Scalar tx_fr_RF_SHANK_COM = 0.007522400002926588;
const Scalar ty_fr_RF_SHANK_COM = -0.17586000263690948;
const Scalar tz_fr_RF_SHANK_COM = -2.588800089142751E-5;
const Scalar tx_fr_LH_HIP_COM = -5.053000131738372E-5;
const Scalar ty_fr_LH_HIP_COM = -0.008999999612569809;
const Scalar tz_fr_LH_HIP_COM = 7.570000016130507E-4;
const Scalar tx_fr_LH_THIGH_COM = -0.002526399912312627;
const Scalar ty_fr_LH_THIGH_COM = -0.029200000688433647;
const Scalar tz_fr_LH_THIGH_COM = 0.02211800031363964;
const Scalar ty_fr_LH_FOOT = -0.33000001311302185;
const Scalar tx_fr_LH_SHANK_COM = 0.007522400002926588;
const Scalar ty_fr_LH_SHANK_COM = -0.17586000263690948;
const Scalar tz_fr_LH_SHANK_COM = -2.588800089142751E-5;
const Scalar tx_fr_RH_HIP_COM = -5.052800042903982E-5;
const Scalar ty_fr_RH_HIP_COM = 0.00891290046274662;
const Scalar tz_fr_RH_HIP_COM = -7.568899891339242E-4;
const Scalar tx_fr_RH_THIGH_COM = -0.0025265999138355255;
const Scalar ty_fr_RH_THIGH_COM = -0.029193999245762825;
const Scalar tz_fr_RH_THIGH_COM = -0.022119000554084778;
const Scalar ty_fr_RH_FOOT = -0.33000001311302185;
const Scalar tx_fr_RH_SHANK_COM = 0.007522400002926588;
const Scalar ty_fr_RH_SHANK_COM = -0.17586000263690948;
const Scalar tz_fr_RH_SHANK_COM = -2.5889999960782006E-5;
const Scalar m_BASE = 26.398000717163086;
const Scalar comx_BASE = 0.004999999888241291;
const Scalar comy_BASE = 4.130000015720725E-4;
const Scalar comz_BASE = 0.004000000189989805;
const Scalar ix_BASE = 0.16362687945365906;
const Scalar ixy_BASE = 1.0664787259884179E-4;
const Scalar ixz_BASE = 7.220200495794415E-4;
const Scalar iy_BASE = 0.40756234526634216;
const Scalar iyz_BASE = -3.906949859810993E-5;
const Scalar iz_BASE = 0.5266245007514954;
const Scalar m_LF_HIP = 1.57669997215271;
const Scalar comx_LF_HIP = 5.052800042903982E-5;
const Scalar comy_LF_HIP = -0.00891290046274662;
const Scalar comz_LF_HIP = -7.568899891339242E-4;
const Scalar ix_LF_HIP = 0.0014692560071125627;
const Scalar ixy_LF_HIP = 2.989931772390264E-6;
const Scalar ixz_LF_HIP = 2.8048948479408864E-6;
const Scalar iy_LF_HIP = 0.0016921072965487838;
const Scalar iyz_LF_HIP = 4.437652478372911E-6;
const Scalar iz_LF_HIP = 0.0013393567642197013;
const Scalar m_LF_THIGH = 3.0062999725341797;
const Scalar comx_LF_THIGH = -0.002526399912312627;
const Scalar comy_LF_THIGH = -0.029200000688433647;
const Scalar comz_LF_THIGH = 0.02211800031363964;
const Scalar ix_LF_THIGH = 0.009844191372394562;
const Scalar ixy_LF_THIGH = 2.2482109488919377E-4;
const Scalar ixz_LF_THIGH = -1.2514078116510063E-4;
const Scalar iy_LF_THIGH = 0.0053638881072402;
const Scalar iyz_LF_THIGH = -0.0020104958675801754;
const Scalar iz_LF_THIGH = 0.010651079937815666;
const Scalar m_LF_SHANK = 0.5484899878501892;
const Scalar comx_LF_SHANK = 0.007522400002926588;
const Scalar comy_LF_SHANK = -0.17586000263690948;
const Scalar comz_LF_SHANK = -2.588800089142751E-5;
const Scalar ix_LF_SHANK = 0.02266300655901432;
const Scalar ixy_LF_SHANK = -8.785914978943765E-4;
const Scalar ixz_LF_SHANK = 9.193804828555585E-9;
const Scalar iy_LF_SHANK = 3.210374852642417E-4;
const Scalar iyz_LF_SHANK = 1.3768410553893773E-6;
const Scalar iz_LF_SHANK = 0.022994045168161392;
const Scalar m_RF_HIP = 1.57669997215271;
const Scalar comx_RF_HIP = 5.052800042903982E-5;
const Scalar comy_RF_HIP = 0.008999999612569809;
const Scalar comz_RF_HIP = 7.570000016130507E-4;
const Scalar ix_RF_HIP = 0.0014717162121087313;
const Scalar ixy_RF_HIP = -2.9829927825630875E-6;
const Scalar ixz_RF_HIP = -2.804885980367544E-6;
const Scalar iy_RF_HIP = 0.0016921075293794274;
const Scalar iyz_RF_HIP = 4.543156592262676E-6;
const Scalar iz_RF_HIP = 0.0013418167363852262;
const Scalar m_RF_THIGH = 3.006200075149536;
const Scalar comx_RF_THIGH = -0.0025265999138355255;
const Scalar comy_RF_THIGH = -0.029193999245762825;
const Scalar comz_RF_THIGH = -0.022119000554084778;
const Scalar ix_RF_THIGH = 0.009842936880886555;
const Scalar ixy_RF_THIGH = 2.2481016640085727E-4;
const Scalar ixz_RF_THIGH = 1.2522308679763228E-4;
const Scalar iy_RF_THIGH = 0.005363875068724155;
const Scalar iyz_RF_THIGH = 0.0020100418478250504;
const Scalar iz_RF_THIGH = 0.010649743489921093;
const Scalar m_RF_SHANK = 0.5484899878501892;
const Scalar comx_RF_SHANK = 0.007522400002926588;
const Scalar comy_RF_SHANK = -0.17586000263690948;
const Scalar comz_RF_SHANK = -2.588800089142751E-5;
const Scalar ix_RF_SHANK = 0.02266300655901432;
const Scalar ixy_RF_SHANK = -8.785914978943765E-4;
const Scalar ixz_RF_SHANK = 9.193804828555585E-9;
const Scalar iy_RF_SHANK = 3.210374852642417E-4;
const Scalar iyz_RF_SHANK = 1.3768410553893773E-6;
const Scalar iz_RF_SHANK = 0.022994045168161392;
const Scalar m_LH_HIP = 1.57669997215271;
const Scalar comx_LH_HIP = -5.053000131738372E-5;
const Scalar comy_LH_HIP = -0.008999999612569809;
const Scalar comz_LH_HIP = 7.570000016130507E-4;
const Scalar ix_LH_HIP = 0.0014686161885038018;
const Scalar ixy_LH_HIP = -2.982964360853657E-6;
const Scalar ixz_LH_HIP = 2.8396837024047272E-6;
const Scalar iy_LH_HIP = 0.001700907596386969;
const Scalar iyz_LH_HIP = -4.542056558420882E-6;
const Scalar iz_LH_HIP = 0.0013377167051658034;
const Scalar m_LH_THIGH = 3.0062999725341797;
const Scalar comx_LH_THIGH = -0.002526399912312627;
const Scalar comy_LH_THIGH = -0.029200000688433647;
const Scalar comz_LH_THIGH = 0.02211800031363964;
const Scalar ix_LH_THIGH = 0.009844191372394562;
const Scalar ixy_LH_THIGH = 2.248207019874826E-4;
const Scalar ixz_LH_THIGH = -1.2514078116510063E-4;
const Scalar iy_LH_THIGH = 0.0053638881072402;
const Scalar iyz_LH_THIGH = -0.002010494703426957;
const Scalar iz_LH_THIGH = 0.01058248057961464;
const Scalar m_LH_SHANK = 0.5484899878501892;
const Scalar comx_LH_SHANK = 0.007522400002926588;
const Scalar comy_LH_SHANK = -0.17586000263690948;
const Scalar comz_LH_SHANK = -2.588800089142751E-5;
const Scalar ix_LH_SHANK = 0.02266300655901432;
const Scalar ixy_LH_SHANK = -8.785914978943765E-4;
const Scalar ixz_LH_SHANK = 9.193804828555585E-9;
const Scalar iy_LH_SHANK = 3.210374852642417E-4;
const Scalar iyz_LH_SHANK = 1.3768410553893773E-6;
const Scalar iz_LH_SHANK = 0.022994045168161392;
const Scalar m_RH_HIP = 1.57669997215271;
const Scalar comx_RH_HIP = -5.052800042903982E-5;
const Scalar comy_RH_HIP = 0.00891290046274662;
const Scalar comz_RH_HIP = -7.568899891339242E-4;
const Scalar ix_RH_HIP = 0.0014692560071125627;
const Scalar ixy_RH_HIP = 2.989931772390264E-6;
const Scalar ixz_RH_HIP = -2.8048948479408864E-6;
const Scalar iy_RH_HIP = 0.0016921072965487838;
const Scalar iyz_RH_HIP = -4.437652478372911E-6;
const Scalar iz_RH_HIP = 0.0013393567642197013;
const Scalar m_RH_THIGH = 3.006200075149536;
const Scalar comx_RH_THIGH = -0.0025265999138355255;
const Scalar comy_RH_THIGH = -0.029193999245762825;
const Scalar comz_RH_THIGH = -0.022119000554084778;
const Scalar ix_RH_THIGH = 0.009842936880886555;
const Scalar ixy_RH_THIGH = 2.2481006453745067E-4;
const Scalar ixz_RH_THIGH = 1.2522308679763228E-4;
const Scalar iy_RH_THIGH = 0.005363875068724155;
const Scalar iyz_RH_THIGH = 0.0020100418478250504;
const Scalar iz_RH_THIGH = 0.010649743489921093;
const Scalar m_RH_SHANK = 0.5484899878501892;
const Scalar comx_RH_SHANK = 0.007522400002926588;
const Scalar comy_RH_SHANK = -0.17586000263690948;
const Scalar comz_RH_SHANK = -2.5889999960782006E-5;
const Scalar ix_RH_SHANK = 0.022661106660962105;
const Scalar ixy_RH_SHANK = -8.785114623606205E-4;
const Scalar ixz_RH_SHANK = 9.265558986726319E-9;
const Scalar iy_RH_SHANK = 3.1925749499350786E-4;
const Scalar iyz_RH_SHANK = 1.3727380974160042E-6;
const Scalar iz_RH_SHANK = 0.022896643728017807;

}
}
#endif
