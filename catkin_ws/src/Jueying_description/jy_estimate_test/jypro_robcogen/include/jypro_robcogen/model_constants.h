#ifndef RCG_JYPRO_MODEL_CONSTANTS_H_
#define RCG_JYPRO_MODEL_CONSTANTS_H_

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

namespace JYPro {
namespace rcg {

// Do not use 'constexpr' to allow for non-literal scalar types

const Scalar rx_RF_HFE = 4.71238899230957;
const Scalar sin_rx_RF_HFE = ScalarTraits::sin(rx_RF_HFE);
const Scalar cos_rx_RF_HFE = ScalarTraits::cos(rx_RF_HFE);
const Scalar rx_RH_HFE = 4.71238899230957;
const Scalar sin_rx_RH_HFE = ScalarTraits::sin(rx_RH_HFE);
const Scalar cos_rx_RH_HFE = ScalarTraits::cos(rx_RH_HFE);
const Scalar tx_LF_HAA = -0.13500000536441803;
const Scalar ty_LF_HAA = 0.33000001311302185;
const Scalar tx_LF_HFE = -0.041999999433755875;
const Scalar tx_LF_KFE = 0.33000001311302185;
const Scalar ty_LF_KFE = 1.0000000036274937E-15;
const Scalar tx_RF_HAA = 0.13500000536441803;
const Scalar ty_RF_HAA = 0.33000001311302185;
const Scalar tx_RF_HFE = 0.041999999433755875;
const Scalar tx_RF_KFE = 0.33000001311302185;
const Scalar ty_RF_KFE = 1.0000000036274937E-15;
const Scalar tx_LH_HAA = -0.13500000536441803;
const Scalar ty_LH_HAA = -0.33000001311302185;
const Scalar tx_LH_HFE = -0.041999999433755875;
const Scalar tx_LH_KFE = 0.33000001311302185;
const Scalar ty_LH_KFE = 1.0000000036274937E-15;
const Scalar tx_RH_HAA = 0.13500000536441803;
const Scalar ty_RH_HAA = -0.33000001311302185;
const Scalar tx_RH_HFE = 0.041999999433755875;
const Scalar tx_RH_KFE = 0.33000001311302185;
const Scalar ty_RH_KFE = 1.0000000036274937E-15;
const Scalar tx_fr_BASE_COM = 4.3154999730177224E-4;
const Scalar ty_fr_BASE_COM = 0.0392180010676384;
const Scalar tz_fr_BASE_COM = 0.025123000144958496;
const Scalar tx_fr_LF_HIP_COM = 0.02689066343009472;
const Scalar ty_fr_LF_HIP_COM = -0.0011362772202119231;
const Scalar tz_fr_LF_HIP_COM = 4.144858758081682E-5;
const Scalar tx_fr_LF_THIGH_COM = 0.07269000262022018;
const Scalar ty_fr_LF_THIGH_COM = -0.007757700048387051;
const Scalar tz_fr_LF_THIGH_COM = -0.019015999510884285;
const Scalar tx_fr_LF_FOOT = 0.33000001311302185;
const Scalar ty_fr_LF_FOOT = 1.0000000036274937E-15;
const Scalar tx_fr_LF_SHANK_COM = 0.14343999326229095;
const Scalar ty_fr_LF_SHANK_COM = 0.0014413000317290425;
const Scalar tz_fr_LF_SHANK_COM = -1.657699976931326E-5;
const Scalar tx_fr_RF_HIP_COM = -0.02689066343009472;
const Scalar ty_fr_RF_HIP_COM = 0.0011362772202119231;
const Scalar tz_fr_RF_HIP_COM = 4.1448594856774434E-5;
const Scalar tx_fr_RF_THIGH_COM = 0.07269000262022018;
const Scalar ty_fr_RF_THIGH_COM = -0.007757700048387051;
const Scalar tz_fr_RF_THIGH_COM = 0.019015999510884285;
const Scalar tx_fr_RF_FOOT = 0.33000001311302185;
const Scalar ty_fr_RF_FOOT = 1.0000000036274937E-15;
const Scalar tx_fr_RF_SHANK_COM = 0.14343999326229095;
const Scalar ty_fr_RF_SHANK_COM = 0.0014413000317290425;
const Scalar tz_fr_RF_SHANK_COM = -1.657699976931326E-5;
const Scalar tx_fr_LH_HIP_COM = 0.02689066343009472;
const Scalar ty_fr_LH_HIP_COM = -0.0011362772202119231;
const Scalar tz_fr_LH_HIP_COM = 4.1448580304859206E-5;
const Scalar tx_fr_LH_THIGH_COM = 0.07269003242254257;
const Scalar ty_fr_LH_THIGH_COM = -0.007757717743515968;
const Scalar tz_fr_LH_THIGH_COM = -0.019015999510884285;
const Scalar tx_fr_LH_FOOT = 0.33000001311302185;
const Scalar ty_fr_LH_FOOT = 1.0000000036274937E-15;
const Scalar tx_fr_LH_SHANK_COM = 0.14343999326229095;
const Scalar ty_fr_LH_SHANK_COM = 0.0014413000317290425;
const Scalar tz_fr_LH_SHANK_COM = -1.657699976931326E-5;
const Scalar tx_fr_RH_HIP_COM = -0.02689066343009472;
const Scalar ty_fr_RH_HIP_COM = 0.0011362772202119231;
const Scalar tz_fr_RH_HIP_COM = 4.14485766668804E-5;
const Scalar tx_fr_RH_THIGH_COM = 0.07269000262022018;
const Scalar ty_fr_RH_THIGH_COM = -0.007757700048387051;
const Scalar tz_fr_RH_THIGH_COM = 0.019015999510884285;
const Scalar tx_fr_RH_FOOT = 0.33000001311302185;
const Scalar ty_fr_RH_FOOT = 1.0000000036274937E-15;
const Scalar tx_fr_RH_SHANK_COM = 0.14343670010566711;
const Scalar ty_fr_RH_SHANK_COM = 0.0014413081808015704;
const Scalar tz_fr_RH_SHANK_COM = -1.6577043425058946E-5;
const Scalar m_BASE = 31.086000442504883;
const Scalar comx_BASE = 4.3154999730177224E-4;
const Scalar comy_BASE = 0.0392180010676384;
const Scalar comz_BASE = 0.025123000144958496;
const Scalar ix_BASE = 2.282216787338257;
const Scalar ixy_BASE = -0.001753631280735135;
const Scalar ixz_BASE = -6.377218523994088E-4;
const Scalar iy_BASE = 0.2907680869102478;
const Scalar iyz_BASE = 0.15675657987594604;
const Scalar iz_BASE = 2.2293882369995117;
const Scalar m_LF_HIP = 2.5054919719696045;
const Scalar comx_LF_HIP = 0.02689066343009472;
const Scalar comy_LF_HIP = -0.0011362772202119231;
const Scalar comz_LF_HIP = 4.144858758081682E-5;
const Scalar ix_LF_HIP = 0.0056823561899363995;
const Scalar ixy_LF_HIP = 3.718729567481205E-5;
const Scalar ixz_LF_HIP = 6.523941010527778E-6;
const Scalar iy_LF_HIP = 0.006759366951882839;
const Scalar iyz_LF_HIP = 7.439266482833773E-7;
const Scalar iz_LF_HIP = 0.005939791910350323;
const Scalar m_LF_THIGH = 4.5005998611450195;
const Scalar comx_LF_THIGH = 0.07269000262022018;
const Scalar comy_LF_THIGH = -0.007757700048387051;
const Scalar comz_LF_THIGH = -0.019015999510884285;
const Scalar ix_LF_THIGH = 0.012722307816147804;
const Scalar ixy_LF_THIGH = -0.005804118234664202;
const Scalar ixz_LF_THIGH = -4.269573255442083E-4;
const Scalar iy_LF_THIGH = 0.0843958854675293;
const Scalar iyz_LF_THIGH = -1.323796750511974E-4;
const Scalar iz_LF_THIGH = 0.0859442874789238;
const Scalar m_LF_SHANK = 0.5067200064659119;
const Scalar comx_LF_SHANK = 0.14343999326229095;
const Scalar comy_LF_SHANK = 0.0014413000317290425;
const Scalar comz_LF_SHANK = -1.657699976931326E-5;
const Scalar ix_LF_SHANK = 2.209027879871428E-4;
const Scalar ixy_LF_SHANK = 3.322898701298982E-4;
const Scalar ixz_LF_SHANK = -2.8333013233350357E-6;
const Scalar iy_LF_SHANK = 0.022641779854893684;
const Scalar iyz_LF_SHANK = 1.4907092449334414E-8;
const Scalar iz_LF_SHANK = 0.022780831903219223;
const Scalar m_RF_HIP = 2.5054919719696045;
const Scalar comx_RF_HIP = -0.02689066343009472;
const Scalar comy_RF_HIP = 0.0011362772202119231;
const Scalar comz_RF_HIP = 4.1448594856774434E-5;
const Scalar ix_RF_HIP = 0.0056823561899363995;
const Scalar ixy_RF_HIP = 3.718729567481205E-5;
const Scalar ixz_RF_HIP = -6.52393146083341E-6;
const Scalar iy_RF_HIP = 0.006759366951882839;
const Scalar iyz_RF_HIP = -7.439986688950739E-7;
const Scalar iz_RF_HIP = 0.005939791910350323;
const Scalar m_RF_THIGH = 4.5005998611450195;
const Scalar comx_RF_THIGH = 0.07269000262022018;
const Scalar comy_RF_THIGH = -0.007757700048387051;
const Scalar comz_RF_THIGH = 0.019015999510884285;
const Scalar ix_RF_THIGH = 0.012722308747470379;
const Scalar ixy_RF_THIGH = -0.005804119165986776;
const Scalar ixz_RF_THIGH = 4.2705846135504544E-4;
const Scalar iy_RF_THIGH = 0.0843958854675293;
const Scalar iyz_RF_THIGH = 1.323690521530807E-4;
const Scalar iz_RF_THIGH = 0.0859442874789238;
const Scalar m_RF_SHANK = 0.5067200064659119;
const Scalar comx_RF_SHANK = 0.14343999326229095;
const Scalar comy_RF_SHANK = 0.0014413000317290425;
const Scalar comz_RF_SHANK = -1.657699976931326E-5;
const Scalar ix_RF_SHANK = 2.209027879871428E-4;
const Scalar ixy_RF_SHANK = 3.322898701298982E-4;
const Scalar ixz_RF_SHANK = -2.8333013233350357E-6;
const Scalar iy_RF_SHANK = 0.022641779854893684;
const Scalar iyz_RF_SHANK = 1.4907092449334414E-8;
const Scalar iz_RF_SHANK = 0.022780831903219223;
const Scalar m_LH_HIP = 2.5054919719696045;
const Scalar comx_LH_HIP = 0.02689066343009472;
const Scalar comy_LH_HIP = -0.0011362772202119231;
const Scalar comz_LH_HIP = 4.1448580304859206E-5;
const Scalar ix_LH_HIP = 0.0056823561899363995;
const Scalar ixy_LH_HIP = 3.718729567481205E-5;
const Scalar ixz_LH_HIP = 6.523939646285726E-6;
const Scalar iy_LH_HIP = 0.006759366951882839;
const Scalar iyz_LH_HIP = 7.439266482833773E-7;
const Scalar iz_LH_HIP = 0.005939791910350323;
const Scalar m_LH_THIGH = 4.500598907470703;
const Scalar comx_LH_THIGH = 0.07269003242254257;
const Scalar comy_LH_THIGH = -0.007757717743515968;
const Scalar comz_LH_THIGH = -0.019015999510884285;
const Scalar ix_LH_THIGH = 0.012722297571599483;
const Scalar ixy_LH_THIGH = -0.005804082844406366;
const Scalar ixz_LH_THIGH = -4.2700249468907714E-4;
const Scalar iy_LH_THIGH = 0.0843958631157875;
const Scalar iyz_LH_THIGH = -1.323769974987954E-4;
const Scalar iz_LH_THIGH = 0.08594411611557007;
const Scalar m_LH_SHANK = 0.5067200064659119;
const Scalar comx_LH_SHANK = 0.14343999326229095;
const Scalar comy_LH_SHANK = 0.0014413000317290425;
const Scalar comz_LH_SHANK = -1.657699976931326E-5;
const Scalar ix_LH_SHANK = 2.209027879871428E-4;
const Scalar ixy_LH_SHANK = 3.322898701298982E-4;
const Scalar ixz_LH_SHANK = -2.8333013233350357E-6;
const Scalar iy_LH_SHANK = 0.022641779854893684;
const Scalar iyz_LH_SHANK = 1.4907092449334414E-8;
const Scalar iz_LH_SHANK = 0.022780831903219223;
const Scalar m_RH_HIP = 2.5054919719696045;
const Scalar comx_RH_HIP = -0.02689066343009472;
const Scalar comy_RH_HIP = 0.0011362772202119231;
const Scalar comz_RH_HIP = 4.14485766668804E-5;
const Scalar ix_RH_HIP = 0.0056823561899363995;
const Scalar ixy_RH_HIP = 3.718729567481205E-5;
const Scalar ixz_RH_HIP = -6.523930096591357E-6;
const Scalar iy_RH_HIP = 0.006759366951882839;
const Scalar iyz_RH_HIP = -7.439984983648174E-7;
const Scalar iz_RH_HIP = 0.005939791910350323;
const Scalar m_RH_THIGH = 4.5005998611450195;
const Scalar comx_RH_THIGH = 0.07269000262022018;
const Scalar comy_RH_THIGH = -0.007757700048387051;
const Scalar comz_RH_THIGH = 0.019015999510884285;
const Scalar ix_RH_THIGH = 0.012722308747470379;
const Scalar ixy_RH_THIGH = -0.005804119165986776;
const Scalar ixz_RH_THIGH = 4.2705846135504544E-4;
const Scalar iy_RH_THIGH = 0.0843958854675293;
const Scalar iyz_RH_THIGH = 1.323690521530807E-4;
const Scalar iz_RH_THIGH = 0.0859442874789238;
const Scalar m_RH_SHANK = 0.5067174434661865;
const Scalar comx_RH_SHANK = 0.14343670010566711;
const Scalar comy_RH_SHANK = 0.0014413081808015704;
const Scalar comz_RH_SHANK = -1.6577043425058946E-5;
const Scalar ix_RH_SHANK = 2.209012018283829E-4;
const Scalar ixy_RH_SHANK = 3.322838747408241E-4;
const Scalar ixz_RH_SHANK = -2.8333038244454656E-6;
const Scalar iy_RH_SHANK = 0.022641584277153015;
const Scalar iyz_RH_SHANK = 1.490668743997503E-8;
const Scalar iz_RH_SHANK = 0.022780176252126694;

}
}
#endif
