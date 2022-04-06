#ifndef RCG_JYPRO_INERTIA_PROPERTIES_H_
#define RCG_JYPRO_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "model_constants.h"
#include "dynamics_parameters.h"

namespace JYPro {
namespace rcg {

class InertiaProperties {
    public:
        InertiaProperties();
        ~InertiaProperties();
        const InertiaMatrix& getTensor_BASE() const;
        const InertiaMatrix& getTensor_LF_HIP() const;
        const InertiaMatrix& getTensor_LF_THIGH() const;
        const InertiaMatrix& getTensor_LF_SHANK() const;
        const InertiaMatrix& getTensor_RF_HIP() const;
        const InertiaMatrix& getTensor_RF_THIGH() const;
        const InertiaMatrix& getTensor_RF_SHANK() const;
        const InertiaMatrix& getTensor_LH_HIP() const;
        const InertiaMatrix& getTensor_LH_THIGH() const;
        const InertiaMatrix& getTensor_LH_SHANK() const;
        const InertiaMatrix& getTensor_RH_HIP() const;
        const InertiaMatrix& getTensor_RH_THIGH() const;
        const InertiaMatrix& getTensor_RH_SHANK() const;
        Scalar getMass_BASE() const;
        Scalar getMass_LF_HIP() const;
        Scalar getMass_LF_THIGH() const;
        Scalar getMass_LF_SHANK() const;
        Scalar getMass_RF_HIP() const;
        Scalar getMass_RF_THIGH() const;
        Scalar getMass_RF_SHANK() const;
        Scalar getMass_LH_HIP() const;
        Scalar getMass_LH_THIGH() const;
        Scalar getMass_LH_SHANK() const;
        Scalar getMass_RH_HIP() const;
        Scalar getMass_RH_THIGH() const;
        Scalar getMass_RH_SHANK() const;
        const Vector3& getCOM_BASE() const;
        const Vector3& getCOM_LF_HIP() const;
        const Vector3& getCOM_LF_THIGH() const;
        const Vector3& getCOM_LF_SHANK() const;
        const Vector3& getCOM_RF_HIP() const;
        const Vector3& getCOM_RF_THIGH() const;
        const Vector3& getCOM_RF_SHANK() const;
        const Vector3& getCOM_LH_HIP() const;
        const Vector3& getCOM_LH_THIGH() const;
        const Vector3& getCOM_LH_SHANK() const;
        const Vector3& getCOM_RH_HIP() const;
        const Vector3& getCOM_RH_THIGH() const;
        const Vector3& getCOM_RH_SHANK() const;
        Scalar getTotalMass() const;


        /*!
         * Fresh values for the runtime parameters of the robot JYPro,
         * causing the update of the inertia properties modeled by this
         * instance.
         */
        void updateParameters(const RuntimeInertiaParams&);

    private:
        RuntimeInertiaParams params;

        InertiaMatrix tensor_BASE;
        InertiaMatrix tensor_LF_HIP;
        InertiaMatrix tensor_LF_THIGH;
        InertiaMatrix tensor_LF_SHANK;
        InertiaMatrix tensor_RF_HIP;
        InertiaMatrix tensor_RF_THIGH;
        InertiaMatrix tensor_RF_SHANK;
        InertiaMatrix tensor_LH_HIP;
        InertiaMatrix tensor_LH_THIGH;
        InertiaMatrix tensor_LH_SHANK;
        InertiaMatrix tensor_RH_HIP;
        InertiaMatrix tensor_RH_THIGH;
        InertiaMatrix tensor_RH_SHANK;
        Vector3 com_BASE;
        Vector3 com_LF_HIP;
        Vector3 com_LF_THIGH;
        Vector3 com_LF_SHANK;
        Vector3 com_RF_HIP;
        Vector3 com_RF_THIGH;
        Vector3 com_RF_SHANK;
        Vector3 com_LH_HIP;
        Vector3 com_LH_THIGH;
        Vector3 com_LH_SHANK;
        Vector3 com_RH_HIP;
        Vector3 com_RH_THIGH;
        Vector3 com_RH_SHANK;
};


inline InertiaProperties::~InertiaProperties() {}

inline const InertiaMatrix& InertiaProperties::getTensor_BASE() const {
    return this->tensor_BASE;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LF_HIP() const {
    return this->tensor_LF_HIP;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LF_THIGH() const {
    return this->tensor_LF_THIGH;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LF_SHANK() const {
    return this->tensor_LF_SHANK;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RF_HIP() const {
    return this->tensor_RF_HIP;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RF_THIGH() const {
    return this->tensor_RF_THIGH;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RF_SHANK() const {
    return this->tensor_RF_SHANK;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LH_HIP() const {
    return this->tensor_LH_HIP;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LH_THIGH() const {
    return this->tensor_LH_THIGH;
}
inline const InertiaMatrix& InertiaProperties::getTensor_LH_SHANK() const {
    return this->tensor_LH_SHANK;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RH_HIP() const {
    return this->tensor_RH_HIP;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RH_THIGH() const {
    return this->tensor_RH_THIGH;
}
inline const InertiaMatrix& InertiaProperties::getTensor_RH_SHANK() const {
    return this->tensor_RH_SHANK;
}
inline Scalar InertiaProperties::getMass_BASE() const {
    return this->tensor_BASE.getMass();
}
inline Scalar InertiaProperties::getMass_LF_HIP() const {
    return this->tensor_LF_HIP.getMass();
}
inline Scalar InertiaProperties::getMass_LF_THIGH() const {
    return this->tensor_LF_THIGH.getMass();
}
inline Scalar InertiaProperties::getMass_LF_SHANK() const {
    return this->tensor_LF_SHANK.getMass();
}
inline Scalar InertiaProperties::getMass_RF_HIP() const {
    return this->tensor_RF_HIP.getMass();
}
inline Scalar InertiaProperties::getMass_RF_THIGH() const {
    return this->tensor_RF_THIGH.getMass();
}
inline Scalar InertiaProperties::getMass_RF_SHANK() const {
    return this->tensor_RF_SHANK.getMass();
}
inline Scalar InertiaProperties::getMass_LH_HIP() const {
    return this->tensor_LH_HIP.getMass();
}
inline Scalar InertiaProperties::getMass_LH_THIGH() const {
    return this->tensor_LH_THIGH.getMass();
}
inline Scalar InertiaProperties::getMass_LH_SHANK() const {
    return this->tensor_LH_SHANK.getMass();
}
inline Scalar InertiaProperties::getMass_RH_HIP() const {
    return this->tensor_RH_HIP.getMass();
}
inline Scalar InertiaProperties::getMass_RH_THIGH() const {
    return this->tensor_RH_THIGH.getMass();
}
inline Scalar InertiaProperties::getMass_RH_SHANK() const {
    return this->tensor_RH_SHANK.getMass();
}
inline const Vector3& InertiaProperties::getCOM_BASE() const {
    return this->com_BASE;
}
inline const Vector3& InertiaProperties::getCOM_LF_HIP() const {
    return this->com_LF_HIP;
}
inline const Vector3& InertiaProperties::getCOM_LF_THIGH() const {
    return this->com_LF_THIGH;
}
inline const Vector3& InertiaProperties::getCOM_LF_SHANK() const {
    return this->com_LF_SHANK;
}
inline const Vector3& InertiaProperties::getCOM_RF_HIP() const {
    return this->com_RF_HIP;
}
inline const Vector3& InertiaProperties::getCOM_RF_THIGH() const {
    return this->com_RF_THIGH;
}
inline const Vector3& InertiaProperties::getCOM_RF_SHANK() const {
    return this->com_RF_SHANK;
}
inline const Vector3& InertiaProperties::getCOM_LH_HIP() const {
    return this->com_LH_HIP;
}
inline const Vector3& InertiaProperties::getCOM_LH_THIGH() const {
    return this->com_LH_THIGH;
}
inline const Vector3& InertiaProperties::getCOM_LH_SHANK() const {
    return this->com_LH_SHANK;
}
inline const Vector3& InertiaProperties::getCOM_RH_HIP() const {
    return this->com_RH_HIP;
}
inline const Vector3& InertiaProperties::getCOM_RH_THIGH() const {
    return this->com_RH_THIGH;
}
inline const Vector3& InertiaProperties::getCOM_RH_SHANK() const {
    return this->com_RH_SHANK;
}

inline Scalar InertiaProperties::getTotalMass() const {
    return m_BASE + m_LF_HIP + m_LF_THIGH + m_LF_SHANK + m_RF_HIP + m_RF_THIGH + m_RF_SHANK + m_LH_HIP + m_LH_THIGH + m_LH_SHANK + m_RH_HIP + m_RH_THIGH + m_RH_SHANK;
}

}
}

#endif
