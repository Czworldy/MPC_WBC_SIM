#pragma once

#include <array>
#include <cstddef>

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace legged_robot {

template <typename T>
using feet_array_t = std::array<T, 4>;
template <typename T>
using feet_arm_array_t = std::array<T, 5>;
using contact_flag_t = feet_arm_array_t<bool>;

using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using vector6_t = Eigen::Matrix<scalar_t, 6, 1>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using quaternion_t = Eigen::Quaternion<scalar_t>;

} // namespace legged_robot
} // namespace ocs2