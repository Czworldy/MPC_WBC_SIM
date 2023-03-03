// #pragma once

// #include<vector>
// #include<eigen3/Eigen/Dense>

// using namespace std;
// namespace ocs2{
// // Dynamically sized vector
// template <typename T>
// using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

// // Dynamically sized matrix
// template <typename T>
// using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

// // 3x1 Vector
// template <typename T>
// using Vec31 = typename Eigen::Matrix<T, 3, 1>;

// // 8x1 Vector
// template <typename T>
// using Vec8 = typename Eigen::Matrix<T, 8, 1>;

// // 4x3 Matrix
// template <typename T>
// using Mat43 = typename Eigen::Matrix<T, 4, 3>;

// // 4x1 Vector
// template <typename T>
// using Quat = typename Eigen::Matrix<T, 4, 1>;

// // Spatial Vector (6x1, all subspaces)
// template <typename T>
// using SVec = typename Eigen::Matrix<T, 6, 1>;

// // 4x1 Vector
// template <typename T>
// using Vec41 = typename Eigen::Matrix<T, 4, 1>;

// // 2x1 Vector
// template <typename T>
// using Vec2 = typename Eigen::Matrix<T, 2, 1>;

// // 12x1 Vector
// template <typename T>
// using Vec12 = typename Eigen::Matrix<T, 12, 1>;

// // 6x1 Vector
// template <typename T>
// using Vec6 = typename Eigen::Matrix<T, 6, 1>;

// // 5x1 Vector
// template <typename T>
// using Vec5 = typename Eigen::Matrix<T, 5, 1>;

// // Rotation Matrix
// template <typename T>
// using RotMat = typename Eigen::Matrix<T, 3, 3>;

// // 3x3 Matrix
// template <typename T>
// using Mat3 = typename Eigen::Matrix<T, 3, 3>;

// // 3x6 Matrix
// template <typename T>
// using Mat36 = typename Eigen::Matrix<T, 3, 6>;

// // 6x12 Matrix
// template <typename T>
// using Mat6_12 = typename Eigen::Matrix<T, 6, 12>;

// // 2x12 Matrix
// template <typename T>
// using Mat2_12 = typename Eigen::Matrix<T, 2, 12>;

// // 2x2 Matrix
// template <typename T>
// using Mat2 = typename Eigen::Matrix<T, 2, 2>;

// // std::vector (a list) of Eigen things
// template <typename T>
// using vectorAligned = typename std::vector<T, Eigen::aligned_allocator<T>>;

// // 6x6 Matrix
// template <typename T>
// using Mat6 = typename Eigen::Matrix<T, 6, 6>;

// // 1x6 Matrix
// template <typename T>
// using Mat16 = typename Eigen::Matrix<T, 1, 6>;

// // 4x4 Matrix
// template <typename T>
// using Mat4 = typename Eigen::Matrix<T, 4, 4>;

// // 1x3 Matrix
// template <typename T>
// using Mat13 = typename Eigen::Matrix<T, 1, 3>;
// }