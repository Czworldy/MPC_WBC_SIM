#ifndef CPPTYPES_H
#define CPPTYPES_H

#include<vector>
#include<eigen3/Eigen/Dense>

using namespace std;

// Dynamically sized vector
template <typename T>
using DVec = typename Eigen::Matrix<T, Eigen::Dynamic, 1>;

// Dynamically sized matrix
template <typename T>
using DMat = typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

// 3x1 Vector
template <typename T>
using Vec31 = typename Eigen::Matrix<T, 3, 1>;

// 8x1 Vector
template <typename T>
using Vec8 = typename Eigen::Matrix<T, 8, 1>;

// 4x3 Matrix
template <typename T>
using Mat43 = typename Eigen::Matrix<T, 4, 3>;

// 4x1 Vector
template <typename T>
using Quat = typename Eigen::Matrix<T, 4, 1>;

// Spatial Vector (6x1, all subspaces)
template <typename T>
using SVec = typename Eigen::Matrix<T, 6, 1>;

// 4x1 Vector
template <typename T>
using Vec41 = typename Eigen::Matrix<T, 4, 1>;

// 2x1 Vector
template <typename T>
using Vec2 = typename Eigen::Matrix<T, 2, 1>;

// 12x1 Vector
template <typename T>
using Vec12 = typename Eigen::Matrix<T, 12, 1>;

// 6x1 Vector
template <typename T>
using Vec6 = typename Eigen::Matrix<T, 6, 1>;

// 5x1 Vector
template <typename T>
using Vec5 = typename Eigen::Matrix<T, 5, 1>;

// Rotation Matrix
template <typename T>
using RotMat = typename Eigen::Matrix<T, 3, 3>;

// 3x3 Matrix
template <typename T>
using Mat3 = typename Eigen::Matrix<T, 3, 3>;

// 3x6 Matrix
template <typename T>
using Mat36 = typename Eigen::Matrix<T, 3, 6>;

// 6x12 Matrix
template <typename T>
using Mat6_12 = typename Eigen::Matrix<T, 6, 12>;

// 2x12 Matrix
template <typename T>
using Mat2_12 = typename Eigen::Matrix<T, 2, 12>;

// 2x2 Matrix
template <typename T>
using Mat2 = typename Eigen::Matrix<T, 2, 2>;

// std::vector (a list) of Eigen things
template <typename T>
using vectorAligned = typename std::vector<T, Eigen::aligned_allocator<T>>;

// 6x6 Matrix
template <typename T>
using Mat6 = typename Eigen::Matrix<T, 6, 6>;

// 1x6 Matrix
template <typename T>
using Mat16 = typename Eigen::Matrix<T, 1, 6>;

// 4x4 Matrix
template <typename T>
using Mat4 = typename Eigen::Matrix<T, 4, 4>;

// 1x3 Matrix
template <typename T>
using Mat13 = typename Eigen::Matrix<T, 1, 3>;


/** size_t trajectory type. */
using size_array_t = std::vector<size_t>;
/** Array of size_t trajectory type. */
using size_array2_t = std::vector<size_array_t>;

/** Scalar type. */
using scalar_t = float;
/** Scalar trajectory type. */
using scalar_array_t = std::vector<scalar_t>;
/** Array of scalar trajectory type. */
using scalar_array2_t = std::vector<scalar_array_t>;
/** Array of arrays of scalar trajectory type. */
using scalar_array3_t = std::vector<scalar_array2_t>;

/** Dynamic-size vector type. */
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
/** Dynamic vector's trajectory type. */
using vector_array_t = std::vector<vector_t>;
/** Array of dynamic vector's trajectory type. */
using vector_array2_t = std::vector<vector_array_t>;
/** Array of arrays of dynamic vector trajectory type. */
using vector_array3_t = std::vector<vector_array2_t>;

/** Dynamic-size row vector type. */
using row_vector_t = Eigen::Matrix<scalar_t, 1, Eigen::Dynamic>;

/** Dynamic-size matrix type. */
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
/** Dynamic matrix's trajectory type. */
using matrix_array_t = std::vector<matrix_t>;
/** Array of dynamic matrix's trajectory type. */
using matrix_array2_t = std::vector<matrix_array_t>;
/** Array of arrays of dynamic matrix trajectory type. */
using matrix_array3_t = std::vector<matrix_array2_t>;

/** Eigen scalar type. */
using eigen_scalar_t = Eigen::Matrix<scalar_t, 1, 1>;
/** Eigen scalar trajectory type. */
using eigen_scalar_array_t = std::vector<eigen_scalar_t>;
/** Array of eigen scalar trajectory type. */
using eigen_scalar_array2_t = std::vector<eigen_scalar_array_t>;
/** Array of arrays of eigen scalar trajectory type. */
using eigen_scalar_array3_t = std::vector<eigen_scalar_array2_t>;

using vector_foot_t = std::vector< Eigen::Matrix< Eigen::Matrix<float, 3, 1 >, 4, 1> >;


#endif