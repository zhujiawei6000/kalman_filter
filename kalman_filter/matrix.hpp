#pragma once
#include <Eigen/Dense>

#define KF_VECTOR(NAME, N)                                                     \
  typedef kf::Vector<N> Base;                                                  \
  using typename Base::Scalar;                                                 \
  using Base::RowsAtCompileTime;                                               \
  using Base::ColsAtCompileTime;                                               \
  using Base::SizeAtCompileTime;                                               \
                                                                               \
  NAME(void) : kf::Vector<N>() {}                                              \
                                                                               \
  template <typename OtherDerived>                                             \
  NAME(const Eigen::MatrixBase<OtherDerived>& other) : kf::Vector<N>(other) {} \
                                                                               \
  template <typename OtherDerived>                                             \
  NAME& operator=(const Eigen::MatrixBase<OtherDerived>& other) {              \
    this->Base::operator=(other);                                              \
    return *this;                                                              \
  }

namespace kf {

template <size_t Row, size_t Col>
using Matrix = Eigen::Matrix<double, Row, Col>;

/**
 * @brief Template type for vectors
 * @param T The numeric scalar type
 * @param N The vector dimension
 */
template <size_t N>
class Vector : public kf::Matrix<N, 1> {
 public:
  //! Matrix base type
  typedef kf::Matrix<N, 1> Base;

  using Base::ColsAtCompileTime;
  using Base::RowsAtCompileTime;
  using Base::SizeAtCompileTime;
  using typename Base::Scalar;

  Vector(void) : kf::Matrix<N, 1>() {}

  /**
   * @brief Copy constructor
   */
  template <typename OtherDerived>
  Vector(const Eigen::MatrixBase<OtherDerived>& other) : kf::Matrix<N, 1>(other) {}
  /**
   * @brief Copy assignment constructor
   */
  template <typename OtherDerived>
  Vector& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
    this->Base::operator=(other);
    return *this;
  }
};

}  // namespace kf