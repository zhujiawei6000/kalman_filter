#pragma once
#include <Eigen/Dense>

namespace kf {
template<typename Vector, size_t NumCol>
auto HStack(std::array<Vector, NumCol> cols) {
    constexpr size_t NumRow = Vector::RowsAtCompileTime;
    Matrix<NumRow, NumCol> result;
    for (size_t i = 0; i < NumCol; ++i) {
        result.col(i) = cols[i];
    }
    return result;
}
}