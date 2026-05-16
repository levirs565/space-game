#ifndef SPACE_MAT3_HPP
#define SPACE_MAT3_HPP

#include "Vec2.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

struct Mat3 {
  std::array<std::array<double, 3>, 3> values;

  Mat3() = default;
  Mat3(std::array<std::array<double, 3>, 3> &&values) : values(values) {}
  Mat3(double a, double b, double c, //
       double d, double e, double f, //
       double g, double h, double i)
      : values({{a, b, c}, {d, e, f}, {g, h, i}}) {}

  const std::array<double, 3> &operator[](size_t index) const {
    return values[index];
  }

  std::array<double, 3> &operator[](size_t index) { return values[index]; }

  Mat3 operator*(const Mat3 &b) const {
    Mat3 result{};
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++) {
          result[i][j] += (*this)[i][k] * b[k][j];
        }
      }
    }
    return result;
  }

  Vec3 operator*(const Vec3 &b) const {
    return {
        (*this)[0][0] * b.x + (*this)[0][1] * b.y + (*this)[0][2] * b.z,
        (*this)[1][0] * b.x + (*this)[1][1] * b.y + (*this)[1][2] * b.z,
        (*this)[2][0] * b.x + (*this)[2][1] * b.y + (*this)[2][2] * b.z,
    };
  }

  Vec3 operator*(const Vec2 &b) const { return (*this) * b.toHomogenous(); }

  [[nodiscard]] Vec2 getScale() const {
    return {std::sqrt((*this)[0][0] * (*this)[0][0] +
                      (*this)[1][0] * (*this)[1][0]),
            std::sqrt((*this)[0][1] * (*this)[0][1] +
                      (*this)[1][1] * (*this)[1][1])};
  }

  [[nodiscard]] Mat3 affineInverse() const {
    double a = (*this)[0][0];
    double b = (*this)[0][1];
    double c = (*this)[1][0];
    double d = (*this)[1][1];
    double tx = (*this)[0][2];
    double ty = (*this)[1][2];

    double det = a * d - b * c;

    if (std::abs(det) < 1e-7) {
      throw std::invalid_argument("Inverse not found");
    }

    double invDet = 1.0 / det;

    double ia =  d * invDet;
    double ib = -b * invDet;
    double ic = -c * invDet;
    double id =  a * invDet;

    // clang-format off
    return {
      ia, ib, -(ia*tx + ib*ty),
      ic, id, -(ic*tx + id*ty),
      0, 0, 1
    };
    // clang-format on
  }

  static Mat3 identity() {
    // clang-format off
    return {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
    // clang-format on
  }

  static Mat3 translation(double x, double y) {
    // clang-format off
    return {
        1.0, 0.0, x,
        0.0, 1.0, y,
        0.0, 0.0, 1.0
    };
    // clang-format on
  }

  static Mat3 translation(const Vec2 &amount) {
    return translation(amount.x, amount.y);
  }

  static Mat3 scale(double x, double y) {
    // clang-format off
    return {
        x,   0.0, 0.0,
        0.0, y,   0.0,
        0.0, 0.0, 1.0
    };
    // clang-format on
  }

  static Mat3 scale(double s) { return scale(s, s); }

  static Mat3 rotation(double radian) {
    // clang-format off
    return {
      std::cos(radian), -std::sin(radian), 0.0,
      std::sin(radian), std::cos(radian), 0.0,
      0.0, 0.0, 1.0
    };
    // clang-format on
  }

  static Mat3 rotationAround(double radian, const Vec2 &around) {
    return translation(around) * rotation(radian) *
           translation(Vec2(0, 0) - around);
  }
};

#endif // SPACE_MAT3_HPP
