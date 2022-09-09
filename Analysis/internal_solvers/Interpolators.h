#pragma once

#include "Eigen/Dense"

enum class InterpolationScheme {
  linear
};

class Interpolators {
public:
  static void interpolate(const Eigen::VectorXd& xData,
      const Eigen::VectorXd& yData,
      double xQuery,
      double& yOut,
      double& slopeOut,
      InterpolationScheme interpolationScheme);
  
  static void interpolate(const Eigen::VectorXd& xData,
      const Eigen::VectorXd& yData,
      double xQuery,
      double& yOut,
      InterpolationScheme interpolationScheme);
};

