#include "Interpolators.h"


void lin_interp(const Eigen::VectorXd& xData,
    const Eigen::VectorXd& yData,
    const double xQuery,
    double& yOut,
    double& slopeOut)
{
  // just some random values so that in case something goes wrong, the slope is not undefined.
  double yJPlusOne = 1;
  double yJ = 0;
  double xJPlusOne = 1;
  double xJ = 0;
  
  Eigen::Index jPlusOne = 0;
  
  // If the inquired x is the first element or on the left hand side of the first element of the xData
  if (xQuery <= xData[0]) {
    xJ = xData[0];
    xJPlusOne = xData[1];
    
    yJ = yData[0];
    yJPlusOne = yData[1];
    
    yOut = - (yJPlusOne - yJ) / (xJPlusOne - xJ) * (xJ - xQuery) + yJ; // linearly extrapolate
    
  }
  else if (xQuery >= xData[xData.size() - 1]
      ) // If the inquired x is the last element or on the right hand side of the last element of the xData
  {
    xJ = xData[xData.size() - 1 - 1];
    xJPlusOne = xData[xData.size() - 1];
    
    yJ = yData[xData.size() - 1 - 1];
    yJPlusOne = yData[xData.size() - 1];
    
    yOut = (yJPlusOne - yJ) / (xJPlusOne - xJ) * (xQuery - xJPlusOne) + yJPlusOne; // linearly extrapolate
  }
  else {
    
    for (Eigen::Index j = 0; j < xData.size() - static_cast<long long>(1); j ++) {
      
      jPlusOne = j + 1;
      
      if (xData[j] <= xQuery && xQuery < xData[jPlusOne]) {
        
        xJ = xData[j];
        xJPlusOne = xData[jPlusOne];
        
        yJ = yData[j];
        yJPlusOne = yData[jPlusOne];
        
        yOut = (yJPlusOne - yJ) / (xJPlusOne - xJ) * (xQuery - xJ) + yJ; // linearly interpolate
        
      }
    }
  }
  
  slopeOut = (yJPlusOne - yJ) / (xJPlusOne - xJ);
}

// If another types of interpolation is used, define the function here.

void Interpolators::interpolate(const Eigen::VectorXd& xData,
    const Eigen::VectorXd& yData,
    const double xQuery,
    double& yOut,
    double& slopeOut,
    const InterpolationScheme interpolationScheme)
{
  switch (interpolationScheme) {
  
  case InterpolationScheme::linear:
    lin_interp(xData, yData, xQuery, yOut, slopeOut);
    break;
    
  }
}

void Interpolators::interpolate(const Eigen::VectorXd& xData,
    const Eigen::VectorXd& yData,
    const double xQuery,
    double& yOut,
    const InterpolationScheme interpolationScheme)
{
  double dummySlp{};
  switch (interpolationScheme) {
  
  case InterpolationScheme::linear:
    lin_interp(xData, yData, xQuery, yOut, dummySlp);
    break;
    
  }
}


