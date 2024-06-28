#ifndef UTILS_HPP
#define UTILS_HPP

#define g 9.81
#define PI 3.14159265359

namespace utils {
  /**
   * @brief Converts degrees to radians
   * 
   * @param deg degrees. CCW is +ve
   * @return double radians, CCW is +ve
   */
  double deg2rad(double deg);

  /**
   * @brief Converts radians to degrees
   * 
   * @param rad radians, CCW is +ve
   * @return double degrees, CCW is +ve
   */
  double rad2deg(double rad);
}

#endif