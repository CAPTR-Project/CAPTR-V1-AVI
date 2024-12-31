#include "utils.hpp"

double utils::deg2rad(double deg) {
  return deg * PI / 180.0;
}

double utils::rad2deg(double rad) {
  return rad * 180.0 / PI;
}

