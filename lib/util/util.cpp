#include "util.h"

//map from an input to output range linearly
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  //if (x <= in_min) return out_min;
  //if (x >= in_max) return out_max;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}