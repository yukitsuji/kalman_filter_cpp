#ifndef TRAKING_H_
#define TRAKING_H_

#include "measurement_package.h"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"

class Tracking
{
public:
  Tracking();
  virtual ~Tracking();
  void processMeasurement(const MeasurementPackage &measurement_pack);
  KalmanFilter kf_;

private:
  bool is_initialized_;
  long previous_timestamp_;

  float noise_ax;
  float noise_ay;
};

#endif /* TRACKING_H_ */
