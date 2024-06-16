// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#include "buff_calculate.h"

#include <stdint.h>
#include <stdio.h>

#include "math.h"
#define PI 3.1415926

void calcuateBuffPostition(float xc, float yc, float zc, float theta,
                           int blade_id, float a, float b, float w,
                           uint64_t cap_timestamp, uint16_t t_offset,
                           float* target_x, float* target_y, float* target_z) {
  // TODO: obtain self timestamp and set delay
  uint64_t self_timestamp = 1234502;  // ms stamp
  float delay = 0.3;                  // s

  //   TODO end

  float r = 0.7;

  float t0 = (float)(cap_timestamp + t_offset) / 1000;
  float t1 = (float)(self_timestamp+ t_offset) / 1000 + delay;
  if (w == 0) {
    theta += b * (t1 - t0);
  } else {
    theta += a / w * (cos(w * t0) - cos(w * t1)) + b * (t1 - t0);
  }
  *target_x = xc + r * (sin(theta) * yc / sqrt(pow(xc, 2) + pow(yc, 2)));
  *target_y = yc + r * (-sin(theta) * xc / sqrt(pow(xc, 2) + pow(yc, 2)));
  *target_z = zc + r * cos(theta);
}

int main() {
  // serial param propotion
  uint8_t id = 2;
  float x = 0.6626;
  float y = 0.0001;
  float z = 0.1245;
  float yaw = 1.57075;
  float spd_a = 0.0;
  float spd_b = 1.04717;
  float spd_w = 0.0;
  uint64_t cap_timestamp = 1234500;
  uint16_t t_offset = 1233;
  // propotion end

  float target_x;
  float target_y;
  float target_z;

  calcuateBuffPostition(x, y, z, yaw, (int)id, spd_a, spd_b, spd_w,
                        cap_timestamp, t_offset, &target_x, &target_y,
                        &target_z);

  printf("x: %f\ny: %f\ny: %f\n", target_x, target_y, target_z);
  return 0;
}