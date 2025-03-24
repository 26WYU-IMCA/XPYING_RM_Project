#ifndef PNP_SOLVER_H
#define PNP_SOLVER_H

#include "Armor_match.h"
// std
#include <array>

class PnPSolver
{
public:
  PnPSolver();

  // Get 3d position
  bool solvePnP(const armorDetector::Armor &armor, cv::Mat &rvec, cv::Mat &tvec);

  // Calculate the distance between armor center and image center
  float calculateDistanceToCenter(const cv::Point2f &image_point);

  // 相机参数
  cv::Mat camera_matrix_ = (cv::Mat_<double>(3, 3) << 1034.174789, 0.000000, 638.103833,
                                                      0.000000, 1034.712789, 494.576371,
                                                      0.000000, 0.000000, 1.000000);
  cv::Mat dist_coeffs_ = (cv::Mat_<double>(1, 5) << -0.100831, 0.094629, -0.000572, 0.000629, 0.000000);

  // Four vertices of armor in 3d
  std::vector<cv::Point3f> small_armor_points_;

private:
  // Unit: mm
  static constexpr float SMALL_ARMOR_WIDTH_ = 230;
  static constexpr float SMALL_ARMOR_HEIGHT_ = 127;

  // 相机参数获取
};

#endif