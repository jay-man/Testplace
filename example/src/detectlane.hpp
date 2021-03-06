/**
 * Copyright (C) 2017 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#ifndef LOGIC_PERCEPTION_DETECTLANE
#define LOGIC_PERCEPTION_DETECTLANE

#include <deque>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>
#include <cmath>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "opendlv-standard-message-set.hpp"
#include "cluon-complete.hpp" 


class DetectLane {
 private:
  DetectLane(DetectLane const &) = delete;
  DetectLane(DetectLane &&) = delete;
  DetectLane &operator=(DetectLane const &) = delete;
  DetectLane &operator=(DetectLane &&) = delete;

 public:
  DetectLane() noexcept;
  virtual ~DetectLane();
 public:
  void Datatrigger(cv::Mat, uint32_t, uint32_t, uint16_t, uint8_t, uint8_t, uint16_t, uint16_t, float, float, float, double, double, double, uint16_t, uint16_t, uint16_t, uint16_t, bool);	
  void setUp();
  void UpdateVisualMemory();
  void UpdateVisualLines();
  std::vector<cv::Vec2f> GetGrouping(std::vector<cv::Vec2f>, double);
  std::vector<std::pair<cv::Vec2f, cv::Vec2f>> GetParametricRepresentation(std::vector<cv::Vec2f>);
  void UpdatePointsOnLines(std::vector<std::pair<cv::Vec2f, cv::Vec2f>>);
  std::vector<uint16_t> GetLanes() const;
  std::vector<uint16_t> GetCurrentLane() const;
  Eigen::Vector3d TransformPointToGlobalFrame(Eigen::Vector3d) const;
  Eigen::MatrixXd ReadMatrix(std::string const, uint8_t const, uint8_t const) const;
  void DrawWindows();

  cv::Mat cv_image;
  cv::Mat m_currentImg;
  uint16_t m_blurKernelSize; 
  cv::Mat m_cannyImg;
  cv::Mat m_adapThreshImg;
  std::deque<std::pair<cluon::data::TimeStamp, cv::Mat>> m_visualMemory;
  uint8_t m_adapThreshKernelSize;
  uint8_t m_adapThreshConst;
  uint16_t m_cannyThreshold;
  uint16_t m_houghThreshold;
  std::vector<cv::Vec2f> m_linesRaw;
  std::vector<cv::Vec2f> m_linesProcessed;
  std::vector<uint16_t> m_laneLineIds;
  std::vector<uint16_t> m_currentLaneLineIds;
  std::vector<cv::Vec2f> m_xScreenP;
  std::vector<cv::Vec2f> m_yScreenP;
  std::vector<cv::Vec2f> m_xWorldP;
  std::vector<cv::Vec2f> m_yWorldP;
  float m_lineDiff;
  float m_OneLineDiff;
  float m_HorisontalLimit;
  double m_memThreshold;
  double m_upperLaneLimit;
  double m_lowerLaneLimit;
  int16_t m_screenSize[2];
  int16_t m_roi[4];
  std::mutex m_mtx;
  bool m_debug;
  Eigen::Matrix3d m_transformationMatrix;
};


#endif
