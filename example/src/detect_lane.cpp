#include "detect_lane.hpp"

DetectLane::DetectLane() noexcept:
 UpdateVisualMemory {}
{
}

void DetectLane::UpdateVisualMemory() {
  odcore::data::TimeStamp now;
  cv::Rect rectROI(m_roi[0], m_roi[1], m_roi[2], m_roi[3]);
  cv::Mat visualImpression;
  try {
    odcore::base::Lock l(m_mtx);
    visualImpression = m_currentImg(rectROI).clone();
  } catch (cv::Exception& e) {
    std::cerr << "[" << getName() << "] " << "Error cropping the image due to dimension size. " << std::endl;
    return;
  }
	  
  // Reduce the noise in image
  cv::medianBlur(visualImpression, visualImpression, m_blurKernelSize);
  
  m_visualMemory.push_back(std::make_pair(now, visualImpression));
  // Delete old mem, comparison in microseconds in the timestamps
  int64_t const MEMCAP_IN_MICROSECONDS = static_cast<int64_t>(m_memThreshold*1000000.0);
  bool memoryIsTooOld = (now - m_visualMemory.front().first).toMicroseconds() > MEMCAP_IN_MICROSECONDS;
  while (!m_visualMemory.empty() && memoryIsTooOld) {
    m_visualMemory.pop_front();
    memoryIsTooOld = (now - m_visualMemory.front().first).toMicroseconds() > MEMCAP_IN_MICROSECONDS;
  }
}
