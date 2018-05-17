#include "detectlane.hpp"

DetectLane::DetectLane() noexcept :
  cv_image()
  , m_currentImg()
  , m_blurKernelSize(3)
  , m_cannyImg()
  , m_adapThreshImg()
  , m_visualMemory()
// m_intensityThreshold = 140
  , m_adapThreshKernelSize(5)
  , m_adapThreshConst(25)
  , m_cannyThreshold(180) //220
  , m_houghThreshold(80) //#80,100
  , m_linesRaw()
  , m_linesProcessed()
  , m_lineDiff(2.1f) //1.6
  , m_OneLineDiff(5.0f) //#how much a line can differ between the two points
  , m_HorisontalLimit(12.0f) // meters on each side we should consider
  , m_memThreshold(0.5) //seconds
//uint16_t m_upperLaneLimit = 250;
//uint16_t m_lowerLaneLimit = 450;
//m_screenWidth = 1280
//m_screenHeight = 720
// m_debug = 1
// m_camera = name
  , m_roi{}
  

    /*
	// JL: odcore from OpenDaVinci -> concurrency
  boost::property_tree::ptree pt;
  boost::property_tree::ini_parser::read_ini("cv_config.ini", pt);
  m_adapThreshKernelSize = 
      pt.getValue<uint16_t>("m_adapThreshKernelSize");
  m_adapThreshConst = 
      pt.getValue<uint16_t>("m_adapThreshConst");
  m_cannyThreshold = 
      pt.getValue<uint16_t>("m_cannyThreshold");
  m_houghThreshold = 
      pt.getValue<uint16_t>("m_houghThreshold");
  m_lineDiff = pt.getValue<float>("m_lineDiff");
  m_OneLineDiff = pt.getValue<float>("m_OneLineDiff");
  m_HorisontalLimit = 
      pt.getValue<float>("m_HorisontalLimit");
  m_blurKernelSize = pt.getValue<uint16_t>("m_blurKernelSize");
  m_memThreshold = pt.getValue<double>("m_memThreshold");
  m_upperLaneLimit = 
      pt.getValue<uint16_t>("m_upperLaneLimit");
  m_lowerLaneLimit = 
      pt.getValue<uint16_t>("m_lowerLaneLimit");
  m_screenSize[0] = pt.getValue<uint16_t>("m_screenWidth");
  m_screenSize[1] = pt.getValue<uint16_t>("m_screenHeight");
  m_roi[0] = pt.getValue<uint16_t>("m_roiX");
  m_roi[1] = pt.getValue<uint16_t>("m_roiY");
  m_roi[2] = pt.getValue<uint16_t>("m_roiWidth");
  m_roi[3] = pt.getValue<uint16_t>("m_roiHeight");
  m_debug = (pt.getValue<int32_t>("m_debug") == 1);
  m_cameraName = pt.getValue<std::string>("m_camera");
  std::string const projectionFilename = m_cameraName + "-pixel2world-matrix.csv";
  m_transformationMatrix = ReadMatrix(projectionFilename,3,3);
  */
{
  //m_roi={30,150,1000,50}; //#205 Pixels away from the upper left corner in X
  m_roi[0]=30;
  m_roi[1]=150; //#200 Pixels away from the upper part of picture in Y
  m_roi[2]=1000; //#110 Done Pixel width of the captured box in X
  m_roi[3]=50; //#300 Pixel height of the captured box in Y
}


DetectLane::~DetectLane()
{
  cv_image.release();
}

// but in the constructure
//void DetectLane::setUp(){}

void DetectLane::Datatrigger(cv::Mat image) {
  m_currentImg = image.clone();
  UpdateVisualMemory();
  UpdateVisualLines();
}
  
void DetectLane::UpdateVisualMemory() {
  cluon::data::TimeStamp  now;
  cv::Rect rectROI(m_roi[0], m_roi[1], m_roi[2], m_roi[3]);
  cv::Mat visualImpression;
  try {
    //odcore::base::Lock l(m_mtx);
    visualImpression = m_currentImg(rectROI).clone();
  } catch (cv::Exception& e) {
    std::cerr << "Error cropping the image due to dimension size. " << std::endl;
    return;
  }
  // Reduce the noise in image
  cv::medianBlur(visualImpression, visualImpression, m_blurKernelSize);
  
  m_visualMemory.push_back(std::make_pair(now, visualImpression));
  // Delete old mem, comparison in microseconds in the timestamps
  int64_t const MEMCAP_IN_MICROSECONDS = static_cast<int64_t>(m_memThreshold*1000000.0);
  bool memoryIsTooOld = deltaInMicroseconds(now, m_visualMemory.front().first) > MEMCAP_IN_MICROSECONDS;
  while (!m_visualMemory.empty() && memoryIsTooOld) {
    m_visualMemory.pop_front();
  //  memoryIsTooOld = (now - m_visualMemory.front().first).toMicroseconds() > MEMCAP_IN_MICROSECONDS;
	  
  } 
}

void DetectLane::UpdateVisualLines()
{
  // Canny transformation 
  cv::Canny(m_visualMemory.back().second, m_cannyImg, m_cannyThreshold, m_cannyThreshold*3, 3);
  cv::Sobel(m_cannyImg, m_cannyImg, m_cannyImg.depth(), 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);

  // Adaptive Threshold
  cv::cvtColor(m_visualMemory.back().second, m_adapThreshImg, CV_RGB2GRAY);
  cv::adaptiveThreshold(m_adapThreshImg,m_adapThreshImg,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, m_adapThreshKernelSize, m_adapThreshConst);
  cv::Sobel(m_adapThreshImg, m_adapThreshImg, m_adapThreshImg.depth(), 1, 0, 3, 1, 0, cv::BORDER_DEFAULT); 

  // Vector holder for each line (rho,theta)
  std::vector<cv::Vec2f> detectedLinesCanny;
  std::vector<cv::Vec2f> detectedLinesAdapThresh;

  // Hough Transform 
  // OpenCV function that uses the Hough transform and finds the "strongest" lines in the transformation    
  cv::HoughLines(m_cannyImg, detectedLinesCanny, 1, opendlv::Constants::PI/180.0, m_houghThreshold);
  cv::HoughLines(m_adapThreshImg, detectedLinesAdapThresh, 1, opendlv::Constants::PI/180.0, m_houghThreshold);
  m_linesRaw = detectedLinesCanny;
  m_linesRaw.insert(m_linesRaw.end(), detectedLinesAdapThresh.begin(), detectedLinesAdapThresh.end());
  double const DIST_RADIUS = 100.0;
  m_linesProcessed = GetGrouping(m_linesRaw, DIST_RADIUS);
}

