#include "detectlane.hpp"

DetectLane::DetectLane() noexcept :
  cv_image()
  , m_currentImg()
  , m_blurKernelSize() // 3
  , m_cannyImg()
  , m_adapThreshImg()
  , m_visualMemory()
  , m_adapThreshKernelSize() //5
  , m_adapThreshConst() //3
  , m_cannyThreshold() //current: 30, 220
  , m_houghThreshold() //#80,100
  , m_linesRaw()
  , m_linesProcessed()
  , m_laneLineIds()
  , m_currentLaneLineIds()
  , m_xScreenP()
  , m_yScreenP()
  , m_xWorldP()
  , m_yWorldP()
  , m_lineDiff() //current 2.1f, 1.6
  , m_OneLineDiff() //current 5.0f how much a line can differ between the two points
  , m_HorisontalLimit() // current 12.0f meters on each side we should consider
  , m_memThreshold() //current 0.5 seconds
  , m_upperLaneLimit() //current 200
  , m_lowerLaneLimit() //current 500
  , m_screenSize{}
  , m_roi{}
  , m_mtx()
  , m_debug(1) //1
  , m_transformationMatrix()
{

  //m_roi={30,150,1000,50}; 
  //m_roi[0]=200; //#205 Pixels away from the upper left corner in X
  //m_roi[1]=170; //#200 Pixels away from the upper part of picture in Y
  //m_roi[2]=800; //#110 Done Pixel width of the captured box in X
  //m_roi[3]=400; //#300 Pixel height of the captured box in Y

  //m_screenSize[0]=1280;
  //m_screenSize[1]=720;
	  
  m_transformationMatrix = ReadMatrix("logic-perception-detectlane.camera-pixel2world-matrix.csv",3,3);	
}


DetectLane::~DetectLane()
{
  cv_image.release();
}

void DetectLane::Datatrigger(cv::Mat image, uint32_t width, uint32_t height, uint16_t blurKernelSize, uint8_t adapThreshKernelSize,
							 uint8_t adapThreshConst, uint16_t cannyThreshold, uint16_t houghThreshold, float lineDiff, float OneLineDiff,
							 float HorisontalLimit, double memThreshold, double upperLaneLimit, double lowerLaneLimit, uint16_t roiX,
							 uint16_t roiY, uint16_t roiWidth, uint16_t roiHeight, bool debug) {

  m_screenSize[0]=width;
  m_screenSize[1]=height;
  m_blurKernelSize=blurKernelSize;
  m_adapThreshKernelSize=adapThreshKernelSize;
  m_adapThreshConst=adapThreshConst;
  m_cannyThreshold=cannyThreshold;
  m_houghThreshold=houghThreshold;
  m_lineDiff=lineDiff;
  m_OneLineDiff=OneLineDiff;
  m_HorisontalLimit=HorisontalLimit;
  m_memThreshold=memThreshold;
  m_upperLaneLimit=upperLaneLimit;
  m_lowerLaneLimit=lowerLaneLimit;
  m_roi[0]=roiX;
  m_roi[1]=roiY;
  m_roi[2]=roiWidth;
  m_roi[3]=roiHeight;
  m_debug=debug;
	
  m_currentImg = image.clone();
  UpdateVisualMemory();
  UpdateVisualLines();
  // Get parametric line representation
  std::vector<std::pair<cv::Vec2f, cv::Vec2f>> linesParam = GetParametricRepresentation(m_linesProcessed);
	
  // Update points on lines
  UpdatePointsOnLines(linesParam);
	
  // Pair up lines to form a surface
  m_currentLaneLineIds.clear();
  m_laneLineIds.clear();
  m_laneLineIds = GetLanes();
  m_currentLaneLineIds = GetCurrentLane();
	
  if (m_debug) {
    DrawWindows();
  }
	//TODO: Update middel lane calculation: opendlv::model::Cartesian3> -> EIGEN.Vector
/*	    if (edges.size() >= 4) {
      double x1 = edges[0].getX();
      double y1 = edges[0].getY();
      double x2 = edges[2].getX();
      double y2 = edges[2].getY();

      double xAim = (x1 + x2) / 2.0;
      double yAim = (y1 + y2) / 2.0;

      double aimAngle = atan2(yAim, xAim);

      std::cout << "Aim point: " << xAim << ":" << yAim << std::endl;
      std::cout << "Aim angle: " << aimAngle << std::endl;

      odcore::data::TimeStamp now;
      opendlv::model::Direction direction(0.0f, 0.0f);
      opendlv::model::Direction desiredDirection(static_cast<float>(aimAngle), 0.0f);
      opendlv::perception::StimulusDirectionOfMovement stimulus(now, desiredDirection, direction);
     
      odcore::data::Container c(stimulus);
      getConference().send(c);
    }
	*/
  for (auto it = m_currentLaneLineIds.begin(); it != m_currentLaneLineIds.end(); it++) {
	std::cout << (m_xWorldP[*it][0], m_yWorldP[*it][0]) << std::endl;
	std::cout << (m_xWorldP[*it][1], m_yWorldP[*it][1]) << std::endl;
  }
}
  
void DetectLane::UpdateVisualMemory() {
  cluon::data::TimeStamp  now;
  cv::Rect rectROI(m_roi[0], m_roi[1], m_roi[2], m_roi[3]);
  cv::Mat visualImpression;
  try {
    std::lock_guard<std::mutex> l(m_mtx);
    visualImpression = m_currentImg(rectROI).clone();
  } catch (cv::Exception& e) {
    std::cerr << "Error cropping the image due to dimension size. " << std::endl;
    return;
  }
  // Reduce the noise in image
  cv::medianBlur(visualImpression, visualImpression, m_blurKernelSize);
  
  m_visualMemory.push_back(std::make_pair(now, visualImpression));

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
  cv::HoughLines(m_cannyImg, detectedLinesCanny, 1, M_PI/180.0, m_houghThreshold);
  cv::HoughLines(m_adapThreshImg, detectedLinesAdapThresh, 1, M_PI/180.0, m_houghThreshold);
  m_linesRaw = detectedLinesCanny;
  m_linesRaw.insert(m_linesRaw.end(), detectedLinesAdapThresh.begin(), detectedLinesAdapThresh.end());
  double const DIST_RADIUS = 100.0;
  m_linesProcessed = GetGrouping(m_linesRaw, DIST_RADIUS);
}

// Grouping or lumping close lying lines into groups
// This can be improved and optimized
std::vector<cv::Vec2f> DetectLane::GetGrouping(std::vector<cv::Vec2f> a_lines, double a_groupingRadius)
{
  if (a_lines.empty()) {
    return a_lines;
  }
  std::vector<std::vector<cv::Vec2f>> group;
  std::vector<cv::Vec2f> groupMean;
  std::vector<cv::Vec2f> groupSum;
  std::vector<cv::Vec2f> empty;
  group.push_back(empty);
  group.back().push_back(a_lines.at(0));
  groupMean.push_back(group.at(0).at(0));
  groupSum.push_back(group.at(0).at(0));
  
  for (uint16_t i = 1; i < a_lines.size(); i++) {
    bool groupAssigned = false;
    for (uint16_t j = 0; j < group.size() && !groupAssigned; j++) {
      double xDiff = a_lines[i][0] - groupMean[j][0];
      double yDiff = a_lines[i][1] - groupMean[j][1];
      double absDiff = sqrt(pow(xDiff, 2) + pow(yDiff, 2));

      if (absDiff <= a_groupingRadius) {
        group.at(j).push_back(a_lines.at(i));
        groupSum.at(j) += a_lines.at(i);
        groupMean.at(j)[0] = groupSum.at(j)[0] / group.at(j).size();
        groupMean.at(j)[1] = groupSum.at(j)[1] / group.at(j).size();
        groupAssigned = true;
      }
    }
    if (!groupAssigned) {
      group.push_back(empty);
      group.back().push_back(a_lines.at(i));
      groupMean.push_back(a_lines.at(i));
      groupSum.push_back(a_lines.at(i));
    }
  }
  return groupMean;
}
// Converting to the conventional line representation
std::vector<std::pair<cv::Vec2f, cv::Vec2f>> DetectLane::GetParametricRepresentation(
    std::vector<cv::Vec2f> a_groups)
{
  std::vector<std::pair<cv::Vec2f, cv::Vec2f>> lineParamRep;
  for (uint16_t i = 0; i < a_groups.size(); i++) {
    float rho = a_groups[i][0];
    float theta = a_groups[i][1];
    float x0 = cosf(theta)*rho;
    float y0 = sinf(theta)*rho;
    float heading = -((float)M_PI / 2.0f - theta);
    lineParamRep.push_back(std::make_pair(cv::Vec2f((float)cos(heading),(float)sin(heading)), cv::Vec2f(x0,y0)));
  }
  return lineParamRep;
}

// Finding corresponding point by using the intersection of the detected lines and the limit ROI
void DetectLane::UpdatePointsOnLines(std::vector<std::pair<cv::Vec2f, cv::Vec2f>> a_param)
{
  m_xScreenP.clear();
  m_yScreenP.clear();
  m_xWorldP.clear();
  m_yWorldP.clear();
  for (auto it = a_param.begin(); it != a_param.end(); it++) {
    float t1;
    float t2;
    // To handle special case of dividing 0  
    if (fabs((*it).first[1]) > 0.000001f) {
      t1 =  (static_cast<float>(m_upperLaneLimit - m_roi[1]) - (*it).second[1]) / (float)sin((*it).first[1]);
      t2 =  (static_cast<float>(m_lowerLaneLimit - m_roi[1]) - (*it).second[1]) / (float)sin((*it).first[1]);
    } else {
      t1 = 0;
      t2 = 0;
    }
    Eigen::Vector3d point1, point2;
    
    float x1 = (t1 * (*it).first[0] + (*it).second[0]) + m_roi[0]; 
    point1 << x1, m_upperLaneLimit, 1;
    point1 = TransformPointToGlobalFrame(point1);

    float x2 =  (t2 * (*it).first[0] + (*it).second[0] + m_roi[0]); 
    point2 << x2, m_lowerLaneLimit, 1;
    point2 = TransformPointToGlobalFrame(point2);

    m_xWorldP.push_back(cv::Vec2f((float)point1(0),(float)point2(0)));
    m_yWorldP.push_back(cv::Vec2f((float)point1(1),(float)point2(1)));

    m_xScreenP.push_back(cv::Vec2f(x1,x2));
    m_yScreenP.push_back(cv::Vec2f((float)m_upperLaneLimit,(float)m_lowerLaneLimit));
    
  }
}

// Finding the current lane
// This could be improved. Grave assumptions.
std::vector<uint16_t> DetectLane::GetCurrentLane() const
{
  std::vector<uint16_t> lineIds;
  int8_t leftLineId = -1;
  int8_t rightLineId = -1;
  for (uint8_t i = 0; i < m_yWorldP.size(); i++) {
    if (m_yWorldP[i][1] < 0) {
      if (leftLineId == -1 || m_yWorldP[i][1] > m_yWorldP[leftLineId][1]) {
        leftLineId = i;
      }
    } else {
      if (rightLineId == -1 ||m_yWorldP[i][1] < m_yWorldP[rightLineId][1]) {
        rightLineId = i;
      }
    }
  }
  if (leftLineId != rightLineId && leftLineId != -1 && rightLineId != -1) {
    lineIds.push_back(leftLineId);
    lineIds.push_back(rightLineId);
  }
  return lineIds;
}

Eigen::MatrixXd DetectLane::ReadMatrix(std::string const a_fileName
    , uint8_t const a_nRows
    , uint8_t const a_nCols) const
{
  std::ifstream file(a_fileName);
  Eigen::MatrixXd matrix(a_nRows, a_nCols);
  if (file.is_open()) {
    for (int i = 0; i < a_nRows; i++) {
      for (int j = 0; j < a_nCols; j++) {
        double item = 0.0;
        file >> item;
        matrix(i,j) = item;
      }
    }
    std::cout << "Read the projection matrix: " << matrix << std::endl;
  } else {
    std::cout << "Couldn't read the projection matrix." << std::endl;
  }
  file.close();
  return matrix;
}
Eigen::Vector3d DetectLane::TransformPointToGlobalFrame(Eigen::Vector3d a_point) const
{
  a_point = m_transformationMatrix * a_point;
  a_point = a_point / a_point(2);
  return a_point;
}

std::vector<uint16_t> DetectLane::GetLanes() const
{
  std::vector<uint16_t> laneLineIds;
  // Filter out lines that are close to each other
  // This could be greatly improved and revised
  std::vector<uint16_t> toBeRemoved;
  for (uint16_t i = 0; i < m_yWorldP.size(); i++) {
    bool check = true;
    if (std::abs(m_yWorldP[i][0] - m_yWorldP[i][1]) > m_OneLineDiff ) {
      toBeRemoved.push_back(i);
      // std::cout << "Too big diff between the line itself: " << (m_yWorldP[i][0] - m_yWorldP[i][1]) << " numbers: " << m_yWorldP[i][0] << " , " << m_yWorldP[i][1] << std::endl;
      check = false;
    }
    else if ((std::abs(m_yWorldP[i][0]) > m_HorisontalLimit) || (std::abs(m_yWorldP[i][1]) > m_HorisontalLimit)) {
      toBeRemoved.push_back(i);
      // std::cout << "Out of scope, more than : " << m_HorisontalLimit << "m away from the truck" << std::endl;
      check = false;
    }
    // Finding the closest Y-axis point to the truck and removing the ones who are not
    for (uint16_t j = i+1; j < m_yWorldP.size() && check; j++) {
      if (m_yWorldP[i][1] * m_yWorldP[j][1] > 0 && std::abs(m_yWorldP[i][1] - m_yWorldP[j][1]) < m_lineDiff) {
        if (std::abs(m_yWorldP[i][1]) > std::abs(m_yWorldP[j][1])) {
          toBeRemoved.push_back(i);
        } else {
          toBeRemoved.push_back(j);
        }
      }
    }
  }
  // std::cout << "To be removed ID: ";
  // for (uint16_t i = 0; i < toBeRemoved.size(); i++) {
  //   std::cout << toBeRemoved.at(i) << ", ";
  // }
  // std::cout << std::endl;
  // Loop through all coordinates 
  for (uint16_t i = 0; i < m_xScreenP.size(); i++) {    
    bool hit = false;
    // Loop through and check if the coordinate should be removed or not
    for (auto it = toBeRemoved.begin(); it != toBeRemoved.end() && !hit; it++) {
      if (i == *it) {
        hit = true;
      }
    }
    if (!hit) {
      laneLineIds.push_back(i);
    }
  }
  return laneLineIds;
}


void DetectLane::DrawWindows()
{
  // Print out all detected lines
  for (uint16_t i = 0; i < m_linesProcessed.size(); i++) {
    float rho = m_linesProcessed[i][0];
    float theta = m_linesProcessed[i][1];

    float a = (float)cos(theta), b = (float)sin(theta);
    float x0 = (a*rho), y0 = (b*rho);
	int tempX = (int)((float) (m_roi[0] + x0) + 2000.0f*(-b));
	int tempY = (int)((float) (m_roi[1] + y0) + 2000.0f*(a));
	cv::Point pt1(tempX,tempY);
    tempX = (int)((float) (m_roi[0] + x0) - 2000.0f*(-b));
	tempY = (int)((float) (m_roi[1] + y0) - 2000.0f*(a));
	cv::Point pt2(tempX, tempY);

    cv::line(m_currentImg, pt1, pt2, cv::Scalar(0,0,255), 2, 1 );
  }

  // Printing out screen points
  for (uint8_t i = 0; i < m_laneLineIds.size(); i++) {
    for (uint8_t k = 0; k < 2; k++) {
	  cv::Point point((uint)m_xScreenP[m_laneLineIds[i]][k], (uint)m_yScreenP[m_laneLineIds[i]][k]);
	  // int cv_point_int = cv::Point(m_xScreenP[m_laneLineIds[i]][k], m_yScreenP[m_laneLineIds[i]][k]);
      cv::circle(m_currentImg, point, 10, cv::Scalar(0,255,0), 3, 8);
    }
  }
  // Printing out current lane points
  for (uint8_t i = 0; i < m_currentLaneLineIds.size(); i++) {
    for (uint8_t k = 0; k < 2; k++) {
	  cv::Point point((uint)m_xScreenP[m_currentLaneLineIds[i]][k], (uint)m_yScreenP[m_currentLaneLineIds[i]][k]);
      cv::circle(m_currentImg, point, 10, cv::Scalar(0,0,255), 3, 8);
    }
  }
  
  // Add the lane limitation lines to the image
  cv::line(m_currentImg, cv::Point(0,(int)m_upperLaneLimit), cv::Point(m_screenSize[0],(int)m_upperLaneLimit), cv::Scalar(255,0,0), 3, 1 );
  cv::line(m_currentImg, cv::Point(0,(int)m_lowerLaneLimit), cv::Point(m_screenSize[0],(int)m_lowerLaneLimit), cv::Scalar(255,0,0), 3, 1 );
  
  // Add the roi to the image
  cv::line(m_currentImg, cv::Point(m_roi[0],0), cv::Point(m_roi[0],m_screenSize[1]), cv::Scalar(255,255,0), 3, 1 );
  cv::line(m_currentImg, cv::Point(m_roi[0]+m_roi[2],0), cv::Point(m_roi[0]+m_roi[2],m_screenSize[1]), cv::Scalar(255,255,0), 3, 1 );
  cv::line(m_currentImg, cv::Point(0,m_roi[1]), cv::Point(m_screenSize[0],m_roi[1]), cv::Scalar(255,255,0), 3, 1 );
  cv::line(m_currentImg, cv::Point(0,m_roi[1]+m_roi[3]), cv::Point(m_screenSize[0],m_roi[1]+m_roi[3]), cv::Scalar(255,255,0), 3, 1 ); 


  const int32_t windowWidth = 640/2;
  const int32_t windowHeight = 400/2;

  // Set number of windows to display
  cv::Mat display[4];
  
  cv::resize(m_visualMemory.back().second, display[0], cv::Size(windowWidth,windowHeight), 0, 0, cv::INTER_AREA);
  cv::imshow("Memory image", display[0]);

  cv::resize(m_adapThreshImg, display[1], cv::Size(windowWidth,windowHeight), 0, 0, cv::INTER_AREA);
  cv::imshow("Adaptive threshold", display[1]);

  cv::resize(m_cannyImg, display[2], cv::Size(windowWidth,windowHeight), 0, 0, cv::INTER_AREA);
  cv::imshow("Canny image", display[2]);

  cv::resize(m_currentImg, display[3], cv::Size(windowWidth, windowHeight), 0, 0, cv::INTER_AREA);
  cv::imshow("Result", display[3]);

  cv::waitKey(1);
}
