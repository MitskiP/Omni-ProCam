#pragma once

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vector>

class BetterBlobDetector : public cv::SimpleBlobDetector
{
public:

    BetterBlobDetector(const cv::SimpleBlobDetector::Params &parameters = cv::SimpleBlobDetector::Params());

    const std::vector < std::vector<cv::Point> > getContours();

    virtual void detect( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat());

protected:

  struct CV_EXPORTS Center
  {
      cv::Point2d location;
      double radius;
      double confidence;
  };


    virtual void detectImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat()) const;
    virtual void findBlobs(const cv::Mat &image, const cv::Mat &binaryImage,
                           std::vector<Center> &centers, std::vector < std::vector<cv::Point> >&contours) const;

    Params params;
};
