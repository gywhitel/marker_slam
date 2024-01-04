# ifndef MATH_HPP
# define MATH_HPP

#include <vector>
#include <opencv2/opencv.hpp>

using std::vector;

/**
  * @brief Return object points for the system centered in a single marker, given the marker length
  */
static void getSingleMarkerObjectPoints(float markerLength, vector<cv::Point3f>& objPoints) {
    
    CV_Assert(markerLength > 0);

    // set coordinate system in the middle of the marker, with Z pointing out
    objPoints.clear();
    objPoints.push_back(cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(cv::Vec3f( markerLength / 2.f, markerLength / 2.f, 0));
    objPoints.push_back(cv::Vec3f( markerLength / 2.f,-markerLength / 2.f, 0));
    objPoints.push_back(cv::Vec3f(-markerLength / 2.f,-markerLength / 2.f, 0));
}

// Euclidean distance between two points
static double dist(const cv::Point2f &p1, const cv::Point2f &p2)
{
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;

    double dx = x1 - x2;
    double dy = y1 - y2;

    return sqrt(dx*dx + dy*dy);
}

// Compute area in image of a fiducial, using Heron's formula
// to find the area of two triangles
static double calcFiducialArea(const std::vector<cv::Point2f> &pts)
{
    const cv::Point2f &p0 = pts.at(0);
    const cv::Point2f &p1 = pts.at(1);
    const cv::Point2f &p2 = pts.at(2);
    const cv::Point2f &p3 = pts.at(3);

    double a1 = dist(p0, p1);
    double b1 = dist(p0, p3);
    double c1 = dist(p1, p3);

    double a2 = dist(p1, p2);
    double b2 = dist(p2, p3);
    double c2 = c1;

    double s1 = (a1 + b1 + c1) / 2.0;
    double s2 = (a2 + b2 + c2) / 2.0;

    a1 = sqrt(s1*(s1-a1)*(s1-b1)*(s1-c1));
    a2 = sqrt(s2*(s2-a2)*(s2-b2)*(s2-c2));
    return a1+a2;
}

// estimate reprojection error
static double getReprojectionError(const vector<cv::Point3f> &objectPoints,
                            const vector<cv::Point2f> &imagePoints,
                            const cv::Mat &cameraMatrix, const cv::Mat  &distCoeffs,
                            const cv::Vec3d &rvec, const cv::Vec3d &tvec) {

    vector<cv::Point2f> projectedPoints;

    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix,
                      distCoeffs, projectedPoints);

    // calculate RMS image error
    double totalError = 0.0;
    for (unsigned int i=0; i<objectPoints.size(); i++) {
        double error = dist(imagePoints[i], projectedPoints[i]);
        totalError += error*error;
    }
    double rerror = totalError/(double)objectPoints.size();
    return rerror;
}

# endif
