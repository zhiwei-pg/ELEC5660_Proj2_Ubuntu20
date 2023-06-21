#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <board.h>
#include <Eigen/Eigen>
#include <Eigen/SVD>
//EIgen SVD libnary, may help you solve SVD
//JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

using namespace cv;
using namespace Eigen;

//global varialbles for aruco detector
float MarkerSize = 0.202; // 0.20 / 1.5 * 1.524;
old_aruco::BoardConfiguration BoardConfig;
ros::Publisher pub_odom_yourwork;
ros::Publisher pub_odom_ref;
cv::Mat cameraMatrix, distCoeffs;
cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

// test function, can be used to verify your estimation
void calculateReprojectionError(const std::vector<cv::Point3f> &pts_3, const std::vector<cv::Point2f> &pts_2, const cv::Mat R, const cv::Mat t)
{
    puts("calculateReprojectionError begins");
    std::vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, cameraMatrix, distCoeffs);
    for (unsigned int i = 0; i < pts_3.size(); i++)
    {
        cv::Mat p_mat(3, 1, CV_64FC1);
        p_mat.at<double>(0, 0) = pts_3[i].x;
        p_mat.at<double>(1, 0) = pts_3[i].y;
        p_mat.at<double>(2, 0) = pts_3[i].z;
        cv::Mat p = (R * p_mat + t);
        printf("(%f, %f, %f) -> (%f, %f) and (%f, %f)\n",
               pts_3[i].x, pts_3[i].y, pts_3[i].z,
               un_pts_2[i].x, un_pts_2[i].y,
               p.at<double>(0) / p.at<double>(2), p.at<double>(1) / p.at<double>(2));
    }
    puts("calculateReprojectionError ends");
}

// the main function you need to work with
// pts_3: 3D position (x, y, z) in world frame
// pts_2: 2D position (u, v) in image frame
void process(const std::vector<cv::Point3f> &pts_3, const std::vector<cv::Point2f> &pts_2, const ros::Time& frame_time)
{
    //version 1, as reference
    cv::Mat r, rvec, t;
    cv::solvePnP(pts_3, pts_2, cameraMatrix, distCoeffs, rvec, t);
    cv::Rodrigues(rvec, r);
    Matrix3d R_ref;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {
            R_ref(i,j) = r.at<double>(i, j);
        }
    Quaterniond Q_ref;
    Q_ref = R_ref;
    nav_msgs::Odometry odom_ref;
    odom_ref.header.stamp = frame_time;
    odom_ref.header.frame_id = "world";
    odom_ref.pose.pose.position.x = t.at<double>(0, 0);
    odom_ref.pose.pose.position.y = t.at<double>(1, 0);
    odom_ref.pose.pose.position.z = t.at<double>(2, 0);
    odom_ref.pose.pose.orientation.w = Q_ref.w();
    odom_ref.pose.pose.orientation.x = Q_ref.x();
    odom_ref.pose.pose.orientation.y = Q_ref.y();
    odom_ref.pose.pose.orientation.z = Q_ref.z();
    pub_odom_ref.publish(odom_ref);

    // version 2, your work
    Matrix3d R;
    Vector3d T;
    R.setIdentity();
    T.setZero();
    ROS_INFO("write your code here!");
    //...
    //...
    //...
    Quaterniond Q_yourwork;
    Q_yourwork = R;
    nav_msgs::Odometry odom_yourwork;
    odom_yourwork.header.stamp = frame_time;
    odom_yourwork.header.frame_id = "world";
    odom_yourwork.pose.pose.position.x = T(0);
    odom_yourwork.pose.pose.position.y = T(1);
    odom_yourwork.pose.pose.position.z = T(2);
    odom_yourwork.pose.pose.orientation.w = Q_yourwork.w();
    odom_yourwork.pose.pose.orientation.x = Q_yourwork.x();
    odom_yourwork.pose.pose.orientation.y = Q_yourwork.y();
    odom_yourwork.pose.pose.orientation.z = Q_yourwork.z();
    pub_odom_yourwork.publish(odom_yourwork);
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{

    double t = clock();

    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(bridge_ptr->image, dictionary, corners, ids);
    cv::Mat imageShow;
    bridge_ptr->image.copyTo(imageShow);

    // If at least one marker detected
    std::vector<cv::Point3f> pts_3;
    std::vector<cv::Point2f> pts_2;
    for(unsigned int i = 0; i < ids.size(); i++)
    {
        // push to PnP
        int id = ids[i];
        for (unsigned int j = 0; j < 4; j++)
        {
            pts_2.push_back( corners[i][j] );
            pts_3.push_back( BoardConfig.getMarkerInfo( id ).at( j ) * MarkerSize / 1000 );
        }
        // draw
        cv::aruco::drawDetectedMarkers(imageShow, corners, ids);
    }
    
    if (ids.size() > 5)
        process(pts_3, pts_2, img_msg->header.stamp);


    cv::imshow("image", imageShow);
    cv::waitKey(10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_detector");
    ros::NodeHandle n("~");

    ros::Subscriber sub_img = n.subscribe("image_raw", 100, img_callback);
    pub_odom_yourwork = n.advertise<nav_msgs::Odometry>("odom_yourwork",10);
    pub_odom_ref = n.advertise<nav_msgs::Odometry>("odom_ref",10);
    //init aruco detector
    std::string cam_cal, board_config;
    n.getParam("cam_cal_file", cam_cal);
    n.getParam("board_config_file", board_config);
    BoardConfig.readFromFile(board_config);

    //init intrinsic parameters
    cv::FileStorage param_reader(cam_cal, cv::FileStorage::READ);
    param_reader["camera_matrix"] >> cameraMatrix;
    param_reader["distortion_coefficients"] >> distCoeffs;

    //init window for visualization
    cv::namedWindow("image", 1);

    ros::spin();
}