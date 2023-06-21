#include <Eigen/Eigen>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>

using namespace std;
using namespace Eigen;

ros::Publisher odom_pub;
ros::Publisher path_pub;
nav_msgs::Path path_ekf;
ros::Time current_time, last_time;

MatrixXd Q  = MatrixXd::Identity( 12, 12 );
MatrixXd Rt = MatrixXd::Identity( 6, 6 );

void
imu_callback( const sensor_msgs::Imu::ConstPtr& msg )
{
    // your code for propagation
}

// Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void
odom_callback( const nav_msgs::Odometry::ConstPtr& msg )
{
    // your code for update
    // camera position in the IMU frame = (0.05, 0.05, 0)
    // camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //							                 0, -1, 0,
    //                               0, 0, -1;
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "ekf" );
    ros::NodeHandle n( "~" );

    ros::Subscriber s1 = n.subscribe( "imu", 1000, imu_callback );
    ros::Subscriber s2 = n.subscribe( "tag_odom", 1000, odom_callback );

    odom_pub = n.advertise< nav_msgs::Odometry >( "ekf_odom", 100 );
    path_pub = n.advertise< nav_msgs::Path >( "ekf_path", 10000 );

    Rcam = Quaterniond( 0, 1, 0, 0 ).toRotationMatrix( );
    cout << "R_cam" << endl << Rcam << endl;

    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    Q.topLeftCorner( 6, 6 )      = 0.01 * Q.topLeftCorner( 6, 6 );
    Q.bottomRightCorner( 6, 6 )  = 0.01 * Q.bottomRightCorner( 6, 6 );
    Rt.topLeftCorner( 3, 3 )     = 0.1 * Rt.topLeftCorner( 3, 3 );
    Rt.bottomRightCorner( 3, 3 ) = 0.1 * Rt.bottomRightCorner( 3, 3 );
    Rt.bottomRightCorner( 1, 1 ) = 0.1 * Rt.bottomRightCorner( 1, 1 );

    path_ekf.header.frame_id = "world";

    ros::spin( );
}
