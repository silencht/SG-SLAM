#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<cv_bridge/cv_bridge.h>
#include<message_filters/subscriber.h>
#include<message_filters/time_synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<pcl/io/pcd_io.h>

using namespace std;

ros::Publisher CamPose_Pub;
ros::Publisher Camodom_Pub;
ros::Publisher odom_pub;

geometry_msgs::PoseStamped Cam_Pose;
geometry_msgs::PoseWithCovarianceStamped Cam_odom;
nav_msgs::Odometry odom;

cv::Mat Camera_Pose;
tf::Transform sg_slam_tf;
tf::TransformBroadcaster *sg_slam_tf_broadcaster;

ros::Time current_time, last_time;
double lastx=0,lasty=0,lastth=0; 

void Pub_CamPose(cv::Mat &pose);

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

private:
    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sg_slam_ros_rgbd");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun sg_slam_ros_rgbd path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ros::Rate loop_rate(50);
    ros::NodeHandle nh;
    CamPose_Pub = nh.advertise<geometry_msgs::PoseStamped>("/Camera_Pose",1);
    Camodom_Pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/Camera_Odom", 1);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);

    ImageGrabber igb(&SLAM);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 30);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 30);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10),rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

//  save global point cloud to .pcd file 
	if(SLAM.is_global_pc_reconstruction)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		*global_point_cloud = SLAM.GetPointCloudMapper()->get_globalMap();
		if(!global_point_cloud->empty())
			pcl::io::savePCDFileBinary("global_pcd.pcd",*global_point_cloud);

	}
    // Stop all threads
    SLAM.Shutdown();
    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB,"bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Camera_Pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    Pub_CamPose(Camera_Pose); 
}

void Pub_CamPose(cv::Mat &pose)
{
    cv::Mat Rwc(3,3,CV_32F);
	cv::Mat twc(3,1,CV_32F);
	Eigen::Matrix<double,3,3> rotationMat;
	sg_slam_tf_broadcaster = new tf::TransformBroadcaster;
	if(pose.dims < 2 || pose.rows < 3)
	{
        Rwc = Rwc;
		twc = twc;
	}
	else
	{
		Rwc = pose.rowRange(0,3).colRange(0,3).t();//pose is Tcw, so Rwc need .t()
		twc = -Rwc*pose.rowRange(0,3).col(3);
		
		rotationMat << Rwc.at<float>(0,0), Rwc.at<float>(0,1), Rwc.at<float>(0,2),
					   Rwc.at<float>(1,0), Rwc.at<float>(1,1), Rwc.at<float>(1,2),
					   Rwc.at<float>(2,0), Rwc.at<float>(2,1), Rwc.at<float>(2,2);
		Eigen::Quaterniond Q(rotationMat);

		// sg-slam's trans. x is twc.at<float>(0), y is twc.at<float>(1), z is twc.at<float>(2)
		// ros's x <-- sg-slam's Z; ros's y <-- sg-slam's -x; ros's z <-- sg-slam's -y
		sg_slam_tf.setOrigin(tf::Vector3(twc.at<float>(2), -twc.at<float>(0), -twc.at<float>(1)));
		sg_slam_tf.setRotation(tf::Quaternion(Q.z(), -Q.x(), -Q.y(), Q.w()));
		//sg_slam_tf_broadcaster.sendTransform(tf::StampedTransform(sg_slam_tf, ros::Time::now(), "/map", "/camera"));
		//delete sg_slam_tf_broadcaster;
		Cam_Pose.header.stamp = ros::Time::now();
		Cam_Pose.header.frame_id = "/map";
		tf::pointTFToMsg(sg_slam_tf.getOrigin(), Cam_Pose.pose.position);
		tf::quaternionTFToMsg(sg_slam_tf.getRotation(), Cam_Pose.pose.orientation);

		CamPose_Pub.publish(Cam_Pose);

		/*
		Cam_odom.header.stamp = ros::Time::now();
		Cam_odom.header.frame_id = "/map";
		tf::pointTFToMsg(sg_slam_tf.getOrigin(), Cam_odom.pose.pose.position);
		tf::quaternionTFToMsg(sg_slam_tf.getRotation(), Cam_odom.pose.pose.orientation);
		Cam_odom.pose.covariance = {0.01, 0, 0, 0, 0, 0,
									0, 0.01, 0, 0, 0, 0,
									0, 0, 0.01, 0, 0, 0,
									0, 0, 0, 0.01, 0, 0,
									0, 0, 0, 0, 0.01, 0,
									0, 0, 0, 0, 0, 0.01};
		Camodom_Pub.publish(Cam_odom);
		*/
		
		/*
		odom.header.stamp =ros::Time::now();
		odom.header.frame_id = "/map";

		// Set the position
		odom.pose.pose.position = Cam_odom.pose.pose.position;
		odom.pose.pose.orientation = Cam_odom.pose.pose.orientation;

		// Set the velocity
		odom.child_frame_id = "/camera_sensor";
		current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();
		double vx = (Cam_odom.pose.pose.position.x - lastx)/dt;
		double vy = (Cam_odom.pose.pose.position.y - lasty)/dt;
		double vth = (Cam_odom.pose.pose.orientation.z - lastth)/dt;
		
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		// Publish the message
		odom_pub.publish(odom);
		
		last_time = current_time;
		lastx = Cam_odom.pose.pose.position.x;
		lasty = Cam_odom.pose.pose.position.y;
		lastth = Cam_odom.pose.pose.orientation.z;
		*/
	}
}


