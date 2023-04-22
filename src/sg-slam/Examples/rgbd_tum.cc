/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


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

using namespace std;

ros::Publisher CamPose_Pub;
ros::Publisher Camodom_Pub;
ros::Publisher odom_pub;

geometry_msgs::PoseStamped Cam_Pose;
geometry_msgs::PoseWithCovarianceStamped Cam_odom;
nav_msgs::Odometry odom;

cv::Mat Camera_Pose;
tf::Transform sg_slam_tf;
tf::TransformBroadcaster * sg_slam_tf_broadcaster;

ros::Time current_time, last_time;
double lastx=0,lasty=0,lastth=0;

void Pub_CamPose(cv::Mat &pose);
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    ros::init(argc, argv , "rgbd_tum");
    ros::start();

    if(argc != 5)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;

    ros::Rate loop_rate(50);
    ros::NodeHandle nh;
    CamPose_Pub = nh.advertise<geometry_msgs::PoseStamped>("/Camera_Pose",1);
    Camodom_Pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/Camera_Odom", 1);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    for(int ni=0; ni<nImages && ros::ok() ; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }


        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        Camera_Pose = SLAM.TrackRGBD(imRGB,imD,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        //std::cout<<ttrack*1000<<std::endl;
        
        vTimesTrack[ni]=ttrack;

        Pub_CamPose(Camera_Pose); 

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

	if(SLAM.is_global_pc_reconstruction)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		*global_point_cloud = SLAM.GetPointCloudMapper()->get_globalMap();
        if(!global_point_cloud->empty())
		    pcl::io::savePCDFileBinary("global_pcd.pcd",*global_point_cloud);

	}
    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    return 0;
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

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
