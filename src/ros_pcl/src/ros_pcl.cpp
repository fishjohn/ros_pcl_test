//
// Created by luohx on 20-4-27.
//

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.
#include <pcl/io/ply_io.h>

#include <stdio.h>
#include <cstring>

typedef pcl::PointXYZ PointT;  //x,y,z
typedef pcl::PointCloud<PointT> PointCloudT;//

bool next_iteration = false;

//print rotation matrix and translation matrix
void print4x4Matrix(const Eigen::Matrix4d &matrix) {
  printf("Rotation matrix :\n");
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
  printf("Translation vector :\n");
  printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void keyboardEventOccurred(const std_msgs::String::ConstPtr &str) {//use space key to step in next iteration and update visualization
  if (std::strncmp(str->data.c_str(), " ", 1) == 0) {
    next_iteration = true;
    std::cout << "keyboard input space~" << std::endl;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ros_pcl");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub_in = nh.advertise<sensor_msgs::PointCloud2>("pcl_in", 30);
  ros::Publisher pcl_pub_tr = nh.advertise<sensor_msgs::PointCloud2>("pcl_tr", 30);
  ros::Publisher pcl_pub_icp = nh.advertise<sensor_msgs::PointCloud2>("pcl_icp", 30);
  ros::Time now_time, last_time;

  int iterations = 10;
  PointCloudT::Ptr cloud_in(new PointCloudT);
  PointCloudT::Ptr cloud_tr(new PointCloudT);
  PointCloudT::Ptr cloud_icp(new PointCloudT);
  //load point cloud data from pcd file and put in cloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile("/home/luohx/code/pcl_test/table_scene_lms400.pcd", *cloud_in) < 0) {
    ROS_ERROR("Error loadind cloud");
    return -1;
  }

  //voxel_grid filter
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_in);
  sor.setLeafSize(0.02, 0.02, 0.02);
  sor.filter(*cloud_in);

  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity(); //init an identity matrix

  double theta = M_PI / 4;
  transformation_matrix(0, 0) = cos(theta);//  rotation around the Z axis
  transformation_matrix(0, 1) = -sin(theta);
  transformation_matrix(1, 0) = sin(theta);
  transformation_matrix(1, 1) = cos(theta);
  //Z-axis translation vector
  transformation_matrix(2, 3) = 0.2;
  // print transformation matrix
  std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
  print4x4Matrix(transformation_matrix);

  pcl::transformPointCloud(*cloud_in, *cloud_icp, transformation_matrix);
  *cloud_tr = *cloud_icp;

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(iterations);
  icp.setInputSource(cloud_icp);
  icp.setInputTarget(cloud_in);
  icp.align(*cloud_icp);
  icp.setMaximumIterations(1);

  if (icp.hasConverged()) {
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
    std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
    transformation_matrix = icp.getFinalTransformation().cast<double>();
    print4x4Matrix(transformation_matrix);
  } else {
    ROS_ERROR("\nICP has not converged.\n");
    return (-1);
  }
  //change pcl::PointCloud to sensor_msgs::PointCloud2 and send to rviz to view
  sensor_msgs::PointCloud2 output_in;
  sensor_msgs::PointCloud2 output_tr;
  sensor_msgs::PointCloud2 output_icp;
  pcl::toROSMsg(*cloud_in, output_in);
  pcl::toROSMsg(*cloud_tr, output_tr);
  pcl::toROSMsg(*cloud_icp, output_icp);
  output_in.header.frame_id = "odom";
  output_tr.header.frame_id = "odom";
  output_icp.header.frame_id = "odom";

  ros::Subscriber sub = nh.subscribe("keys", 1, keyboardEventOccurred);
  while (ros::ok()) {
    if (next_iteration) {
      iterations = 10;
      now_time = ros::Time::now();
      last_time = now_time;
      for (int i = 0; i < 10; i++) {
        std::cout << "Applied 1 ICP iteration in " << std::endl;
        icp.align(*cloud_icp);

        if (icp.hasConverged()) {
          std::cout << "\nICP transformation " << i << " : cloud_icp -> cloud_in" << std::endl;
          transformation_matrix *= icp.getFinalTransformation().cast<double>();
          print4x4Matrix(transformation_matrix);
          pcl::toROSMsg(*cloud_icp, output_icp);
          output_icp.header.frame_id = "odom";
        } else {
          ROS_ERROR ("\nICP has not converged.\n");
          return (-1);
        }
      }
      now_time = ros::Time::now();
      std::cout << "ICP 10 times iterations takes time :" << now_time - last_time << "s" << std::endl;
    }
    next_iteration = false;

    pcl_pub_in.publish(output_in);
    pcl_pub_icp.publish(output_icp);
    pcl_pub_tr.publish(output_tr);
    ros::spinOnce();
  }
  return 0;
}