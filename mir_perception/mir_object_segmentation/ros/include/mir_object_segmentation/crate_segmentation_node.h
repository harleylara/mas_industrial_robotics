/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#ifndef MIR_OBJECT_SEGMENTATION_CRATE_SEGMENTATION_NODE_H
#define MIR_OBJECT_SEGMENTATION_CRATE_SEGMENTATION_NODE_H

#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

/** \brief This node subscribes to pointcloud topic.
 * Inputs:
 * ~event_in:
 *      - e_start: starts subscribing to pointcloud topic, and publishing object
 *      lists
 *      - e_stop: stops subscribing
 * Outputs:
 * ~event_out:
 *      - e_started: started listening to new messages
 *      - e_stopped: stopped subscribing
 *
 * \author Santosh Thoduka
 */

namespace mpu = mir_perception_utils;
using mpu::visualization::BoundingBoxVisualizer;
using mpu::visualization::ClusteredPointCloudVisualizer;
using mpu::visualization::LabelVisualizer;
using mpu::visualization::Color;

class CrateSegmentationNode
{
 public:
  CrateSegmentationNode();
  virtual ~CrateSegmentationNode();

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_debug_;
  ros::Publisher pub_object_list_;
  ros::Publisher pub_pose_list_;
  ros::Publisher pub_event_out_;

  ros::Subscriber sub_cloud_;
  ros::Subscriber sub_event_in_;

  boost::shared_ptr<tf::TransformListener> tf_listener_;

  CrateDetection crate_detection;

  BoundingBoxVisualizer bounding_box_visualizer_;
  ClusteredPointCloudVisualizer cluster_visualizer_;
  LabelVisualizer label_visualizer_;

  // Parameters
  std::string target_frame_id_;
  std::string source_frame_id_;

 private:
  void pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &msg);
  void eventCallback(const std_msgs::String::ConstPtr &msg);
};

#endif  // MIR_OBJECT_SEGMENTATION_CRATE_SEGMENTATION_NODE_H
