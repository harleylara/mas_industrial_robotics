/*
 * Copyright 2018 Bonn-Rhein-Sieg University
 *
 * Author: Mohammad Wasil, Santosh Thoduka
 *
 */
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <std_msgs/Float64.h>

#include <mas_perception_msgs/BoundingBox.h>
#include <mas_perception_msgs/BoundingBoxList.h>
#include <mas_perception_msgs/ObjectList.h>
#include <mas_perception_msgs/RecognizeObject.h>

#include <mir_perception_utils/bounding_box_visualizer.h>
#include <mir_perception_utils/clustered_point_cloud_visualizer.h>
#include <mir_perception_utils/label_visualizer.h>
#include <mir_perception_utils/pointcloud_utils_ros.h>

#include <mir_object_segmentation/crate_detection.h>
#include <mir_object_segmentation/crate_segmentation_node.h>

#include <string>

CrateSegmentationNode::CrateSegmentationNode()
    : nh_("~"),
      bounding_box_visualizer_("output/bounding_boxes", Color(Color::SEA_GREEN)),
      cluster_visualizer_("output/tabletop_clusters")
{
  sub_event_in_ = nh_.subscribe("event_in", 1, &CrateSegmentationNode::eventCallback, this);
  pub_event_out_ = nh_.advertise<std_msgs::String>("event_out", 1);
  pub_object_list_ = nh_.advertise<mas_perception_msgs::ObjectList>("output/object_list", 1);
  pub_pose_list_ = nh_.advertise<geometry_msgs::PoseArray>("output/object_poses", 1);
  pub_debug_ = nh_.advertise<sensor_msgs::PointCloud2>("output/debug_cloud", 1);

  tf_listener_.reset(new tf::TransformListener);

  nh_.param<std::string>("target_frame_id", target_frame_id_, "base_link");
  nh_.param<std::string>("source_frame_id", source_frame_id_, "fixed_camera_link");
}

CrateSegmentationNode::~CrateSegmentationNode() {}

void estimatePose(const BoundingBox &box, geometry_msgs::PoseStamped &pose)
{
  BoundingBox::Points vertices = box.getVertices();
  Eigen::Vector3f n1;
  Eigen::Vector3f n2;
  Eigen::Vector3f n3 = (vertices[4] - vertices[0]) / (vertices[4] - vertices[0]).norm();
  if ((vertices[1] - vertices[0]).norm() > (vertices[3] - vertices[0]).norm()) {
    n1 = (vertices[1] - vertices[0]) / (vertices[1] - vertices[0]).norm();
  } else {
    n1 = (vertices[3] - vertices[0]) / (vertices[3] - vertices[0]).norm();
  }
  n2 = n3.cross(n1);
  Eigen::Matrix3f m;
  m << n1, n2, n3;
  Eigen::Quaternion<float> q(m);
  q.normalize();

  Eigen::Vector3f centroid = box.getCenter();
  pose.pose.position.x = centroid(0);
  pose.pose.position.y = centroid(1);
  pose.pose.position.z = (vertices[0](2) + vertices[1](2) + vertices[2](2) + vertices[3](2)) / 4.0;
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
}


inline void convertBoundingBox(const BoundingBox &bounding_box,
                               mas_perception_msgs::BoundingBox &bounding_box_msg)
{
  const BoundingBox::Point &center = bounding_box.getCenter();
  bounding_box_msg.center.x = center[0];
  bounding_box_msg.center.y = center[1];
  bounding_box_msg.center.z = center[2];
  bounding_box_msg.dimensions.x = bounding_box.getDimensions()[0];
  bounding_box_msg.dimensions.y = bounding_box.getDimensions()[1];
  bounding_box_msg.dimensions.z = bounding_box.getDimensions()[2];

  const BoundingBox::Points &vertices = bounding_box.getVertices();
  for (size_t i = 0; i < vertices.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = vertices[i][0];
    pt.y = vertices[i][1];
    pt.z = vertices[i][2];
    bounding_box_msg.vertices.push_back(pt);
  }
}

void CrateSegmentationNode::pointcloudCallback(const sensor_msgs::PointCloud2::Ptr &msg)
{
    sensor_msgs::PointCloud2ConstPtr input_cloud_msg = msg;
    input_cloud_msg->header.frame_id = this->source_frame_id_;
    sensor_msgs::PointCloud2 msg_transformed;
    mas_perception_msgs::ObjectList object_list;

    if (!mpu::pointcloud::transformPointCloudMsg(tf_listener_, target_frame_id_, *input_cloud_msg,
                                                 msg_transformed))
    {
        ROS_WARN_STREAM("Cannot get transform to target frame " << target_frame_id_);
      return;
    }

    PointCloud::Ptr cloud = boost::make_shared<PointCloud>();
    pcl::PCLPointCloud2 pc2;
    pcl_conversions::toPCL(msg_transformed, pc2);
    pcl::fromPCLPointCloud2(pc2, *cloud);

    PointCloud::Ptr segmented_crate(new PointCloud);
    BoundingBox crate_bb = crate_detection.detectCrate(cloud, segmented_crate);
    std::vector<PointCloud::Ptr> clusters;
    std::vector<BoundingBox> boxes;
    crate_detection.findObjects(segmented_crate, clusters, boxes);

    mas_perception_msgs::BoundingBoxList bounding_boxes;
    bounding_boxes.bounding_boxes.resize(clusters.size());

    geometry_msgs::PoseArray poses;
    //create object list size as per clusters
    object_list.objects.resize(clusters.size());
    poses.header.stamp = ros::Time::now();
    poses.header.frame_id = target_frame_id_;
    std::vector<std::string> labels;
  
    ros::Time now = ros::Time::now();
    for (int i = 0; i < clusters.size(); i++) {
        convertBoundingBox(boxes[i], bounding_boxes.bounding_boxes[i]);
        geometry_msgs::PoseStamped pose;
        estimatePose(boxes[i], pose);
        pose.header.stamp = now;
        pose.header.frame_id = target_frame_id_;

        //hack to set roll and pitch to zero , can be done with Eigen
        //quaterninon in estimate pose function above
        tf::Quaternion temp;
        tf::quaternionMsgToTF(pose.pose.orientation, temp);
        tf::Matrix3x3 m(temp);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
        //End of hack
        
        poses.poses.push_back(pose.pose);


        //Assign ros_cloud 
        sensor_msgs::PointCloud2 ros_cloud;
        ros_cloud.header.frame_id = target_frame_id_;
        pcl::toROSMsg(*clusters[i], ros_cloud);

        // Assign number name for every object by default then recognize it later
        object_list.objects[i].views.resize(1);
        object_list.objects[i].views[0].point_cloud = ros_cloud;
        object_list.objects[i].name = std::to_string(i);;
        object_list.objects[i].probability = 0.0;
        object_list.objects[i].pose = pose;
    }
    pub_object_list_.publish(object_list);
    bounding_box_visualizer_.publish(bounding_boxes.bounding_boxes, target_frame_id_);
    cluster_visualizer_.publish<PointT>(clusters, target_frame_id_);
    pub_pose_list_.publish(poses);


    std_msgs::String event_out;
    event_out.data = "e_add_cloud_stopped";
    pub_event_out_.publish(event_out);

    PointCloud::Ptr cloud_debug(new PointCloud);
    cloud_debug = segmented_crate;
    sensor_msgs::PointCloud2 ros_pc2;
    pcl::toROSMsg(*cloud_debug, ros_pc2);
    ros_pc2.header.frame_id = target_frame_id_;
    pub_debug_.publish(ros_pc2);
    //msg_transformed.header.frame_id = target_frame_id_;
    //pub_debug_.publish(msg_transformed);
}


void CrateSegmentationNode::eventCallback(const std_msgs::String::ConstPtr &msg)
{
  std_msgs::String event_out;
  if (msg->data == "e_start") {
    sub_cloud_ = nh_.subscribe("input", 1, &CrateSegmentationNode::pointcloudCallback, this);
    ROS_INFO_STREAM("Subscribed to pointcloud");
    event_out.data = "e_started";
  } else if (msg->data == "e_stop") {
    sub_cloud_.shutdown();
    ROS_INFO_STREAM("Stopped subscription to pointcloud");
    event_out.data = "e_stopped";
  } else {
    return;
  }
  pub_event_out_.publish(event_out);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crate_segmentation_node");
  CrateSegmentationNode scene_seg;
  ROS_INFO_STREAM("\033[1;32m[crate_segmentation_node] node started \033[0m\n");
  ros::spin();
  return 0;
}
