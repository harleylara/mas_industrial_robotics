/*
 * Copyright 2016 Bonn-Rhein-Sieg University
 *
 * Author: Sergey Alexandrov
 *
 */

#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>
//#include <opencv/cv.h>
#include <algorithm>
#include <limits>

#include <pcl/common/transforms.h>

#include <mir_object_segmentation/crate_detection.h>


// Taken from https://github.com/b-it-bots/mas_industrial_robotics/blob/melodic/mir_perception/mir_perception_utils/common/src/bounding_box.cpp
BoundingBox BoundingBox::create(const PointCloud::ConstPtr &cloud, const Eigen::Vector3f &normal, float shrink_to_percentage = 1.0f)
{
  BoundingBox box;

  /*
  // Step 1: transform the cloud so that z-axis as aligned with plane normal.
  Eigen::Vector3f perpendicular(-normal[1], normal[0], normal[2]);
  Eigen::Affine3f transform = pcl::getTransFromUnitVectorsZY(normal, perpendicular);
  Eigen::Affine3f inverse_transform = transform.inverse(Eigen::Isometry);
  PointCloud cloud_transformed;
  pcl::transformPointCloud(*cloud, cloud_transformed, transform);
  */

  // Step 2: project cloud onto the plane and calculate bounding box for the
  // projected points.
  // Have no idea how this "mem storage" and "seq" beasts work.
  // Initialization example taken from:
  // http://opencv.willowgarage.com/documentation/dynamic_structures.html#seqsort
  CvMemStorage *storage = cvCreateMemStorage(0);
  CvSeq *points = cvCreateSeq(CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage);

  // CvPoints are made of integers, so we will need to scale our points (which
  // are in meters).
  const float SCALE = 1000.0;
  float min_z = std::numeric_limits<float>::max();
  float max_z = -1 * std::numeric_limits<float>::max();

  for (size_t i = 0; i < cloud->points.size(); i++) {
    const PointT &pt = cloud->points[i];
    if (!std::isnan(pt.z)) {
      CvPoint p;
      p.x = pt.x * SCALE;
      p.y = pt.y * SCALE;
      cvSeqPush(points, &p);
      if (pt.z > max_z) max_z = pt.z;
      if (pt.z < min_z) min_z = pt.z;
    }
  }

  CvBox2D box2d = cvMinAreaRect2(points);
  cvReleaseMemStorage(&storage);
  box2d.size.width = box2d.size.width * shrink_to_percentage;
  box2d.size.height = box2d.size.height * shrink_to_percentage;
  box.dimensions_[0] = max_z - min_z;
  box.dimensions_[1] = std::max(box2d.size.width, box2d.size.height) / SCALE;
  box.dimensions_[2] = std::min(box2d.size.width, box2d.size.height) / SCALE;
  box.center_[0] = box2d.center.x / SCALE;
  box.center_[1] = box2d.center.y / SCALE;
  box.center_[2] = min_z + box.dimensions_[0] / 2.0;
  //box.center_ = inverse_transform * box.center_;

  cv::Point2f vertices[4];
  cv::RotatedRect(box2d).points(vertices);
  for (size_t i = 0; i < 4; i++) {
    const cv::Point2f &pt = vertices[i];
    Eigen::Vector3f p;
    p << pt.x / SCALE, pt.y / SCALE, min_z;
    //box.vertices_.push_back(inverse_transform * p);
    box.vertices_.push_back(p);
  }
  for (size_t i = 0; i < 4; i++) {
    const cv::Point2f &pt = vertices[i];
    Eigen::Vector3f p;
    p << pt.x / SCALE, pt.y / SCALE, max_z;
    //box.vertices_.push_back(inverse_transform * p);
    box.vertices_.push_back(p);
  }
  return box;
}

void BoundingBox::setMinHeight(double z_min)
{
    double z_max = vertices_[4][2];
    for (int i = 0; i < 4; i++)
    {
        vertices_[i][2] = z_min;
    }
    dimensions_[0] = z_max - z_min;
    center_[2] = z_min + dimensions_[0] / 2.0;
}

void BoundingBox::getMinMax3D(PointT &min_pt, PointT &max_pt)
{
    for (int i = 0; i < 4; i++)
    {
        if (vertices_[i][0] < min_pt.x)
        {
            min_pt.x = vertices_[i][0];
        }
        if (vertices_[i][0] > max_pt.x)
        {
            max_pt.x = vertices_[i][0];
        }
        if (vertices_[i][1] < min_pt.y)
        {
            min_pt.y = vertices_[i][1];
        }
        if (vertices_[i][1] > max_pt.y)
        {
            max_pt.y = vertices_[i][1];
        }
    }
    if (vertices_[0][2] < vertices_[4][2])
    {
        min_pt.z = vertices_[0][2];
        max_pt.z = vertices_[4][2];
    }
    else
    {
        min_pt.z = vertices_[4][2];
        max_pt.z = vertices_[0][2];
    }
}

float BoundingBox::getAngle()
{

    float x1 = vertices_[0][0];
    float x2 = vertices_[1][0];
    float y1 = vertices_[0][1];
    float y2 = vertices_[1][1];
    float angle = std::atan2((y2-y1), (x2-x1));
    return angle;
}

/*
void BoundingBox::shrink(float percentage)
{
    float startx = vertices_[0][0];
    float starty = vertices_[0][1];
    float endx = vertices_[1][0];
    float endy = vertices_[1][1];

    float new_start_x = startx + percentage * (endx - startx)
    float new_end_x = startx + (1.0 - percentage) * (endx - startx)
    float new_start_y = starty + percentage * (endy - starty)
    float new_end_y = starty + (1.0 - percentage) * (endy - starty)


}
*/
