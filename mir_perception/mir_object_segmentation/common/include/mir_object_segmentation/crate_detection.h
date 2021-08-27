#ifndef CRATE_DETECTION_H
#define CRATE_DETECTION_H

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::Normal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudN;

// Taken from https://github.com/b-it-bots/mas_industrial_robotics/blob/melodic/mir_perception/mir_perception_utils/common/include/mir_perception_utils/bounding_box.h
class BoundingBox
{
 public:
  typedef Eigen::Vector3f Point;
  typedef std::vector<Point, Eigen::aligned_allocator<Point>> Points;

  inline const Point &getCenter() const { return center_; }
  inline const Points &getVertices() const { return vertices_; }
  inline Eigen::Vector3f getDimensions() const { return dimensions_; }
  inline float getVolume() const { return dimensions_[0] * dimensions_[1] * dimensions_[2]; }
  /** \brief Create a bounding box around the cloud, restricting it to be
   * parallel to the plane defined by the normal.
   * \param[in] Point cloud
   * \param[in] Normal
   * */
  static BoundingBox create(const typename PointCloud::ConstPtr &cloud,
                            const Eigen::Vector3f &normal, float shrink_to_percentage);

  /** \brief Create a bounding box around the point vector, restricting it to be
   * parallel to the plane defined by the normal.
   * \param[in] Point vector
   * \param[in] Normal
  */
  static BoundingBox create(const typename PointCloud::VectorType &points,
                            const Eigen::Vector3f &normal);

  void getMinMax3D(PointT &min_pt, PointT &max_pt);

  void setMinHeight(double z_min);
  float getAngle();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  Point center_;
  Points vertices_;
  Eigen::Vector3f dimensions_;
};


class CrateDetection
{
private:
    /*
     * Returns bounding box of detected box
     *
     * full_cloud: input pointcloud
     * bounding_box: oriented bounding box of detected box; contains a list of vertices of the bounding box,
     *               the order of which is the bottom four vertices (clockwise) followed by the top
     *               four vertices (in the same order as the bottom)
     */

    double filter_min, filter_max, crate_height;
    float cosine_sim;
    int K;

public:
    CrateDetection();
    virtual ~CrateDetection();
    BoundingBox detectCrate(const PointCloud::Ptr &full_cloud, PointCloud::Ptr &segmented_cloud);
    void findObjects(const PointCloud::Ptr &filtered_cloud, std::vector<PointCloud::Ptr> &clusters, std::vector<BoundingBox> &boxes);

    void setFilterMin(double filter_min)
    {
        this->filter_min = filter_min;
    }
    void setFilterMax(double filter_max)
    {
        this->filter_max = filter_max;
    }

    void setCrateHeight(double crate_height)
    {
        this->crate_height = crate_height;
    }
    void setCosineSim(float cosine_sim)
    {
        this->cosine_sim = cosine_sim;
    }
    void setK(int K)
    {
        this->K = K;
    }
};


#endif /* CRATE_DETECTION_H */

