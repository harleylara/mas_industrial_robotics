
#include <pcl/filters/passthrough.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/random_sample.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/local_maximum.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <random>


#include <mir_object_segmentation/crate_detection.h>
CrateDetection::CrateDetection() : filter_min(0.0), filter_max(0.22), crate_height(0.22), cosine_sim(0.95) // set defaults
{
}

CrateDetection::~CrateDetection()
{
}

BoundingBox CrateDetection::detectCrate(const PointCloud::Ptr &full_cloud, PointCloud::Ptr &segmented_cloud)
{
     // 1. Do some filtering / segmentation

    // remove the ground
    pcl::PassThrough<PointT> passthrough_filter;
    PointCloud::Ptr filtered_cloud(new PointCloud);
    passthrough_filter.setInputCloud(full_cloud);
    passthrough_filter.setFilterFieldName("z");
    passthrough_filter.setFilterLimits(filter_min, filter_max);
    passthrough_filter.filter(*filtered_cloud);

    // get the front of the crate (approximately, doesn't matter if it's not complete)
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*filtered_cloud, min_pt, max_pt);

    passthrough_filter.setInputCloud(filtered_cloud);
    passthrough_filter.setFilterFieldName("x");
    passthrough_filter.setFilterLimits(min_pt.x - 0.05, min_pt.x + 0.2);
    passthrough_filter.filter(*filtered_cloud);

    // get top of the crate
    pcl::getMinMax3D(*filtered_cloud, min_pt, max_pt);

    passthrough_filter.setInputCloud(full_cloud);
    passthrough_filter.setFilterFieldName("z");
    passthrough_filter.setFilterLimits(max_pt.z - 0.02, max_pt.z);
    passthrough_filter.filter(*filtered_cloud);

    std::cout<<filtered_cloud->points.size()<<std::endl;

    // segment crate from the wall
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (filtered_cloud);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (1000);
    ec.setMaxClusterSize (100000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (filtered_cloud);
    ec.extract (clusters);

    
    std::cout<<"Created clusters"<<std::endl;

    int max_indices = 0;
    int max_indices_size_id = 0;
    int counter = 0;
    for(auto &p: clusters)
    {
        if(p.indices.size()>max_indices)
        {
            max_indices = p.indices.size();
            max_indices_size_id = counter;
        }
        counter++;
        std::cout<<p.indices.size()<<std::endl;
    }
    
    //clusters[0].indices.size ();

    std::cout<<"The biggest cluster id: "<<max_indices_size_id<<std::endl;

    pcl::ExtractIndices<PointT> extract_cluster;
    extract_cluster.setInputCloud(filtered_cloud);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);

    *indices = clusters[max_indices_size_id];

    std::cout<<indices->indices.size()<<std::endl;
    
    extract_cluster.setIndices(indices);
    extract_cluster.setNegative(false);
    extract_cluster.filter(*filtered_cloud);
    std::cout<<filtered_cloud->points.size()<<std::endl;

    // get oriented bounding box of top
    BoundingBox shrunk_box = BoundingBox::create(filtered_cloud, Eigen::Vector3f(0,0,1), 0.8f);
    BoundingBox full_box = BoundingBox::create(filtered_cloud, Eigen::Vector3f(0,0,1), 1.0f);

    // adjust height to known height of crate
    shrunk_box.setMinHeight(max_pt.z - crate_height);
    full_box.setMinHeight(max_pt.z - crate_height);

    // create a pointcloud with the bottom 4 bounding box vertices
    PointCloud::Ptr bbpc(new PointCloud);
    bbpc->width = 5;
    bbpc->height = 1;
    bbpc->points.resize(4);
    int idx = 0;
    for (auto &p: *bbpc)
    {
        p.x = shrunk_box.getVertices()[idx][0];
        p.y = shrunk_box.getVertices()[idx][1];
        p.z = shrunk_box.getVertices()[idx][2];
        idx += 1;
    }
    bbpc->points.push_back(bbpc->points[0]);

    float min_z = shrunk_box.getVertices()[0][2];
    float max_z = shrunk_box.getVertices()[4][2];

    // get all points in bounding box
    pcl::ExtractPolygonalPrismData<PointT> epp;
    pcl::PointIndices::Ptr seg_indices(new pcl::PointIndices);
    epp.setInputPlanarHull(bbpc);
    epp.setInputCloud(full_cloud);
    //epp.setHeightLimits((max_pt.z - crate_height), max_pt.z);
    epp.segment(*seg_indices);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(full_cloud);
    extract.setIndices(seg_indices);
    extract.setNegative(false);
    extract.filter(*filtered_cloud);

    // apply passthrough filter again so we don't get anything above the box
    passthrough_filter.setInputCloud(filtered_cloud);
    passthrough_filter.setFilterFieldName("z");
    passthrough_filter.setFilterLimits(min_z, max_z);
    passthrough_filter.filter(*filtered_cloud);

    // 3. set the segmented cloud
    segmented_cloud = filtered_cloud;
    return full_box;
}

void CrateDetection::findObjects(const PointCloud::Ptr &filtered_cloud, std::vector<PointCloud::Ptr> &clusters, std::vector<BoundingBox> &boxes)
{
  pcl::search::Search<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  PointCloudN::Ptr normals (new PointCloudN);
  pcl::NormalEstimation<PointT, PointNT> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (filtered_cloud);
  //normal_estimator.setKSearch (50);
  normal_estimator.setRadiusSearch(0.01);
  normal_estimator.compute (*normals);

  PointCloud::Ptr clus(new PointCloud);
  for (int i = 0; i < normals->size(); i++)
  {
    Eigen::Vector4f norm;
    norm[0] = normals->points[i].normal_x;
    norm[1] = normals->points[i].normal_y;
    norm[2] = normals->points[i].normal_z;
    norm[3] = 0.f;
    Eigen::Vector4f axis;
    axis[0] = 0.0;
    axis[1] = 0.0;
    axis[2] = 1.0;
    axis[3] = 0.0;
    // only keep points with a normal close to z-axis (1.0 = cos(0) = perfectly aligned)
    if (std::abs(axis.dot(norm)) > this->cosine_sim)
    {
        clus->points.push_back(filtered_cloud->points[i]);
    }
  }
  clus->width = clus->points.size();
  clus->height = 1;

  pcl::EuclideanClusterExtraction<PointT> cluster_extraction;
  std::vector<pcl::PointIndices> clusters_indices;
  cluster_extraction.setClusterTolerance(0.02);
  cluster_extraction.setMinClusterSize(10);
  cluster_extraction.setMaxClusterSize(100000);
  cluster_extraction.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT>>());
  cluster_extraction.setInputCloud(clus);
  cluster_extraction.extract(clusters_indices);

  for (size_t i = 0; i < clusters_indices.size(); i++) {
      const pcl::PointIndices &clus_indices = clusters_indices[i];
      PointCloud::Ptr cluster(new PointCloud);
      pcl::copyPointCloud(*clus, clus_indices, *cluster);
      BoundingBox box = BoundingBox::create(cluster, Eigen::Vector3f(0,0,1), 1.0f);
      boxes.push_back(box);
      clusters.push_back(cluster);
  }
}
