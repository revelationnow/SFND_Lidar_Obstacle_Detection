// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr cloud_cropped (new pcl::PointCloud<PointT>);
    typename pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    pcl::CropBox<PointT> cropBox(true);
    cropBox.setMax(maxPoint);
    cropBox.setMin(minPoint);
    cropBox.setInputCloud(cloud_filtered);
    cropBox.filter(*cloud_cropped);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_cropped;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
  typename pcl::PointCloud<PointT>::Ptr cloud_p (new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);

  pcl::ExtractIndices<PointT> extract;

  for(auto inlier_id : inliers->indices)
  {
    cloud_p->points.push_back(cloud->points[inlier_id]);
  }

  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_f);

  segResult.first = cloud_f;
  segResult.second = cloud_p;

  return segResult;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::ExtractInliers(typename pcl::PointCloud<PointT>::Ptr cloud, std::unordered_set<int> inliers, bool negative )
{
  typename pcl::PointCloud<PointT>::Ptr result (new pcl::PointCloud<PointT>);

  for(size_t point_ind = 0; point_ind < cloud->size(); point_ind++)
  {
    if ( (inliers.count(point_ind) == 0) == negative)
    {
      result->push_back(cloud->points[point_ind]);
    }
  }
  return result;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneCustom(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // TODO: Fill in this function
  // For max iterations
  for(size_t i = 0; i < maxIterations; i++)
  {
    std::unordered_set<int> inliersResultTemp;

    // Randomly sample subset and fit plane
    PointT p1 = *select_randomly(cloud->begin(), cloud->end());
    PointT p2 = *select_randomly(cloud->begin(), cloud->end());
    PointT p3 = *select_randomly(cloud->begin(), cloud->end());

    int A = ((p1.y - p2.y) * (p3.z - p1.z)) - ((p2.z - p1.z) * (p3.y - p1.y));
    int B = ((p2.z - p1.z) * (p3.x - p1.x)) - ((p2.x - p1.x) * (p3.z - p1.z));
    int C = ((p2.x - p1.x) * (p3.y - p1.y)) - ((p2.y - p1.y) * (p3.x - p1.y));
    int D = -1 * (A * p1.x + B * p1.y + C * p1.z);

    // Measure distance between every point and fitted plane
    // If distance is smaller than threshold count it as inlier
    for(size_t point_ind = 0; point_ind < cloud->size(); point_ind++)
    {
      double dist = fabs((A * cloud->at(point_ind).x) + (B * cloud->at(point_ind).y) + C * cloud->at(point_ind).z + D)/sqrt(A*A +B*B + C*C);

      if(dist <= distanceThreshold)
      {
        inliersResultTemp.insert(point_ind);
      }
    }
    if(inliersResultTemp.size() > inliersResult.size())
    {
      inliersResult = inliersResultTemp;
    }
  }
  // Construct pair of point clouds for points in plane and points outside and return that
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
  segResult.second = ExtractInliers(cloud, inliersResult, false);
  segResult.first  = ExtractInliers(cloud, inliersResult, true);

  return segResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  // TODO:: Fill in this function to find inliers for the cloud.
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (maxIterations);
  seg.setDistanceThreshold (distanceThreshold);
  seg.setInputCloud(cloud);

  seg.segment(*inliers, *coefficients);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
  return segResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringCustom(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();
  std::unique_ptr<KdTree<PointT>> kdtree (new KdTree<PointT>(3));
  for(size_t point_ind = 0; point_ind < cloud->size(); point_ind++)
  {
    kdtree->insert(cloud->points[point_ind],point_ind);
  }

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  typename pcl::PointCloud<PointT>::Ptr current_cloud (new pcl::PointCloud<PointT>);

  std::vector<bool> processed;
  std::vector<bool> point_in_queue;
  std::queue<int> points_to_process;
  int cluster_id = 0;
  for(size_t i = 0; i < cloud->size(); i++)
  {
    processed.push_back(false);
    point_in_queue.push_back(false);
  }
  points_to_process.push(0);
  point_in_queue[0] = true;

  while(!points_to_process.empty())
  {
    int curr_point = points_to_process.front();
    points_to_process.pop();
    point_in_queue[curr_point] = false;
    processed[curr_point] = true;
    current_cloud->push_back(cloud->points[curr_point]);

    auto neighbours = kdtree->search(cloud->points[curr_point], clusterTolerance);
    for(auto it:neighbours)
    {
      if((false == processed[it]) && (false == point_in_queue[it]))
      {
        points_to_process.push(it);
        point_in_queue[it] = true;
      }
    }
    if(points_to_process.empty())
    {
      if(current_cloud->size() >= minSize && current_cloud->size() <= maxSize)
      {
        clusters.push_back(current_cloud);
        current_cloud.reset(new pcl::PointCloud<PointT>);
        cluster_id++;
      }
      else
      {
        current_cloud->clear();
      }
      for(size_t i = 0; i < processed.size(); i++)
      {
        if(processed[i] == false)
        {
          points_to_process.push(i);
          break;
        }
      }
    }
  }
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
#if 0
  for(auto cluster : clusters)
  {
    std::cout<<"Cluster size : "<<cluster->size()<<" Point : ";
    for(auto point : cluster->points)
    {
      std::cout<<point<<",";
    }
    std::cout<<std::endl;

  }
#endif
  return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for(auto cluster_id:cluster_indices)
    {
      typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
      for(auto point:cluster_id.indices)
      {
        cloud_cluster->points.push_back(cloud->points[point]);
      }
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      clusters.push_back(cloud_cluster);

    }



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
