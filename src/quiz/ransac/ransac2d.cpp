/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>
#include  <iterator>


pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for(int i = -5; i < 5; i++)
    {
      double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
      double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
      pcl::PointXYZ point;
      point.x = i+scatter*rx;
      point.y = i+scatter*ry;
      point.z = 0;

      cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while(numOutliers--)
    {
      double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
      double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
      pcl::PointXYZ point;
      point.x = 5*rx;
      point.y = 5*ry;
      point.z = 0;

      cloud->points.push_back(point);

    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem (1.0);
    return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // TODO: Fill in this function
  // For max iterations
  for(size_t i = 0; i < maxIterations; i++)
  {
    std::unordered_set<int> inliersResultTemp;

    pcl::PointXYZ p1 = *select_randomly(cloud->begin(), cloud->end());
    pcl::PointXYZ p2 = *select_randomly(cloud->begin(), cloud->end());
    pcl::PointXYZ p3 = *select_randomly(cloud->begin(), cloud->end());

    int A = (p1.y - p2.y);
    int B = (p2.x - p1.x);
    int C = (p1.x * p2.y) - (p2.x * p1.y);

    for(size_t point_ind = 0; point_ind < cloud->size(); point_ind++)
    {
      double dist = fabs((A * cloud->at(point_ind).x) + (B * cloud->at(point_ind).y) + C )/sqrt(A*A +B*B);

      if(dist <= distanceTol)
      {
        inliersResultTemp.insert(point_ind);
      }
    }
    if(inliersResultTemp.size() > inliersResult.size())
    {
      inliersResult = inliersResultTemp;
    }

    // Randomly sample subset and fit line


  // Measure distance between every point and fitted line
  // If distance is smaller than threshold count it as inlier

  }
  // Return indicies of inliers from fitted line with most inliers

  return inliersResult;

}
std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // TODO: Fill in this function
  // For max iterations
  for(size_t i = 0; i < maxIterations; i++)
  {
    std::unordered_set<int> inliersResultTemp;

    pcl::PointXYZ p1 = *select_randomly(cloud->begin(), cloud->end());
    pcl::PointXYZ p2 = *select_randomly(cloud->begin(), cloud->end());
    pcl::PointXYZ p3 = *select_randomly(cloud->begin(), cloud->end());

    int A = ((p1.y - p2.y) * (p3.z - p1.z)) - ((p2.z - p1.z) * (p3.y - p1.y));
    int B = ((p2.z - p1.z) * (p3.x - p1.x)) - ((p2.x - p1.x) * (p3.z - p1.z));
    int C = ((p2.x - p1.x) * (p3.y - p1.y)) - ((p2.y - p1.y) * (p3.x - p1.y));
    int D = -1 * (A * p1.x + B * p1.y + C * p1.z);

    for(size_t point_ind = 0; point_ind < cloud->size(); point_ind++)
    {
      double dist = fabs((A * cloud->at(point_ind).x) + (B * cloud->at(point_ind).y) + C * cloud->at(point_ind).z + D)/sqrt(A*A +B*B + C*C);

      if(dist <= distanceTol)
      {
        inliersResultTemp.insert(point_ind);
      }
    }
    if(inliersResultTemp.size() > inliersResult.size())
    {
      inliersResult = inliersResultTemp;
    }

    // Randomly sample subset and fit line


  // Measure distance between every point and fitted line
  // If distance is smaller than threshold count it as inlier

  }
  // Return indicies of inliers from fitted line with most inliers

  return inliersResult;

}

int main ()
{

  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();


  // TODO: Change the max iteration and distance tolerance arguments for Ransac function
  std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

  for(int index = 0; index < cloud->points.size(); index++)
  {
    pcl::PointXYZ point = cloud->points[index];
    if(inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }


  // Render 2D point cloud with inliers and outliers
  if(inliers.size())
  {
    renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
      renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
  }
    else
    {
      renderPointCloud(viewer,cloud,"data");
    }

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce ();
    }

}
