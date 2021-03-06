/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <queue>
#include <string>
#include "kdtree.h"

#define CLUSTER_POINT_DIM 2
  struct ClusterPoints{
    float data[CLUSTER_POINT_DIM];

    ClusterPoints(float a, float b)
    {
      data[0] = a;
      data[1] = b;
    }
  };
// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
    viewer->addCoordinateSystem (1.0);

    viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, 0, 0, 1, 1, 1, "window");
    return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<ClusterPoints> points)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    for(int i = 0; i < points.size(); i++)
    {
      pcl::PointXYZ point;
      point.x = points[i].data[0];
      point.y = points[i].data[1];
      point.z = 0;

      cloud->points.push_back(point);

    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;

}


void render2DTree(std::unique_ptr<Node<ClusterPoints>> &node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

  if(node!=NULL)
  {
    Box upperWindow = window;
    Box lowerWindow = window;
    // split on x axis
    if(depth%2==0)
    {
      viewer->addLine(pcl::PointXYZ(node->point.data[0], window.y_min, 0),pcl::PointXYZ(node->point.data[0], window.y_max, 0),0,0,1,"line"+std::to_string(iteration));
      lowerWindow.x_max = node->point.data[0];
      upperWindow.x_min = node->point.data[0];
    }
    // split on y axis
    else
    {
      viewer->addLine(pcl::PointXYZ(window.x_min, node->point.data[1], 0),pcl::PointXYZ(window.x_max, node->point.data[1], 0),1,0,0,"line"+std::to_string(iteration));
      lowerWindow.y_max = node->point.data[1];
      upperWindow.y_min = node->point.data[1];
    }
    iteration++;

    render2DTree(node->left,viewer, lowerWindow, iteration, depth+1);
    render2DTree(node->right,viewer, upperWindow, iteration, depth+1);


  }

}



std::vector<std::vector<int>> euclideanCluster(const std::vector<ClusterPoints>& points, KdTree<ClusterPoints>* tree, float distanceTol)
{

  // TODO: Fill out this function to return list of indices for each cluster

  std::vector<std::vector<int>> clusters;
  std::vector<int> curr_cluster;
  std::vector<bool> processed;
  std::vector<bool> point_in_queue;
  std::queue<int> points_to_process;
  int cluster_id = 0;
  for(size_t i = 0; i < points.size(); i++)
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
    curr_cluster.push_back(curr_point);
    std::cerr<<"Processing : "<<curr_point<<std::endl;

    auto neighbours = tree->search(points[curr_point], distanceTol);
    std::cerr<<"Neighbours of "<<curr_point<<" : ";
    for(auto it:neighbours)
    {
      if((false == processed[it]) && (false == point_in_queue[it]))
      {
        points_to_process.push(it);
        point_in_queue[it] = true;
        std::cerr<<it<<", ";
      }
    }
    std::cerr<<std::endl;

    if(points_to_process.empty())
    {
      std::cerr<<"Points in cluster "<<cluster_id<<" : ";
      for(auto it:curr_cluster)
      {
        std::cerr<<it<<", ";
      }
      std::cerr<<std::endl;
      clusters.push_back(curr_cluster);
      curr_cluster.clear();
      for(size_t i = 0; i < processed.size(); i++)
      {
        if(processed[i] == false)
        {
          points_to_process.push(i);
          break;
        }
      }
      cluster_id++;
      std::cerr<<"New Cluster "<<cluster_id<<std::endl;
    }
  }

  return clusters;

}

int main ()
{

  // Create viewer
  Box window;
  window.x_min = -10;
  window.x_max =  10;
  window.y_min = -10;
  window.y_max =  10;
  window.z_min =   0;
  window.z_max =   0;
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);



  // Create data
  std::vector<ClusterPoints> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
  //std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);

  KdTree<ClusterPoints>* tree = new KdTree<ClusterPoints>(CLUSTER_POINT_DIM);

  for (int i=0; i<points.size(); i++)
    tree->insert(points[i],i);

  int it = 0;
  render2DTree(tree->root,viewer,window, it);

  std::cout << "Test Search" << std::endl;
  std::vector<int> nearby = tree->search({-6,7},3.0);
  for(int index : nearby)
    std::cout << index << ",";
  std::cout << std::endl;

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  //
  std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 3.0);
  //
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  // Render clusters
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  for(std::vector<int> cluster : clusters)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
    for(int indice: cluster)
      clusterCloud->points.push_back(pcl::PointXYZ(points[indice].data[0],points[indice].data[1],0));
    renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
    ++clusterId;
  }
  if(clusters.size()==0)
    renderPointCloud(viewer,cloud,"data");

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce ();
  }

}
