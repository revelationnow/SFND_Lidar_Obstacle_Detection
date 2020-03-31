/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <memory>


// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
  std::unique_ptr<Node<PointT>> left;
  std::unique_ptr<Node<PointT>> right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}

};

template<typename PointT>
struct KdTree
{
  std::unique_ptr<Node<PointT>> root;
  int _dimension;

	KdTree()
	: root(NULL), _dimension(3)
	{}

	KdTree(int dim)
	: root(nullptr), _dimension(dim)
	{}

  void insert_helper(PointT &point, int id, std::unique_ptr<Node<PointT>> &curr_root, int depth = 0)
  {
    if(curr_root == nullptr)
    {
      curr_root.reset(std::move(new Node<PointT>(point, id)));
    }
    else
    {
      if(point.data[depth % _dimension] < curr_root->point.data[depth % _dimension])
      {
        insert_helper(point, id, curr_root->left, depth + 1);
      }
      else
      {
        insert_helper(point, id, curr_root->right, depth + 1);
      }
    }
  }

	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
    insert_helper(point, id, root);

	}

  void search_helper(PointT &target, float distanceTol, std::unique_ptr<Node<PointT>> &curr_root, std::vector<int> &ids, int depth = 0)
  {

    if(NULL == curr_root)
      return;

    float root_dim = curr_root->point.data[depth % _dimension];
    float target_dim = target.data[depth % _dimension];

    bool check_left  = (target_dim <= root_dim) || ((target_dim - distanceTol) <= root_dim);
    bool check_right = (target_dim > root_dim) || ((target_dim + distanceTol) > root_dim);
    if(check_left)
    {
      search_helper(target, distanceTol, curr_root->left, ids, depth + 1);
    }

    if(check_right)
    {
      search_helper(target, distanceTol, curr_root->right, ids, depth + 1);
    }

    float sq_dist_root_targ = 0.0;
    for(size_t i = 0;i < _dimension; i++)
    {
      sq_dist_root_targ += (target.data[i] - curr_root->point.data[i]) * (target.data[i] - curr_root->point.data[i]);
    }

    if(sq_dist_root_targ <= (distanceTol * distanceTol))
    {
      ids.push_back(curr_root->id);
    }
  }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
    search_helper(target, distanceTol, root, ids);
		return ids;
	}

};




