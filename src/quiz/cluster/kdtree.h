/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

  void insert_helper(std::vector<float> &point, int id, Node** curr_root, int depth = 0)
  {
    if(*curr_root == NULL)
    {
      std::cout<<"Found"<<std::endl;
      *curr_root = new Node(point, id);
    }
    else
    {
      if(point[depth % point.size()] < (*curr_root)->point[depth % point.size()])
      {
        std::cout<<"Going left"<<std::endl;
        insert_helper(point, id, &((*curr_root)->left), depth + 1);
      }
      else
      {
        std::cout<<"Going Right"<<std::endl;
        insert_helper(point, id, &((*curr_root)->right), depth + 1);
      }
    }
  }

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
    insert_helper(point, id, &root);

	}

  void search_helper(std::vector<float> target, float distanceTol, Node *curr_root, std::vector<int> &ids, int depth = 0)
  {

    if(NULL == curr_root)
      return;

    float root_dim = curr_root->point[depth % curr_root->point.size()];
    float target_dim = target[depth % curr_root->point.size()];

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
    for(size_t i = 0;i < target.size(); i++)
    {
      sq_dist_root_targ += (target[i] - curr_root->point[i]) * (target[i] - curr_root->point[i]);
    }

    if(sq_dist_root_targ <= (distanceTol * distanceTol))
    {
      ids.push_back(curr_root->id);
    }
  }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
    search_helper(target, distanceTol, root, ids);
		return ids;
	}
	

};




