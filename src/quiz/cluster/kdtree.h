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

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




