/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root, point, 0, id);
	}

	void insertHelper(Node **node, std::vector<float> point, int depth, int id)
	{
		if (*node == NULL)
		{
			// create new Node
			*node = new Node(point, id);
			std::cout << (*node)->point[0] << " " << (*node)->point[1] << " " << (*node)->point[2] << "\n";
		}
		else
		{
			std::vector<float> data = (*node)->point;
			if (depth % 3 == 0)
			{
				//compare X values
				float x = point[0];
				depth++;
				if (x < data[0])
					insertHelper(&((*node)->left), point, depth, id);
				else
					insertHelper(&((*node)->right), point, depth, id);
			}
			else if (depth % 3 == 1)
			{
				//compate Y values
				float y = point[1];
				depth++;
				if (y < data[1])
					insertHelper(&((*node)->left), point, depth, id);
				else
					insertHelper(&((*node)->right), point, depth, id);
			}
			else if (depth % 3 == 2)
			{
				//compate Y values
				float z = point[2];
				depth++;
				if (z < data[2])
					insertHelper(&((*node)->left), point, depth, id);
				else
					insertHelper(&((*node)->right), point, depth, id);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		//  {-6,7}, dist= 3
		Node *node = root;
		int layer = 0;
		//bool isX = true;
		searchHelper(root, target, distanceTol, layer, ids);
		return ids;
	} // end of search

	void searchHelper(Node *node, std::vector<float> target, float distanceTol, int layer, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			std::vector<float> node_point = node->point;
			if (layer % 3 == 0)
			{
				//std::cout << "Heyo:" << (node->id) << " , ";
				//std::cout << "Node point:" << (node_point[0]) << " , " << node_point[1] << "\n";
				layer++;

				if (node_point[0] < (target[0] - distanceTol))
				{
					searchHelper(node->right, target, distanceTol, layer, ids);
				}
				else if (node_point[0] > (target[0] + distanceTol))
				{
					searchHelper(node->left, target, distanceTol, layer, ids);
				}
				else
				{
					//in the box
					// calculate distance.
					float a0 = std::pow(node_point[0] - target[0], 2);
					float a1 = std::pow(node_point[1] - target[1], 2);
					float a2 = std::pow(node_point[2] - target[2], 2);

					float dist = std::sqrt(a0 + a1+ a2);
					
					if (dist <= distanceTol)
					{
						//std::cout << "in the box and distance -> " << dist;
						//std::cout << "Yess" << (node->id) << " , \n";
						ids.push_back(node->id);
					}
					
					searchHelper(node->left, target, distanceTol, layer, ids);
					searchHelper(node->right, target, distanceTol, layer, ids);
				}
			}
			else if(layer % 3 == 1)
			{ // degilse Y ye bak

				layer++;

				if (node_point[1] < (target[1] - distanceTol))
				{
					searchHelper(node->right, target, distanceTol, layer, ids);
				}
				else if (node_point[1] > (target[1] + distanceTol))
				{
					searchHelper(node->left, target, distanceTol, layer, ids);
				}
				else
				{
					float a0 = std::pow(node_point[0] - target[0], 2);
					float a1 = std::pow(node_point[1] - target[1], 2);
					float a2 = std::pow(node_point[2] - target[2], 2);
					float dist = std::sqrt(a0 + a1+ a2);
					if (dist <= distanceTol)
					{
						//std::cout <<"yyy in the box"<< (node->id) << " , \n ";
						ids.push_back(node->id);
					}
					
					searchHelper(node->left, target, distanceTol, layer, ids);
					searchHelper(node->right, target, distanceTol, layer, ids);
				}
			}else if(layer % 3 == 2){
				
				layer++;

				if (node_point[2] < (target[2] - distanceTol))
				{
					searchHelper(node->right, target, distanceTol, layer, ids);
				}
				else if (node_point[2] > (target[2] + distanceTol))
				{
					searchHelper(node->left, target, distanceTol, layer, ids);
				}
				else
				{
					float a0 = std::pow(node_point[0] - target[0], 2);
					float a1 = std::pow(node_point[1] - target[1], 2);
					float a2 = std::pow(node_point[2] - target[2], 2);
					float dist = std::sqrt(a0 + a1 + a2);
					if (dist <= distanceTol)
					{
						//std::cout <<"yyy in the box"<< (node->id) << " , \n ";
						ids.push_back(node->id);
					}
					
					searchHelper(node->left, target, distanceTol, layer, ids);
					searchHelper(node->right, target, distanceTol, layer, ids);
			}
		}
	}
};
