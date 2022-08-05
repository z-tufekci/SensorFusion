
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
		}
		else
		{
			std::vector<float> data = (*node)->point;
			const float coord = point[depth % 3];
	
			//compare X, Y, or Z values
			if (coord < data[depth % 3])
				insertHelper(&((*node)->left), point, depth + 1, id);
			else
				insertHelper(&((*node)->right), point, depth + 1, id);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		Node *node = root;
		searchHelper(root, target, distanceTol, 0, ids);
		return ids;
	} // end of search

	void searchHelper(Node *node, std::vector<float> target, float distanceTol, int layer, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			std::vector<float> node_point = node->point;
			const float coord = node_point[layer % 3];
			const float hedef = target[layer % 3];
			if (coord < (hedef - distanceTol))
			{
				searchHelper(node->right, target, distanceTol, layer + 1, ids);
			}
			else if (coord > (hedef + distanceTol))
			{
				searchHelper(node->left, target, distanceTol, layer + 1, ids);
			}
			else
			{
				//in the box, calculate distance.
				float a0 = std::pow(node_point[0] - target[0], 2);
				float a1 = std::pow(node_point[1] - target[1], 2);
				float a2 = std::pow(node_point[2] - target[2], 2);
				float dist = std::sqrt(a0 + a1+ a2);
				
				if (dist <= distanceTol)
					ids.push_back(node->id);

				searchHelper(node->left, target, distanceTol, layer + 1, ids);
				searchHelper(node->right, target, distanceTol, layer + 1, ids);
			}			
		}
	}
};