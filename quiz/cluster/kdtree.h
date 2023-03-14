/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <cmath>



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

	// Define helper fnc to insert
	void insertHelper(Node** node, uint depth, std::vector<float> point, int id )
	{
		if( *node ==NULL)
			*node =  new Node(point, id);
		else
		{
			uint dims = depth %2; 

			if(point[dims] < (*node)->point[dims])
				insertHelper(&((*node)->left), depth +1, point, id);
			else
				insertHelper(&((*node)->right), depth +1, point, id);

		}
						
	}
	// insert new node:
	void insert(std::vector<float> point, int id)
	{ 
		insertHelper(&root, 0, point, id);
	}

	// helper fnc for seach:

	void searchHelper(Node* node, std::vector<float>target, float distanceTol, uint depth,std::vector<int>& ids)
	{
		if (node != NULL)
		{ 
			//if ((abs(node->point[0]-target[0])<= distanceTol) && (abs(node->point[1]-target[1])<=distanceTol))
			if ( (node->point[0]>=(target[0]-distanceTol)&& node->point[0]<=(target[0]+distanceTol))&&(node->point[1]>=(target[1]-distanceTol)&& node->point[1]<=(target[1]+distanceTol)))
			{
				//float distance = sqrt(pow(node->point[0]-target[0],2) + pow((node->point[1]-target[1]),2));
				float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0]) + (node->point[1]-target[1])*(node->point[1]-target[1]));
				if (distance <= distanceTol)
					ids.push_back(node->id);

			}
			if((target[depth%2]-distanceTol)<node->point[depth%2])
				searchHelper(node->left, target, distanceTol, depth+1, ids);
			if((target[depth%2]+distanceTol)>node->point[depth%2])
				searchHelper(node->right, target, distanceTol, depth+1, ids); 

		}


	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root, target, distanceTol, 0, ids);
		return ids;
	}
	
};




