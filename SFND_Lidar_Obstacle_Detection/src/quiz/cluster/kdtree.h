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
    uint depth;
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

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		if(*node==NULL)
		{
			*node = new Node (point, id);
		} 
		else
		{
			uint xIsZeroYisOne = depth % 2;
			
			/*for KD tree algorithm the first element to compare is x,then y*/
			if(point[xIsZeroYisOne] < ((*node)->point[xIsZeroYisOne]))
			{
				insertHelper(&((*node)->left), depth+1, point, id);
			}
			else
			{
				insertHelper(&((*node)->right), depth+1, point, id);
			}
			
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
		
		/*if(root==NULL)
		{
			root = new Node (point, id);
		} 
		else
		{
			uint xIsZeroYisOne = root->depth % 2;
			
			//for KD tree algorithm the first element to compare is x,then y
			if(point[xIsZeroYisOne] < (root->point[xIsZeroYisOne]))
			{
				root->left = new Node (point, id);
				
			}
			else
			{
				root->right = new Node (point, id);
			}
			root->depth++;
			
		}*/
	}

	void searchHelperRecursive(std::vector<float> targetPoint, Node* node, int depth, float distanceTol, std::vector<int>& ids)
    {
    	if (node != NULL)
        {
        	if (
				   (node->point[0] >= (targetPoint[0]-distanceTol) && (node->point[0] <= (targetPoint[0]+distanceTol)))
                && (node->point[1] >= (targetPoint[1]-distanceTol) && (node->point[1] <= (targetPoint[1]+distanceTol))) 
				//&& (node->point[2] >= (targetPoint[2]-distanceTol) && (node->point[2] <= (targetPoint[2]+distanceTol)))
			   )
           	{
				float xDIstanceElement = (node->point[0]-targetPoint[0])*(node->point[0]-targetPoint[0]);
				float yDIstanceElement = (node->point[1]-targetPoint[1])*(node->point[1]-targetPoint[1]);
				float zDIstanceElement = 0;//(node->point[2]-targetPoint[2])*(node->point[2]-targetPoint[2]);

              	float distance = sqrt(xDIstanceElement + yDIstanceElement + zDIstanceElement);
              
              	if (distance <= distanceTol)
                	ids.push_back(node->id);
            }
          
			uint xIsZeroYisOne = depth % 2;

          	if ((targetPoint[xIsZeroYisOne]-distanceTol)<node->point[xIsZeroYisOne])
            	searchHelperRecursive(targetPoint, node->left, depth+1, distanceTol, ids);

          	if ((targetPoint[xIsZeroYisOne]+distanceTol)>node->point[xIsZeroYisOne])
            	searchHelperRecursive(targetPoint, node->right, depth+1, distanceTol, ids);
        }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelperRecursive(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




