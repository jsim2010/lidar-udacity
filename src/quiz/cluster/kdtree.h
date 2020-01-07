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

	void insert(std::vector<float> point, int id)
	{
		//Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        insert(&root, 0, point, id);
	}

    void insert(Node** node, int depth, std::vector<float> point, int id) {
        if (*node == NULL) {
            *node = new Node(point, id);
        } else {
            int point_index = depth % point.size();

            if (point[point_index] < (*node)->point[point_index]) {
                insert(&(*node)->left, depth + 1, point, id);
            } else {
                insert(&(*node)->right, depth + 1, point, id);
            }
        }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        recursive_search(target, distanceTol, root, 0, ids);
		return ids;
	}

    void recursive_search(std::vector<float> target, float distance, Node* node, int depth, std::vector<int>& ids) {
        if (node != NULL) {
            float node_0 = node->point[0];
            float target_0 = target[0];
            float node_1 = node->point[1];
            float target_1 = target[1];

            if (node_0 >= target_0 - distance && node_0 <= target_0 + distance && node_1 >= target_1 - distance && node_1 <= target_1 + distance) {
                float dist_0 = node_0 - target_0;
                float dist_1 = node_1 - target_1;
                float actual_distance = sqrt((dist_0 * dist_0) + (dist_1 * dist_1));

                if (actual_distance < distance) {
                    ids.push_back(node->id);
                }
            }
            
            int d = depth % 2;
            float node_d = node->point[d];
            float target_d = target[d];

            if (node_d > target_d - distance) {
                recursive_search(target, distance, node->left, depth + 1, ids);
            }
            if (node_d < target_d + distance) {
                recursive_search(target, distance, node->right, depth + 1, ids);
            }
        }
    }
	

};




