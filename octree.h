#ifndef OCTREE_H_INCLUDED
#define OCTREE_H_INCLUDED

#include <iostream>
#include <vector>
#include <cmath>
#include <queue>

enum Node_status { UNUSED, BODY, BODY_END, LEAVES, BLACK, WHITE, MIXED, BODY_BLACK, BODY_WHITE, PIVOT, SURFACE };  //black stands for outside the hull, white stands for inside the hull

struct Onode {
	double x, y, z;
	int generation;
	Node_status status;
	Onode* parent;
	std::vector<Onode*> childs;
	Onode();
	Onode(double xx, double yy, double zz, int g, Node_status ss, Onode* p);
	int num_child()const { return childs.size(); }
	void SetNode(double xx, double yy, double zz, int g, Node_status ss, Onode* p);
};

Onode::Onode() {
	childs.resize(8);
	for (int i = 0; i < 8; i++) {
		childs[i] = NULL;
	}
}

Onode::Onode(double xx, double yy, double zz, int g, Node_status ss, Onode* p) :Onode() {
	SetNode(xx, yy, zz, g, ss, p);
}

void Onode::SetNode(double xx, double yy, double zz, int g, Node_status ss, Onode* p) {
	x = xx;
	y = yy;
	z = zz;
	generation = g;
	status = ss;
	parent = p;
}

class Octree {

	Onode* root_ptr;
	int num_node;

public:

	double x_min, x_max;
	double y_min, y_max;
	double z_min, z_max;

	Octree(int g = 0);
	~Octree();
	Onode* GetRoot()const { return root_ptr; }
	Onode* FindPoint(double xx, double yy, double zz, Onode* node)const;
	Node_status GetStatus(double xx, double yy, double zz, Onode* node_ptr)const;
	int GetSize()const { return num_node; }
	void Print();
	template<typename VST> void TravPost(Onode* node_ptr, VST updater);
	void Expand(Onode* node_ptr, Node_status ss);
	Node_status UpdateStatus(Onode* node_ptr);
	void UpdatePivot(Onode* node_ptr);

};

Onode* Octree::FindPoint(double xx, double yy, double zz, Onode* node)const {
	if (node == NULL || node->status == UNUSED) {
		return NULL;
	}
	double dx = xx - node->x;
	double dy = yy - node->y;
	double dz = zz - node->z;

	if ((abs(dx) < 1e-10) && (abs(dy) < 1e-10) && (abs(dz) < 1e-10)) {
		return node;
	}

	int rank = (0 < dx) * 4 + (0 < dy) * 2 + (0 < dz);
	return FindPoint(xx, yy, zz, node->childs[rank]);

}

Node_status Octree::GetStatus(double xx, double yy, double zz, Onode* node_ptr)const {
	switch (node_ptr->status)
	{
	case(BLACK):case(BODY_BLACK): {	
		return BLACK;
	}
	case(WHITE):case(BODY_WHITE):case(SURFACE): {	
		return WHITE;
	}
	case(PIVOT):case(MIXED): {	
		double dx = xx - node_ptr->x;
		double dy = yy - node_ptr->y;
		double dz = zz - node_ptr->z;
		int rank = (0 < dx) * 4 + (0 < dy) * 2 + (0 < dz);
		return GetStatus(xx, yy, zz, node_ptr->childs[rank]);
	}
	default:
		std::cout << "status of node: " << node_ptr->status << std::endl;	
		break;
	}
	return UNUSED;	
}

void Octree::Print() {
	struct Printer {
		void operator()(Onode* onode_ptr)const {
			std::cout << onode_ptr->x << ' ' << onode_ptr->y << ' ' << onode_ptr->z << ' ' << (Node_status)onode_ptr->status << std::endl;
		}
	}printer;
	TravPost(root_ptr, printer);
}

void Octree::Expand(Onode* node_ptr, Node_status ss) { 

	const int x_sign[8] = { -1,-1,-1,-1,1,1,1,1 };
	const int y_sign[8] = { -1,-1,1,1,-1,-1,1,1 };
	const int z_sign[8] = { -1,1,-1,1,-1,1,-1,1 };

	double x_step = (x_max - x_min) / 4 / pow(2, node_ptr->generation);
	double y_step = (y_max - y_min) / 4 / pow(2, node_ptr->generation);
	double z_step = (z_max - z_min) / 4 / pow(2, node_ptr->generation);

	for (int i = 0; i < 8; i++) {

		double xx = node_ptr->x + x_sign[i] * x_step;
		double yy = node_ptr->y + y_sign[i] * y_step;
		double zz = node_ptr->z + z_sign[i] * z_step;
		int g = node_ptr->generation + 1;

		node_ptr->childs[i] = new Onode(xx, yy, zz, g, ss, node_ptr);

	}
	num_node += 8;
}

Octree::Octree(int g) {      

	x_min = -10; x_max = 10;
	y_min = -10; y_max = 10;
	z_min = 15; z_max = 30;

	root_ptr = new Onode((x_min + x_max) / 2, (y_min + y_max) / 2, (z_min + z_max) / 2, 0, BODY, NULL);
	num_node = 1;

	std::queue<Onode*> q_node;
	q_node.push(root_ptr);
	while (!q_node.empty()) {
		Onode* node_ptr = q_node.front(); q_node.pop();

		if (node_ptr->generation < g - 1) {   
			Expand(node_ptr, BODY);	
			for (int i = 0; i < 8; i++) {	
				q_node.push(node_ptr->childs[i]);
			}
		}
		else {	
			Expand(node_ptr, BODY_END);	
		}
	}
}

Octree::~Octree() {
	struct Destructor {
		void operator()(Onode* node_ptr) {
			delete node_ptr;
		}
	}destructor;
	TravPost(root_ptr, destructor);
}

template<typename VST>
void Octree::TravPost(Onode* node_ptr, VST updater) {
	for (int i = 0; i < 8; i++) {
		if (node_ptr->childs[i] && node_ptr->childs[i]->status != UNUSED) {
			TravPost(node_ptr->childs[i], updater);
		}
	}
	updater(node_ptr);
}

Node_status Octree::UpdateStatus(Onode* node_ptr) {
	for (int i = 0; i < 8; i++) {	
		if (node_ptr->childs[i]->status == MIXED || node_ptr->childs[i]->status == PIVOT) {
			return node_ptr->status = PIVOT;
		}
	}
	for (int i = 0; i < 8; i++) {   
		switch (node_ptr->childs[i]->status) {
		case(BLACK):case(BODY_BLACK): {
			switch (node_ptr->status) {
			case(BODY): {
				node_ptr->status = BODY_BLACK;
				break;
			}
			case(BODY_BLACK): {
				break;
			}
			case(BODY_WHITE): {
				if (node_ptr->childs[i]->status == BLACK) {
					return node_ptr->status = MIXED;
				}
				else {
					return node_ptr->status = PIVOT;
				}

			}
			}
			break;
		}
		case(WHITE):case(BODY_WHITE): {
			switch (node_ptr->status) {
			case(BODY): {
				node_ptr->status = BODY_WHITE;
				break;
			}
			case(BODY_BLACK): {
				if (node_ptr->childs[i]->status == WHITE) {
					return node_ptr->status = MIXED;
				}
				else {
					return node_ptr->status = PIVOT;
				}

			}
			case(BODY_WHITE): {
				break;
			}
			}
			break;
		}
		default:
			return UNUSED;
		}
	}
	return node_ptr->status;
}

void Octree::UpdatePivot(Onode* node_ptr) {
	while (node_ptr != root_ptr && node_ptr->status != PIVOT) {
		node_ptr->status = PIVOT;
		node_ptr = node_ptr->parent;
	}
}

#endif // OCTREE_H_INCLUDED
