#pragma warning(disable : 4996)
#include<pcl/surface/poisson.h>
#include<pcl/point_types.h>
#include<pcl/io/ply_io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include<thread>
#include <vector>
#include <algorithm>
#include<opencv2/core.hpp>
#include<opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <limits>
#include <list>
#include<string>
#include <unordered_set>
#include<filesystem>
#include "octree.h"
#include <chrono>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
//将xyz文件处理并保存为ply文件
void topcd(const std::string name) {
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr could(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	pcl::PolygonMesh cl;
	std::ifstream file(name);
	double x1, x2, x3, x4, x5, x6;
	while (file >> x1 >> x2 >> x3 >> x4 >> x5 >> x6) {
		pcl::PointXYZRGBNormal pt;
		pt.x = x1;
		pt.y = x2;
		pt.z = x3;
		pt.normal_x = x4;
		pt.normal_y = x5;
		pt.normal_z = x6;
		could->points.push_back(pt);
	}
	could->width = (int)could->points.size();
	could->height = 1;
	could->is_dense = true;
	
	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud(could);
	//创建Poisson对象，并设置参数
	pcl::Poisson<pcl::PointXYZRGBNormal> pn;
	pn.setConfidence(true); 
	pn.setDegree(2);
	pn.setDepth(8); 
	pn.setIsoDivide(8);
	pn.setManifold(true); 
	pn.setOutputPolygons(false); 
	pn.setSamplesPerNode(3.0); 
	pn.setScale(1.25); 
	pn.setSolverDivide(8); 

	
	pn.setSearchMethod(tree2);
	pn.setInputCloud(could);
	pcl::PolygonMesh mesh;
	pn.performReconstruction(mesh);

	pcl::io::savePLYFile("result.ply", mesh);
	std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("jdasi"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPolygonMesh(mesh, "my");
	viewer->addCoordinateSystem(10);
	viewer->initCameraParameters();
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
}
// 用于判断投影是否在visual hull内部
struct Projection
{
	Eigen::Matrix<float, 3, 4> m_projMat;
	cv::Mat m_image;
	const uint m_threshold = 125;

	bool outOfRange(int x, int max)const
	{
		return x < 0 || x >= max;
	}

	bool checkRange(double x, double y, double z)const
	{
		Eigen::Vector3f vec3 = m_projMat * Eigen::Vector4f(x, y, z, 1);
		int indX = vec3[1] / vec3[2];
		int indY = vec3[0] / vec3[2];

		if (outOfRange(indX, m_image.size().height) || outOfRange(indY, m_image.size().width))
			return false;
		return m_image.at<uchar>((uint)(vec3[1] / vec3[2]), (uint)(vec3[0] / vec3[2])) > m_threshold;
	}
};

class Model
{
public:
	typedef Octree Voxel;

	Model();
	~Model();

	void saveModel(const char* pFileName);
	void saveModelWithNormal(const char* pFileName);
	void loadMatrix(const char* pFileName);
	void loadImage(const char* pDir, const char* pPrefix, const char* pSuffix);

	void getModel();
	void getSurface();
	Eigen::Vector3f getNormal(float coorX, float coorY, float coorZ);

private:

	int m_neiborSize;
	std::vector<Projection> m_projectionList;
	Voxel* m_voxel;
	std::list<Onode*> m_surface;
	double dot_pitch_x;
	double dot_pitch_y;
	double dot_pitch_z;
	void SetNodeColor(Onode* node_ptr);
	void ExpandAllChildren(Onode* node_ptr);
	void ShrinkPitch();
};

Model::Model() {
	m_neiborSize = 2;

	int generation = 5;
	m_voxel = new Voxel(generation);	//首先生成5代的八叉树骨干用于初步搜索

	dot_pitch_x = (m_voxel->x_max - m_voxel->x_min) * pow(2, -generation);
	dot_pitch_y = (m_voxel->y_max - m_voxel->y_min) * pow(2, -generation);
	dot_pitch_z = (m_voxel->z_max - m_voxel->z_min) * pow(2, -generation);

}

Model::~Model()
{
	delete m_voxel;
}

void Model::saveModel(const char* pFileName)
{
	std::ofstream fout(pFileName);

	for (auto it = m_surface.cbegin(); it != m_surface.cend(); it++) {
		fout << (*it)->x << ' ' << (*it)->y << ' ' << (*it)->z << std::endl;
	}
}

void Model::saveModelWithNormal(const char* pFileName)
{
	std::ofstream fout(pFileName);

	for (auto it = m_surface.cbegin(); it != m_surface.cend(); it++) {
		fout << (*it)->x << ' ' << (*it)->y << ' ' << (*it)->z << ' ';
		Eigen::Vector3f nor = getNormal((*it)->x, (*it)->y, (*it)->z);
		fout << nor(0) << ' ' << nor(1) << ' ' << nor(2) << std::endl;

	}
}

void Model::loadMatrix(const char* pFileName)
{
	std::ifstream fin(pFileName);

	int num;
	Eigen::Matrix<float, 3, 3> matInt;
	Eigen::Matrix<float, 3, 4> matExt;
	Projection projection;
	while (fin >> num)
	{
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				fin >> matInt(i, j);

		double temp;
		fin >> temp;
		fin >> temp;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				fin >> matExt(i, j);

		projection.m_projMat = matInt * matExt;
		m_projectionList.push_back(projection);
	}
}

void Model::loadImage(const char* pDir, const char* pPrefix, const char* pSuffix)
{
	int fileCount = m_projectionList.size();
	std::string fileName(pDir);
	fileName += '/';
	fileName += pPrefix;
	for (int i = 0; i < fileCount; i++)
	{
		m_projectionList[i].m_image = cv::imread(fileName + std::to_string(i) + pSuffix, CV_8UC1);
	}
}

void Model::SetNodeColor(Onode* node_ptr) {
	double coorX = node_ptr->x;
	double coorY = node_ptr->y;
	double coorZ = node_ptr->z;

	int projection_count = m_projectionList.size();

	for (int i = 0; i < projection_count; i++) {
		if (!m_projectionList[i].checkRange(coorX, coorY, coorZ)) {	//如果该点落在任一投影外部
			node_ptr->status = BLACK;	//设置为黑色状态
			return;
		}
	}
	node_ptr->status = WHITE;	//否则落在投影内部，设置为白色
}

void Model::ExpandAllChildren(Onode* node_ptr) {
	for (int i = 0; i < 8; i++) {	//为所有孩子扩展叶子节点
		m_voxel->Expand(node_ptr->childs[i], LEAVES);
		node_ptr->childs[i]->status = PIVOT;	//本函数用于表面点附近，故将非叶子结点设置为PIVOT
		for (int j = 0; j < 8; j++) {
			SetNodeColor(node_ptr->childs[i]->childs[j]);	//设置扩展结点的颜色，并将其
			m_surface.push_back(node_ptr->childs[i]->childs[j]);	//放入表面点集稍后处理
		}
	}
}

void Model::ShrinkPitch() {
	dot_pitch_x /= 2;
	dot_pitch_y /= 2;
	dot_pitch_z /= 2;
}

void Model::getModel() {

	struct UpdateStatusSurface {
		Model* model;

		UpdateStatusSurface(Model* m) :model(m) {};

		void operator()(Onode* node_ptr) {
			switch (node_ptr->status)
			{
			case(BODY_END): {
				model->SetNodeColor(node_ptr);
				model->m_surface.push_back(node_ptr);
				break;
			}
			case(BODY): {
				model->m_voxel->UpdateStatus(node_ptr);
			}
			default:
				break;
			}
		}
	}update_status_surface(this);

	m_voxel->TravPost(m_voxel->GetRoot(), update_status_surface);
}

void Model::getSurface()
{
	// 邻域：上、下、左、右、前、后。
	int dx[6] = { -1, 0, 0, 0, 0, 1 };
	int dy[6] = { 0, 1, -1, 0, 0, 0 };
	int dz[6] = { 0, 0, 0, 1, -1, 0 };

	//用于判断某个点是否在Voxel的范围内
	auto outOfRange = [&](double xx, double yy, double zz) {
		return xx <= m_voxel->x_min || m_voxel->x_max <= xx
			|| yy <= m_voxel->y_min || m_voxel->y_max <= yy
			|| zz < -m_voxel->z_min || m_voxel->z_max <= zz;
	};

	for (auto it = m_surface.begin(); it != m_surface.end();) {	//遍历所有表面附近的点
		if ((*it)->status == WHITE) {	//当它在点云内时
			for (int i = 0; i < 6; i++) {
				double coor_x = (*it)->x + dx[i] * dot_pitch_x;
				double coor_y = (*it)->y + dy[i] * dot_pitch_y;
				double coor_z = (*it)->z + dz[i] * dot_pitch_z;
				if (m_voxel->GetStatus(coor_x, coor_y, coor_z, m_voxel->GetRoot()) == BLACK) {	//查看相邻区域是否在三维点云内，如果有不是的
					(*it)->status = SURFACE;	//标记这一点为表面点
					break;
				}
			}
		}
		if ((*it)->status != SURFACE) {	//若该点本身不在点云内或包括它和六个相邻点在内七个点都位于点云内
			it = m_surface.erase(it);	//从表面集合中删除该点
		}
		else {
			it++;
		}
	}

	std::cout << "surface size: " << m_surface.size() << std::endl;

	if (m_surface.size() < 20000) {	//当表面点数量不足以进行有效三维重建时
		std::unordered_set<Onode*> nodes2expand;
		for (auto it = m_surface.begin(); it != m_surface.end(); it = m_surface.erase(it)) {	//取出所有表面点
			for (int i = 0; i < 6; i++) {	//记录其相邻的结点的父节点（必然包含自身的父节点）
				double coor_x = (*it)->x + dx[i] * dot_pitch_x;
				double coor_y = (*it)->y + dy[i] * dot_pitch_y;
				double coor_z = (*it)->z + dz[i] * dot_pitch_z;
				Onode* node_ptr = m_voxel->FindPoint(coor_x, coor_y, coor_z, m_voxel->GetRoot());
				if (node_ptr) {
					nodes2expand.insert(node_ptr->parent);
					m_voxel->UpdatePivot(node_ptr->parent);
				}
			}
		}
		ShrinkPitch();
		for (auto it = nodes2expand.cbegin(); it != nodes2expand.cend(); it++) {	//对所有记录的父辈结点
			m_voxel->UpdatePivot(*it);	//将父辈结点及其祖先都设置为PIVOT
			ExpandAllChildren(*it);	//扩展他们所有的孩子，放入表面集合
		}
		getSurface();	//递归地处理表面集合
	}
}

Eigen::Vector3f Model::getNormal(float coor_x, float coor_y, float coor_z)
{
	auto outOfRange = [&](float coorX, float coorY, float coorZ) {
		return coorX <= m_voxel->x_min || m_voxel->x_max <= coorX
			|| coorY <= m_voxel->y_min || m_voxel->y_max <= coorY
			|| coorZ < -m_voxel->z_min || m_voxel->z_max <= coorZ;
	};

	std::vector<Eigen::Vector3f> neiborList;
	std::vector<Eigen::Vector3f> innerList;

	for (int dX = -m_neiborSize; dX <= m_neiborSize; dX++) {
		for (int dY = -m_neiborSize; dY <= m_neiborSize; dY++) {
			for (int dZ = -m_neiborSize; dZ <= m_neiborSize; dZ++)
			{
				if (!dX && !dY && !dZ) {
					continue;
				}
				float neiborX = coor_x + dX * dot_pitch_x;
				float neiborY = coor_y + dY * dot_pitch_y;
				float neiborZ = coor_z + dZ * dot_pitch_z;
				if (!outOfRange(neiborX, neiborY, neiborZ))
				{

					Onode* node_ptr = m_voxel->FindPoint(neiborX, neiborY, neiborZ, m_voxel->GetRoot());
					if (node_ptr == NULL) {
						continue;
					}
					switch (node_ptr->status)
					{
					case(SURFACE): {
						neiborList.push_back(Eigen::Vector3f(neiborX, neiborY, neiborZ));
						break;
					}
					case(WHITE): {
						innerList.push_back(Eigen::Vector3f(neiborX, neiborY, neiborZ));
						break;
					}
					default:
						break;
					}
				}
			}
		}
	}
	Eigen::Vector3f point(coor_x, coor_y, coor_z);
	Eigen::MatrixXf matA(3, neiborList.size());
	for (int i = 0; i < neiborList.size(); i++) {
		matA.col(i) = neiborList[i] - point;
	}
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(matA * matA.transpose());
	Eigen::Vector3f eigenValues = eigenSolver.eigenvalues();
	int indexEigen = 0;
	if (abs(eigenValues[1]) < abs(eigenValues[indexEigen])) {
		indexEigen = 1;
	}
	if (abs(eigenValues[2]) < abs(eigenValues[indexEigen])) {
		indexEigen = 2;
	}
	Eigen::Vector3f normalVector = eigenSolver.eigenvectors().col(indexEigen);
	Eigen::Vector3f innerCenter = Eigen::Vector3f::Zero();
	for (auto const& vec : innerList) {
		innerCenter += vec;
	}
	innerCenter /= innerList.size();
	if (normalVector.dot(point - innerCenter) < 0) {
		normalVector *= -1;
	}
	return normalVector;
}

int main(int argc, char** argv)
{
	Model model;
	model.loadMatrix(R"(calibParamsI.txt)");
	model.loadImage(R"(WD_segmented)", "WD2_", "_00020_segmented.png");
	model.getModel();
	model.getSurface();
	model.saveModelWithNormal("../../WithNormal.xyz");
	topcd("../../WithNormal.xyz");
}
