#pragma once

#include <iostream>
#include <io.h>
#include <direct.h>
#include <string>
//#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>  
#include <pcl/segmentation/supervoxel_clustering.h>  
#include <pcl/segmentation/lccp_segmentation.h>
#include <pcl/segmentation/cpc_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ia_kfpcs.h>
#include <pcl/recognition/linemod/line_rgbd.h>
#include <pcl/features/boundary.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>
#include <Eigen/Dense>
#include <set>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

#define Random(x) (rand() % x)

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZL PointTL;

struct RGB {
	int r;
	int g;
	int b;
};
//存顶点信息
struct _xyz {
	double x;
	double y;
	double z;
	int r = 0;
	int g = 0;
	int b = 0;
};
//纹理坐标
struct _xy {
	double x;
	double y;
};
//法线
struct _normal {
	double nx;
	double ny;
	double nz;
};
//面
struct _trif {
	int v[3];
	int t[3];
	int n[3];
};

class PreTreatment
{
public:
	PointCloudT::Ptr Obj2Pcd(string filename);
	int getfilesname();
	PointCloudT::Ptr DownSample(PointCloudT::Ptr cloud);
	PointCloudT::Ptr RemovePlane(PointCloudT::Ptr cloud);
	int Allin(PointCloudT::Ptr cloud);
	int Allin(string filename);
	PointCloudT::Ptr PointCloudScale(double k, PointCloudT::Ptr cloud);
	PointCloudT::Ptr RegionGrowingSeg(PointCloudT::Ptr cloud);
	PointCloudT::Ptr FilterZ(PointCloudT::Ptr cloud);
	PointCloudT::Ptr DoNSeg(PointCloudT::Ptr cloud);
	PointCloudT::Ptr EuclideanSeg(PointCloudT::Ptr cloud);
	PointCloudT::Ptr getColoredCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_, std::vector <pcl::PointIndices> clusters_, float r, float g, float b);
	PointCloudT::Ptr OverSeg(PointCloudT::Ptr cloud);
	PointCloudT::Ptr Label2Color(pcl::PointCloud<PointTL>::Ptr cloud);

private:
	vector<_xyz> V;//存顶点坐标
	vector<_xy> VT;//存纹理坐标
	vector<_normal> VN;//法向量
	vector<_trif> F;//面
	int TEXWIDTH;
	int TEXHEIGHT;
};