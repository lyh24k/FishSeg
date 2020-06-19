#include "pre.h"

int PreTreatment::getfilesname()
{
	string folder_path = "C:\\Users\\neau_\\Desktop\\fishmodel\\20190829\\*.obj";
	vector<String> file_names;
	//保存到当前目录下的pcd文件夹
	string folderpath = folder_path;
	string::size_type position;
	position = folderpath.find_last_of("\\");
	if (position != folderpath.npos)
	{
		folderpath = folderpath.erase(position + 1, -1) + "pcd";
	}
	if (_access(folderpath.c_str(), 0)==-1)
	{
		if (_mkdir(folderpath.c_str())==-1)
		{
			return FALSE;
		}
	}

	glob(folder_path, file_names);
	
	for (int i = 0; i < file_names.size(); i++)
	{
		string filename;
		filename = file_names[i];
		string pcdpath=filename;
		position = pcdpath.find_last_of("\\");
		pcdpath = pcdpath.erase(0, position);
		position = pcdpath.find_last_of(".");
		pcdpath.replace(position+1, 3, "pcd");
		pcdpath = folderpath + pcdpath;
		//cout << filename << endl;
		//cout << pcdpath << endl;
		PointCloudT::Ptr objcloud(new PointCloudT);
		objcloud = Obj2Pcd(filename);
		PointCloudT::Ptr cloud_simple(new PointCloudT);
		cloud_simple = DownSample(objcloud);
		if (!cloud_simple->empty())
		{
			pcl::io::savePCDFileASCII(pcdpath, *cloud_simple);
		}
		else
		{
			cout << "pcd empty" << endl;
			continue;
		}
		cout << pcdpath << " success" << endl;
		
	}
	cout << "finish!!!" << endl;
	return 1;
}

PointCloudT::Ptr PreTreatment::Obj2Pcd(string filename)
{
	clock_t start, endt;
	start = clock();
	ifstream objfile(filename);
	if (!objfile.is_open())
	{
		cout << "open objfile error!" << endl;
		return FALSE;
	}
	string line;
	_xyz v;
	_xy vt;
	_normal vn;
	_trif f;
	Mat textureImg;
	map<int, int> vmap;//保存顶点和UV对应的索引
					   //读取obj，存V VT VN F
	while (getline(objfile, line))
	{
		if (line.length() < 2)
			continue;
		istringstream ss(line);
		string type;
		ss >> type;
		if (type == "#")
		{
			continue;
		}
		else if (type == "v")
		{
			ss >> v.x >> v.y >> v.z;
			V.push_back(v);
		}
		else if (type == "vt")
		{
			ss >> vt.x >> vt.y;
			VT.push_back(vt);
		}
		else if (type == "vn")
		{
			ss >> vn.nx >> vn.ny >> vn.nz;
			VN.push_back(vn);
		}
		else if (type == "mtllib")
		{
			string mtlname;
			string folderpath = filename;
			ss >> mtlname;
			string::size_type position;
			position = mtlname.find_first_of(".");
			position = mtlname.find(".");
			if (position == 0)
			{
				mtlname = mtlname.erase(position, 2);
			}
			position = folderpath.find_last_of("\\");
			if (position != folderpath.npos)
			{
				mtlname = folderpath.erase(position + 1, -1) + mtlname;
			}
			//mtlname= "input/obj/" + mtlname;
			ifstream mtlfile(mtlname);
			if (!mtlfile.is_open())
			{
				cout << "open mtlfile error!" << endl;
				return FALSE;
			}
			string mtlline;
			string texturename;
			while (getline(mtlfile, mtlline))
			{
				istringstream mtlss(mtlline);
				string map_kd;
				mtlss >> map_kd;
				if (map_kd == "map_Kd")
				{
					mtlss >> texturename;
					break;
				}
			}
			mtlfile.close();
			texturename = folderpath + texturename;

			//texturename = filename.replace(filename.find("."), 4, texturename);
			textureImg = imread(texturename, 1);
			if (textureImg.data == NULL)
			{
				cout << "can't find texture file" << endl;
				return FALSE;
			}
			TEXWIDTH = textureImg.cols;
			TEXHEIGHT = textureImg.rows;
		}
		else if (type == "f")
		{
			string s1, s2, s3;
			ss >> s1 >> s2 >> s3;
			for (int i = 0; i < s1.size(); i++)
			{
				if (s1[i] == '/')
				{
					s1[i] = ' ';
				}
			}
			istringstream temp1(s1);
			temp1 >> f.v[0] >> f.t[0] >> f.n[0];
			for (int i = 0; i < s2.size(); i++)
			{
				if (s2[i] == '/')
				{
					s2[i] = ' ';
				}
			}
			istringstream temp2(s2);
			temp2 >> f.v[1] >> f.t[1] >> f.n[1];
			for (int i = 0; i < s3.size(); i++)
			{
				if (s3[i] == '/')
				{
					s3[i] = ' ';
				}
			}
			istringstream temp3(s3);
			temp3 >> f.v[2] >> f.t[2] >> f.n[2];
			for (int i = 0; i < 3; i++)
			{
				vmap[f.v[i]] = f.t[i];
			}
			F.push_back(f);

		}
		else
		{
			continue;
		}
	}
	objfile.close();

	//转pcd
	PointCloudT::Ptr objcloud(new PointCloudT);
	objcloud->resize(V.size());
	for (int i = 0; i < V.size(); i++)
	{
		objcloud->points[i].x = V[i].x;
		objcloud->points[i].y = V[i].y;
		objcloud->points[i].z = V[i].z;
		//计算颜色
		int index = vmap[i + 1];//UV的索引
		if (index > 0 && index < VT.size()) {
			int x = VT[index - 1].x*TEXWIDTH;
			int y = TEXHEIGHT - VT[index - 1].y*TEXHEIGHT;
			if (x >= TEXWIDTH || y >= TEXHEIGHT || x < 0 || y < 0)
			{
				cout << "out of img" << endl;
				return FALSE;
			}
			objcloud->points[i].r = textureImg.at<Vec3b>(y, x)[2];
			objcloud->points[i].g = textureImg.at<Vec3b>(y, x)[1];
			objcloud->points[i].b = textureImg.at<Vec3b>(y, x)[0];
		}

	}
	endt = clock();
	cout << "success: " << (double)(endt - start) / CLOCKS_PER_SEC << endl;

	return objcloud;
	
}

PointCloudT::Ptr PreTreatment::DownSample(PointCloudT::Ptr cloud)
{
	pcl::PointCloud<PointT>::Ptr cloud_simple(new pcl::PointCloud<PointT>());
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(1, 1, 1);
	sor.filter(*cloud_simple);
	return cloud_simple;
}

PointCloudT::Ptr PreTreatment::RemovePlane(PointCloudT::Ptr cloud)
{
	//ransac挑出平面
	pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>());

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.8);
	seg.setMaxIterations(1000);
	seg.setInputCloud(cloud->makeShared());
	seg.segment(*inliers, *coefficients);

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);        //提取内点的索引并存储在其中
	extract.setNegative(true);
	extract.filter(*cloud_filtered);

	return cloud_filtered;
}

int PreTreatment::Allin(PointCloudT::Ptr cloud)
{
	//降采样
	pcl::PointCloud<PointT>::Ptr cloud_downSample(new pcl::PointCloud<PointT>());
	cloud_downSample = DownSample(cloud);
	//Don分割
	//pcl::PointCloud<PointT>::Ptr cloud_DonSeg(new pcl::PointCloud<PointT>());
	//cloud_DonSeg = DoNSeg(cloud);
	//pcl::io::savePCDFileASCII("outputtest/cloud_DonSeg.pcd", *cloud_DonSeg);
	//形态学滤波
	//pcl::PointCloud<PointT>::Ptr cloud_filterZ(new pcl::PointCloud<PointT>());
	//cloud_filterZ = FilterZ(cloud_downSample);
	//pcl::io::savePCDFileASCII("outputtest/cloud_filterZ.pcd", *cloud_filterZ);
	//过分割
	//pcl::PointCloud<PointT>::Ptr cloud_overseg(new pcl::PointCloud<PointT>());
	//cloud_overseg = OverSeg(cloud_downSample);
	//pcl::io::savePCDFileASCII("outputtest/cloud_overseg.pcd", *cloud_overseg);

	//去除平面
	pcl::PointCloud<PointT>::Ptr cloud_removePlane(new pcl::PointCloud<PointT>());
	cloud_removePlane = RemovePlane(cloud_downSample);
	pcl::io::savePCDFileASCII("outputtest/remove_plane.pcd", *cloud_removePlane);
	//欧式分割
	pcl::PointCloud<PointT>::Ptr cloud_Euc(new pcl::PointCloud<PointT>());
	cloud_Euc = EuclideanSeg(cloud_removePlane);
	pcl::io::savePCDFileASCII("outputtest/cloud_Euc.pcd", *cloud_Euc);
	//区域生长
	pcl::PointCloud<PointT>::Ptr cloud_regionSeg(new pcl::PointCloud<PointT>());
	cloud_regionSeg = RegionGrowingSeg(cloud_downSample);
	pcl::io::savePCDFileASCII("outputtest/cloud_regionSeg.pcd", *cloud_regionSeg);
	cout << "All in finish!!" << endl;
	return 1;
}

int PreTreatment::Allin(string filename)
{
	PointCloudT::Ptr cloud(new PointCloudT);
	//读取obj
	cloud = Obj2Pcd(filename);
	if (cloud->empty())
	{
		return FALSE;
	}
	//降采样
	PointCloudT::Ptr downsample_cloud(new PointCloudT);
	downsample_cloud = DownSample(cloud);
	//pcl::io::savePCDFileASCII("output/result.pcd", *downsample_cloud);
	//挑出鱼
	PointCloudT::Ptr fishonly_cloud(new PointCloudT);
	fishonly_cloud = RegionGrowingSeg(downsample_cloud);
	//fishonly_cloud = RemovePlane(downsample_cloud);
	//pcl::io::savePCDFileASCII("output/fishonly.pcd", *fishonly_cloud);

	cout << "All in finish" << endl;
	return 1;
}

PointCloudT::Ptr PreTreatment::PointCloudScale(double k, PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr scalePointCloud(new PointCloudT);
	scalePointCloud->resize(cloud->points.size());
	for (int i = 0; i < cloud->points.size(); ++i)
	{
		scalePointCloud->points[i].x = k*cloud->points[i].x;
		scalePointCloud->points[i].y = k*cloud->points[i].y;
		scalePointCloud->points[i].z = k*cloud->points[i].z;
		scalePointCloud->points[i].r = cloud->points[i].r;
		scalePointCloud->points[i].g = cloud->points[i].g;
		scalePointCloud->points[i].b = cloud->points[i].b;
	}
	return scalePointCloud;
}

PointCloudT::Ptr PreTreatment::RegionGrowingSeg(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr smallcloud(new PointCloudT);
	smallcloud = PointCloudScale(0.01, cloud);
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(smallcloud);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(smallcloud);
	normal_estimator.setKSearch(50);// 1.50 2.30 3.30 4.60 5.50
	normal_estimator.compute(*normals);
	pcl::RegionGrowing<PointT, pcl::Normal> reg;
	reg.setMinClusterSize(10);
	reg.setMaxClusterSize(100000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(30);//1.30 2.20 3.30 4.20 5.20 6.40
	reg.setInputCloud(smallcloud);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(2.0);
	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);
	pcl::PointCloud <PointT>::Ptr colored_cloud = reg.getColoredCloudRGBA();
	pcl::io::savePCDFileASCII("outputtest/region_cloud.pcd", *colored_cloud);
	return colored_cloud;
}

PointCloudT::Ptr PreTreatment::FilterZ(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr cloud_filtered(new PointCloudT);
	pcl::ProgressiveMorphologicalFilter<PointT> pmf;
	pcl::PointIndicesPtr ground(new pcl::PointIndices);
	int max_w_s(20);
	float slope(3.0f);
	float initial_d(0.5f);
	float max_d(2.0f);
	pmf.setInputCloud(cloud);
	pmf.setMaxWindowSize(max_w_s);
	pmf.setSlope(slope);
	pmf.setInitialDistance(initial_d);
	pmf.setMaxDistance(max_d);
	pmf.extract(ground->indices);

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(ground);
	extract.filter(*cloud_filtered);

	return cloud_filtered;
}

//PointCloudT::Ptr PreTreatment::DoNSeg(PointCloudT::Ptr cloud)
//{
//	double smallscale = 5;
//	double mean_radius;
//	double largescale = 10;
//	double threshold = 0.1;
//	double segradius = 1.5;
//	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
//	tree->setInputCloud(cloud);
//	//caculate the mean radius of cloud and mutilply with corresponding input
//	int size_cloud = cloud->size();
//	int step = size_cloud / 10;
//	double total_distance = 0;
//	int i, j = 1;
//	for (i = 0; i<size_cloud; i += step, j++)
//	{
//		std::vector<int> pointIdxNKNSearch(2);
//		std::vector<float> pointNKNSquaredDistance(2);
//		tree->nearestKSearch(cloud->points[i], 2, pointIdxNKNSearch, pointNKNSquaredDistance);
//		total_distance += pointNKNSquaredDistance[1] + pointNKNSquaredDistance[0];
//	}
//	mean_radius = sqrt((total_distance / j));
//	cout << "mean radius of cloud is： " << mean_radius << endl;
//	smallscale *= mean_radius;
//	largescale *= mean_radius;
//	segradius *= mean_radius;
//	if (smallscale >= largescale)
//	{
//		cerr << "Error: Large scale must be > small scale!" << endl;
//		exit(EXIT_FAILURE);
//	}
//	// Compute normals using both small and large scales at each point
//	pcl::NormalEstimationOMP<PointT, pcl::PointNormal> ne;
//	ne.setInputCloud(cloud);
//	ne.setSearchMethod(tree);
//	ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
//	cout << "Calculating normals for scale..." << smallscale << endl;
//	pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<pcl::PointNormal>);
//	ne.setRadiusSearch(smallscale);
//	ne.compute(*normals_small_scale);
//	// calculate normals with the large scale
//	cout << "Calculating normals for scale..." << largescale << endl;
//	pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<pcl::PointNormal>);
//	ne.setRadiusSearch(largescale);
//	ne.compute(*normals_large_scale);
//	pcl::PointCloud<pcl::PointNormal>::Ptr doncloud(new pcl::PointCloud<pcl::PointNormal>);
//	pcl::copyPointCloud<PointT, pcl::PointNormal>(*cloud, *doncloud);
//	cout << "Calculating DoN... " << endl;
//	pcl::DifferenceOfNormalsEstimation<PointT, pcl::PointNormal, pcl::PointNormal> don;
//	don.setInputCloud(cloud);
//	don.setNormalScaleLarge(normals_large_scale);
//	don.setNormalScaleSmall(normals_small_scale);
//	if (!don.initCompute())
//	{
//		std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
//		exit(EXIT_FAILURE);
//	}
//
//	// Compute DoN
//	don.computeFeature(*doncloud);
//	// Save DoN features
//	pcl::PCDWriter writer;
//	writer.write<pcl::PointNormal>("don.pcd", *doncloud, false);
//	cout << "Filtering out DoN mag <= " << threshold << "..." << endl;
//	// Build the condition for filtering
//	pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond(new pcl::ConditionOr<pcl::PointNormal>());
//	range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
//		new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, threshold))
//	);
//	// Build the filter
//	pcl::ConditionalRemoval<pcl::PointNormal> condrem(range_cond);
//	condrem.setInputCloud(doncloud);
//
//	pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
//
//	// Apply filter
//	condrem.filter(*doncloud_filtered);
//
//	doncloud = doncloud_filtered;
//	writer.write<pcl::PointNormal>("don_filtered.pcd", *doncloud, false);
//	pcl::search::KdTree<pcl::PointNormal>::Ptr segtree(new pcl::search::KdTree<pcl::PointNormal>);
//	segtree->setInputCloud(doncloud);
//	std::vector<pcl::PointIndices> cluster_indices;
//	pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
//
//	ec.setClusterTolerance(segradius);
//	ec.setMinClusterSize(50);
//	ec.setMaxClusterSize(100000);
//	ec.setSearchMethod(segtree);
//	ec.setInputCloud(doncloud);
//	ec.extract(cluster_indices);
//
//	pcl::PointCloud <pcl::PointXYZ>::Ptr tmp_xyz(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*doncloud, *tmp_xyz);
//	pcl::PointCloud <PointT>::Ptr colored_cloud = getColoredCloud(tmp_xyz, cluster_indices, 0, 255, 0);
//	return colored_cloud;
//}

PointCloudT::Ptr PreTreatment::EuclideanSeg(PointCloudT::Ptr cloud)
{
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;   //欧式聚类对象
	ec.setClusterTolerance(1.5f);                     // 设置近邻搜索的搜索半径为1cm
	ec.setMinClusterSize(10);                 //设置一个聚类需要的最少的点数目为100
	ec.setMaxClusterSize(50000);               //设置一个聚类需要的最大点数目为25000
	ec.setSearchMethod(tree);                    //设置点云的搜索机制
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中
	PointCloudT::Ptr cloud_EuSeg(new PointCloudT);
	std::vector<pcl::PointIndices>::const_iterator it;
	std::vector<pcl::PointIndices>::const_iterator temp;
	int maxpoints = 0;
	for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		if (it->indices.size() > maxpoints)
		{
			maxpoints = it->indices.size();
			temp = it;
		}
		/*int color_R = Random(255);
		int color_G = Random(255);
		int color_B = Random(255);
		PointT pointEuc;
		for (int i = 0; i < it->indices.size(); ++i)
		{
			pointEuc.x = cloud->points[it->indices[i]].x;
			pointEuc.y = cloud->points[it->indices[i]].y;
			pointEuc.z = cloud->points[it->indices[i]].z;
			pointEuc.r = color_R;
			pointEuc.g = color_G;
			pointEuc.b = color_B;
			cloud_EuSeg->points.push_back(pointEuc);
		}*/
	}
	PointT pointEuc;
	for (int i = 0; i < temp->indices.size(); ++i)
	{
		pointEuc= cloud->points[temp->indices[i]];
		cloud_EuSeg->points.push_back(pointEuc);
	}
	cloud_EuSeg->resize(cloud_EuSeg->points.size());
	return cloud_EuSeg;
}

PointCloudT::Ptr PreTreatment::getColoredCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_, std::vector <pcl::PointIndices> clusters_, float r, float g, float b)
{
	PointCloudT::Ptr colored_cloud;

	if (!clusters_.empty())
	{
		colored_cloud = (new PointCloudT)->makeShared();

		srand(static_cast<unsigned int> (time(0)));
		std::vector<unsigned char> colors;
		for (size_t i_segment = 0; i_segment < clusters_.size(); i_segment++)
		{
			colors.push_back(static_cast<unsigned char> (rand() % 256));
			colors.push_back(static_cast<unsigned char> (rand() % 256));
			colors.push_back(static_cast<unsigned char> (rand() % 256));
		}

		colored_cloud->width = input_->width;
		colored_cloud->height = input_->height;
		colored_cloud->is_dense = input_->is_dense;
		for (size_t i_point = 0; i_point < input_->points.size(); i_point++)
		{
			PointT point;
			point.x = *(input_->points[i_point].data);
			point.y = *(input_->points[i_point].data + 1);
			point.z = *(input_->points[i_point].data + 2);
			point.r = r;
			point.g = g;
			point.b = b;
			colored_cloud->points.push_back(point);
		}

		std::vector< pcl::PointIndices >::iterator i_segment;
		int next_color = 0;
		for (i_segment = clusters_.begin(); i_segment != clusters_.end(); i_segment++)
		{
			std::vector<int>::iterator i_point;
			for (i_point = i_segment->indices.begin(); i_point != i_segment->indices.end(); i_point++)
			{
				int index;
				index = *i_point;
				colored_cloud->points[index].r = colors[3 * next_color];
				colored_cloud->points[index].g = colors[3 * next_color + 1];
				colored_cloud->points[index].b = colors[3 * next_color + 2];
			}
			next_color++;
		}
	}

	return (colored_cloud);
}

PointCloudT::Ptr PreTreatment::OverSeg(PointCloudT::Ptr cloud)
{
	PointCloudT::Ptr smallcloud(new PointCloudT);
	smallcloud = PointCloudScale(1, cloud);
	float smoothness_threshold = 0.1;
	float voxel_resolution = 0.5f;
	float seed_resolution = 5.0f;
	float color_importance = 0.0f;
	float spatial_importance = 1.0f;
	float normal_importance = 4.0f;
	cout << smallcloud->points.size() << endl;
	//生成结晶器
	pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
	super.setUseSingleCameraTransform(false);
	//输入点云和设置参数
	super.setInputCloud(smallcloud);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;
	super.extract(supervoxel_clusters);
	cout << "super " << supervoxel_clusters.size() << endl;
	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);
	pcl::PointCloud<PointTL>::Ptr supervoxel_centroid_cloud = super.getLabeledCloud();
	PointCloudT::Ptr coloredcloud(new PointCloudT);
	coloredcloud = Label2Color(supervoxel_centroid_cloud);
	return coloredcloud;
}

PointCloudT::Ptr PreTreatment::Label2Color(pcl::PointCloud<PointTL>::Ptr cloud)
{
	int labelcount = 0;
	std::map<int, RGB> cloudrgb;
	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (cloudrgb.count(cloud->points[i].label) > 0)
		{
			continue;
		}
		else
		{
			RGB rgb;
			rgb.r = Random(255);
			rgb.g = Random(255);
			rgb.b = Random(255);
			cloudrgb[cloud->points[i].label] = rgb;
		}
		if (cloud->points[i].label > labelcount)
		{
			labelcount = cloud->points[i].label;
		}
	}
	PointCloudT::Ptr colorvoxelcloud(new PointCloudT);
	colorvoxelcloud->resize(cloud->points.size());
	for (int i = 0; i < cloud->points.size(); i++)
	{
		map<int, RGB>::iterator it;
		it = cloudrgb.find(cloud->points[i].label);
		if (it != cloudrgb.end())
		{
			colorvoxelcloud->points[i].x = cloud->points[i].x;
			colorvoxelcloud->points[i].y = cloud->points[i].y;
			colorvoxelcloud->points[i].z = cloud->points[i].z;
			colorvoxelcloud->points[i].r = it->second.r;
			colorvoxelcloud->points[i].g = it->second.g;
			colorvoxelcloud->points[i].b = it->second.b;

		}
		else
		{
			cout << "can't find color" << endl;
			continue;
		}

	}
	return colorvoxelcloud;
}

