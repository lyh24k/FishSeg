#include "pre.h"

int main()
{
	PreTreatment pre;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("input/lancangjiangliefuyu1.pcd", *cloud) == -1) {
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return -1;
	}
	PointCloudT::Ptr rgbcloud(new PointCloudT);
	if (pcl::io::loadPCDFile<PointT>("input/sagajianluoli3.pcd", *rgbcloud) == -1) {
		PCL_ERROR("Couldnot read file.\n");
		system("pause");
		return -1;
	}
	/*PointCloudT::Ptr result(new PointCloudT);
	string path = "C:\\Users\\12401\\Desktop\\fishmodel\\20190908\\gaerxianzhuiwenyexuyu1.obj";
	//pre.Allin(rgbcloud);
	result = pre.Obj2Pcd(path);
	if (!result->empty())
	{
		pcl::io::savePCDFileASCII("input/gaerxianzhuiwenyexuyu1.pcd", *result);
		cout << "success" << endl;
	}*/
	pre.Allin(rgbcloud);
	//pre.getfilesname();

	getchar();
	return 0;
}