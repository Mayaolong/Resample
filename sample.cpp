// ConsoleApplication4.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
/* //ply2pcd
#include <iostream>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>

using namespace std;

int main()
{
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PolygonMesh mesh;
            vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
            pcl::io::loadPolygonFilePLY("3_1.ply", mesh);
            pcl::io::mesh2vtk(mesh, polydata);
            pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);
            pcl::io::savePCDFileASCII("3_1.pcd", *cloud);
            return 0;
}

*/


 /*  //===================pcd2ply
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include<pcl/PCLPointCloud2.h>
#include<iostream>
#include<string>

using namespace pcl;
using namespace pcl::io;
using namespace std;

int PCDtoPLYconvertor(string & input_filename ,string& output_filename)
{
pcl::PCLPointCloud2 cloud;
if (loadPCDFile(input_filename , cloud) < 0)
{
cout << "Error: cannot load the PCD file!!!"<< endl;
return -1;
}
PLYWriter writer;
writer.write(output_filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(),false,false);
return 0;

}

int main()
{
string input_filename = "3_1.pcd";
string output_filename = "03_1.ply";
PCDtoPLYconvertor(input_filename , output_filename);
return 0;
}
*/
/*
// =====================下采样，它的可视化输出写的可以应用
#include <pcl/filters/random_sample.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
 
#define PI 3.1415
 
#include <vector>
 
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
 
PointCloudT::Ptr rotateABit(const PointCloudT::Ptr cloud_in){
    PointCloudT::Ptr cloud_transformed (new PointCloudT);
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    float theta_deg = 30;
    float c = cos(theta_deg*PI / 180.0);
    float s = sin(theta_deg*PI / 180.0);
    transform(1,1) = c;
    transform(3,3) = c;
    transform(1,3) = -s;
    transform(3,1) = s;
    pcl::transformPointCloud(*cloud_in, *cloud_transformed, transform);
    return cloud_transformed;
}
 
int main(){
    PointCloudT::Ptr cloud_in (new PointCloudT), cloud_out (new PointCloudT);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer ("Viewer"));
 
    pcl::io::loadPCDFile("3_1.pcd", *cloud_in);
 
    std::cerr << *cloud_in << std::endl;
 
    pcl::RandomSample<PointT> rs;
    rs.setInputCloud(cloud_in);
    //设置输出点的数量   
rs.setSample(1024);
 
    //下采样并输出到cloud_out
rs.filter(*cloud_out);
 
    //提取采样点的index
//std::vector<int> indices;
    //rs.filter(indices);
 
std::cerr << *cloud_out << std::endl;
 
    //可视化
viewer->addPointCloud(cloud_out, "cloud_out");
 
pcl::visualization::PointCloudColorHandlerCustom<PointT> red(cloud_in, 0, 255, 255);
viewer->addPointCloud(rotateABit(cloud_in), red, "cloud_in");
 
while(!viewer->wasStopped()){
	viewer->spinOnce();
}
return (1);
}
*/


/*  //上采样，程序看着还不错，总是有些include文件的函数线程不安全，头文件太老了
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int default_polynomial_order = 2;
double default_search_radius = 0.0,
    default_sqr_gauss_param = 0.0;


void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -radius X          = sphere radius to be used for finding the k-nearest neighbors used for fitting (default: ");
  print_value ("%f", default_search_radius); print_info (")\n");
  print_info ("                     -sqr_gauss_param X = parameter used for the distance based weighting of neighbors (recommended = search_radius^2) (default: ");
  print_value ("%f", default_sqr_gauss_param); print_info (")\n");
  print_info ("                     -polynomial_order X = order of the polynomial to be fit (polynomial_order > 1, indicates using a polynomial fit) (default: ");
  print_value ("%d", default_polynomial_order); print_info (")\n");
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
compute (const pcl::PCLPointCloud2::ConstPtr &input,
         pcl::PCLPointCloud2 &output,
         double search_radius,
         bool sqr_gauss_param_set,
         double sqr_gauss_param,
         int polynomial_order)
{

  PointCloud<PointXYZ>::Ptr xyz_cloud_pre (new pcl::PointCloud<PointXYZ> ()),
      xyz_cloud (new pcl::PointCloud<PointXYZ> ());
  fromPCLPointCloud2 (*input, *xyz_cloud_pre);

  // Filter the NaNs from the cloud
  for (size_t i = 0; i < xyz_cloud_pre->size (); ++i)
    if (pcl_isfinite (xyz_cloud_pre->points[i].x))
      xyz_cloud->push_back (xyz_cloud_pre->points[i]);
  xyz_cloud->header = xyz_cloud_pre->header;
  xyz_cloud->height = 1;
  xyz_cloud->width = static_cast<uint32_t> (xyz_cloud->size ());
  xyz_cloud->is_dense = false;
  
  

  PointCloud<PointNormal>::Ptr xyz_cloud_smoothed (new PointCloud<PointNormal> ());

  MovingLeastSquares<PointXYZ, PointNormal> mls;
  mls.setInputCloud (xyz_cloud);
  mls.setSearchRadius (search_radius);
  if (sqr_gauss_param_set) mls.setSqrGaussParam (sqr_gauss_param);
  mls.setPolynomialOrder (polynomial_order);

//  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::SAMPLE_LOCAL_PLANE);
//  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::RANDOM_UNIFORM_DENSITY);
//  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::VOXEL_GRID_DILATION);
  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::NONE);
  mls.setPointDensity (60000 * int (search_radius)); // 300 points in a 5 cm radius
  mls.setUpsamplingRadius (0.025);
  mls.setUpsamplingStepSize (0.015);
  mls.setDilationIterations (2);
  mls.setDilationVoxelSize (0.01f);

  search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ> ());
  mls.setSearchMethod (tree);
  mls.setComputeNormals (true);

  PCL_INFO ("Computing smoothed surface and normals with search_radius %f , sqr_gaussian_param %f, polynomial order %d\n",
            mls.getSearchRadius(), mls.getSqrGaussParam(), mls.getPolynomialOrder());
  TicToc tt;
  tt.tic ();
  mls.process (*xyz_cloud_smoothed);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", xyz_cloud_smoothed->width * xyz_cloud_smoothed->height); print_info (" points]\n");

  toPCLPointCloud2 (*xyz_cloud_smoothed, output);
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  pcl::io::savePCDFile (filename, output,  Eigen::Vector4f::Zero (),
                        Eigen::Quaternionf::Identity (), true);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}


int
main (int argc, char** argv)
{
  print_info ("Moving Least Squares smoothing of a point cloud. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 2)
  {
    print_error ("Need one input PCD file and one output PCD file to continue.\n");
    return (-1);
  }

  // Command line parsing
  double search_radius = default_search_radius;
  double sqr_gauss_param = default_sqr_gauss_param;
  bool sqr_gauss_param_set = true;
  int polynomial_order = default_polynomial_order;

  parse_argument (argc, argv, "-radius", search_radius);
  parse_argument (argc, argv, "-polynomial_order", polynomial_order);
  if (parse_argument (argc, argv, "-sqr_gauss_param", sqr_gauss_param) == -1)
    sqr_gauss_param_set = false;

  // Load the first file
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  if (!loadCloud (argv[p_file_indices[0]], *cloud))
    return (-1);

  // Do the smoothing
  pcl::PCLPointCloud2 output;
  compute (cloud, output, search_radius, sqr_gauss_param_set, sqr_gauss_param, polynomial_order);

  // Save into the second file
  saveCloud (argv[p_file_indices[1]], output);
}


*/

/*
//#include <pcl/io/pcd_io.h>
//#include <pcl/surface/mls.h>
//
//int main(int argc,char** argv)
//{
//   // 新建点云存储对象
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//    // 读取文件
//    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
//    {
//        return -1;
//    }
//    // 滤波对象
//    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
//    filter.setInputCloud(cloud);
//    //建立搜索对象
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
//    filter.setSearchMethod(kdtree);
//    //设置搜索邻域的半径为3cm
//    filter.setSearchRadius(0.03);
//    // Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
//    filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
//    // 采样的半径是
//    filter.setUpsamplingRadius(0.03);
//    // 采样步数的大小
//    filter.setUpsamplingStepSize(0.02);
//
//    filter.process(*filteredCloud);
//
//}
*/


//===============经典版  *****  可以使用了，里面包含下采样，均衡采样和 上采样，如果保证采样输出点个是一样的，需要重新设置，现在只能实现上采样的 半径，步骤，步长
/*
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/surface/mls.h>

#include <pcl/filters/random_sample.h>
#include <pcl/common/transforms.h>

#define PI 3.1415
 
#include <vector>
 
//typedef pcl::PointXYZ PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
pcl::PointCloud<pcl::PointXYZ>::Ptr rotateABit(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    float theta_deg = 30;
    float c = cos(theta_deg*PI / 180.0);
    float s = sin(theta_deg*PI / 180.0);
    transform(1,1) = c;
    transform(3,3) = c;
    transform(1,3) = -s;
    transform(3,1) = s;
    pcl::transformPointCloud(*cloud_in, *cloud_transformed, transform);
    return cloud_transformed;
}

int main(int argc, char** argv)
{
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read("3_1.pcd", *cloud); // Remember to download the file first!
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ")."<<std::endl;

	// Create the filtering object
pcl::VoxelGrid<pcl::PointXYZ> sor;    // 创建滤波（下采样）对象
sor.setInputCloud(cloud);             //设置需要过滤的点云
sor.setLeafSize(0.1f, 0.1f, 0.1f);    //设置滤波时创建的体素体积为1cm的立方体
sor.filter(*cloud_filtered1);          //执行滤波处理，储存输出点云 
std::cerr << "PointCloud after filtering: " << cloud_filtered1->width * cloud_filtered1->height
	<< " data points (" << pcl::getFieldsList(*cloud_filtered1) << ")."<<std::endl;

// Create the filtering object

//pcl::UniformSampling<pcl::PointXYZ> sor2;    // 创建均匀采样对象
//sor2.setInputCloud(cloud);             //设置需要过滤的点云
//sor2.setRadiusSearch(0.01f);    //设置滤波时创建的体素体积为1cm的立方体
//sor2.filter(*cloud_filtered2);          //执行滤波处理，储存输出点云 





std::cerr << "PointCloud after filtering: " << cloud_filtered2->width * cloud_filtered2->height
	<< " data points "<< std::endl;

pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> sor3;
sor3.setInputCloud(cloud);
//建立搜索对象
pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
sor3.setSearchMethod(kdtree);
sor3.setSearchRadius(0.1);  //设置搜索邻域的半径为3cm
sor3.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);  // Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSIT
sor3.setUpsamplingRadius(0.1); 	// 采样的半径是
sor3.setUpsamplingStepSize(0.02);// 采样步数的大小
sor3.process(*cloud_filtered3);
std::cerr << "PointCloud after filtering: " << cloud_filtered3->width * cloud_filtered3->height
	<< " data points (" << pcl::getFieldsList(*cloud_filtered3) << ")." << std::endl;
//pcl::PCDWriter writer;
//writer.write("table_scene_lms400_downsampled.pcd", *cloud_filtered,
//Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);

// 下采样输出为固定个数点云
pcl::RandomSample<pcl::PointXYZ> sor2;
sor2.setInputCloud(cloud_filtered3);
sor2.setSample(256);
sor2.filter(*cloud_out);
std::cerr << "PointCloud after 固定点云个数后 filtering: " << cloud_out->width * cloud_filtered3->height
	<< " data points (" << pcl::getFieldsList(*cloud_out) << ")." << std::endl;

pcl::visualization::PCLVisualizer viewer("采样");
int v1(1);   //设置左右窗口
int v2(2);
int v3(3);
int v4(4);
viewer.createViewPort(0.0, 0.0, 0.5, 0.5, v1);
viewer.setBackgroundColor(0, 0, 0, v1);

viewer.createViewPort(0.5, 0.0, 1.0, 0.5, v2);
viewer.setBackgroundColor(0, 0, 0, v2);

viewer.createViewPort(0.0, 0.5, 0.5, 1.0, v3);
viewer.setBackgroundColor(0, 0, 0, v3);

viewer.createViewPort(0.5, 0.5, 1.0, 1.0, v4);
viewer.setBackgroundColor(0, 0, 0, v4);

// The color we will be using
float bckgr_gray_level = 0.0;  // Black    设置背景颜色
float txt_gray_lvl = 1.0 - bckgr_gray_level;

// Original point cloud is white

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
viewer.addPointCloud(cloud, cloud_in_color_h, "cloud_in_v1", v1);   //viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);


pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_green(cloud_filtered1, 20, 180, 20);      // 显示绿色点云
viewer.addPointCloud(cloud_filtered1, cloud_out_green, "cloud_out1", v2); 

 //固定输出
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_orage(cloud_out, 250, 128, 10);     //显示橘色点云
viewer.addPointCloud(cloud_out, cloud_out_orage, "cloud_out2", v3);

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_purple(cloud_filtered3,255, 0, 255);      //显示紫色点云 255 0 255
viewer.addPointCloud(cloud_filtered3, cloud_out_purple, "cloud_out3", v4);

//viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
viewer.setSize(1280, 1024);  // Visualiser window size
//viewer.showCloud(cloud_out);
while (!viewer.wasStopped())
{
	viewer.spinOnce();
}

//保存到PCD文件
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("Up.pcd", *cloud_filtered3, false);//将点云保存到PCD文件中
	writer.write<pcl::PointXYZ>("Guding.pcd", *cloud_out, false);//将点云保存到PCD文件中
	writer.write<pcl::PointXYZ>("xia.pcd", *cloud_filtered1, false);//将点云保存到PCD文件中
	writer.write<pcl::PointXYZ>("yuan.pcd", *cloud, false);//将点云保存到PCD文件中
return (0);
}
*/





//// myl 精简版
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/surface/mls.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/random_sample.h>
#include <pcl/common/transforms.h>
#include <cstring>
#include <pcl/io/ply_io.h>
#include<pcl/PCLPointCloud2.h>
#include <iostream>
#include <fstream>    //这个头文件是必须要有的
#define NUM_POINT 48




using namespace std;
char *input_name = NULL;
char *output_name = NULL;
ofstream outfile;
//=================ply2pcd
void ply2pcd(string filename_ply)
{
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PolygonMesh mesh;
            vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
            pcl::io::loadPolygonFilePLY(filename_ply, mesh);
            pcl::io::mesh2vtk(mesh, polydata);
            pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);
            pcl::io::savePCDFileASCII("ply2pcd.pcd", *cloud);
}
using namespace pcl;
using namespace pcl::io;
 //===================pcd2ply
int PCDtoPLYconvertor(char *output_filename)
{
pcl::PCLPointCloud2 cloud;
if (loadPCDFile("cloud2pcd.pcd" , cloud) < 0)
{
cout << "Error: cannot load the PCD file!!!"<< endl;
return -1;
}
PLYWriter writer;
writer.write(output_filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(),false,false);
return 0;
}

void Upsamp()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the cloud data
	pcl::PCDReader reader;
	pcl::PCDWriter writer;
	// Replace the path below with the path where you saved your file

	 reader.read("ply2pcd.pcd", *cloud); // Remember to download the file first!
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
		<< " data points (" << pcl::getFieldsList(*cloud) << ")."<<std::endl;
   
	if((cloud->width * cloud->height)>= NUM_POINT)
	{
		// Create the filtering object
		/*
		pcl::VoxelGrid<pcl::PointXYZ> sor;    // 创建滤波（下采样）对象
		sor.setInputCloud(cloud);             //设置需要过滤的点云
		sor.setLeafSize(0.008f, 0.008f, 0.008f);    //设置滤波时创建的体素体积为1cm的立方体
		sor.filter(*cloud_filtered1);          //执行滤波处理，储存输出点云 
		std::cerr << "PointCloud after filtering: " << cloud_filtered1->width * cloud_filtered1->height
			<< " data points (" << pcl::getFieldsList(*cloud_filtered1) << ")."<<std::endl;
         if((cloud_filtered1->width * cloud_filtered1->height)< 48)
		{  
			 outfile.open("dianshubugou.txt", ios::app);
			 if(!outfile) //检查文件是否正常打开//不是用于检查文件是否存在
			 {
			  cout<<"abc.txt can't open"<<endl;
			  abort(); //打开失败，结束程序
			 }
			 else
			 {
			 string inname = input_name;
			 string outname = output_name;
			  outfile << "samp "<<input_name<<" "<< output_name<<endl;
			  outfile.close();
			 }                 //关闭文件
		}
		*/
		// 下采样输出为固定个数点云（随机）
		pcl::RandomSample<pcl::PointXYZ> sor2;
		sor2.setInputCloud(cloud);
		sor2.setSample(NUM_POINT);
		sor2.filter(*cloud_out);
	}
	else
	{
		//上采样
		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> sor3;
		sor3.setInputCloud(cloud);
		//建立搜索对象
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
		sor3.setSearchMethod(kdtree);
		sor3.setSearchRadius(0.05);  //设置搜索邻域的半径为3cm
		sor3.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);  // Upsampling 采样的方法有SAMPLE_LOCAL_PLANE  DISTINCT_CLOUD, RANDOM_UNIFORM_DENSIT
		sor3.setUpsamplingRadius(0.05); 	// 采样的半径是
		//double r = 0.06*3.1415*2*cloud->width * cloud->height/(NUM_POINT - cloud->width * cloud->height);
		sor3.setUpsamplingStepSize(0.05);// 采样步数的大小
		sor3.process(*cloud_filtered3);
		std::cerr << "PointCloud after filtering: " << cloud_filtered3->width * cloud_filtered3->height
			<< " data points (" << pcl::getFieldsList(*cloud_filtered3) << ")." << std::endl;
			writer.write<pcl::PointXYZ>("cloud_filtered3.pcd", *cloud_filtered3, false);//将点云保存到PCD文件中
		// 下采样输出为固定个数点云（随机）
		pcl::RandomSample<pcl::PointXYZ> sor2;
		sor2.setInputCloud(cloud_filtered3);
		sor2.setSample(NUM_POINT);
		sor2.filter(*cloud_out);



	}


std::cerr << "PointCloud after 固定点云个数后 filtering: " << cloud_out->width * cloud_out->height
	<< " data points (" << pcl::getFieldsList(*cloud_out) << ")." << std::endl;

writer.write<pcl::PointXYZ>("cloud2pcd.pcd", *cloud_out, false);//将点云保存到PCD文件中
/*
pcl::visualization::PCLVisualizer viewer("采样");
int v1(1);   //设置左右窗口
int v2(2);
int v3(3);
int v4(4);
viewer.createViewPort(0.0, 0.0, 0.5, 0.5, v1);
viewer.setBackgroundColor(0, 0, 0, v1);

viewer.createViewPort(0.0, 0.5, 0.5, 1.0, v3);
viewer.setBackgroundColor(0, 0, 0, v3);

viewer.createViewPort(0.5, 0.5, 1.0, 1.0, v4);
viewer.setBackgroundColor(0, 0, 0, v4);

// The color we will be using
float bckgr_gray_level = 0.0;  // Black    设置背景颜色
float txt_gray_lvl = 1.0 - bckgr_gray_level;


// Original point cloud is white
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl);
viewer.addPointCloud(cloud, cloud_in_color_h, "cloud_in_v1", v1);   //viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_green(cloud_filtered1, 20, 180, 20);      // 显示绿色点云
viewer.addPointCloud(cloud_filtered1, cloud_out_green, "cloud_out1", v2); 

 //固定输出
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_orage(cloud_out, 250, 128, 10);     //显示橘色点云
viewer.addPointCloud(cloud_out, cloud_out_orage, "cloud_out2", v3);

//上采样输出
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_purple(cloud_filtered3,255, 0, 255);      //显示紫色点云 255 0 255
viewer.addPointCloud(cloud_filtered3, cloud_out_purple, "cloud_out3", v4);

//viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
viewer.setSize(1280, 1024);  // Visualiser window size


//viewer.showCloud(cloud_out);
while (!viewer.wasStopped())
{
	viewer.spinOnce();
}
*/
//保存到PCD文件
	
    /*string output_filename_ply;
    string input_filename_pcd = "cloud_out.pcd";
	PCDtoPLYconvertor(input_filename_pcd,output_filename_ply);*/
}
// Program arguments


int ParseArgs(int argc, char **argv)
{
  // Check number of arguments
  if (argc == 1) {
    printf("Usage: samp input_name output_name [options]\n");
    exit(0);
  }

  // Parse arguments
  argc--; argv++;
  while (argc > 0) {
 
      if (!input_name) input_name = *argv;
      else if (!output_name) output_name = *argv;
      else { fprintf(stderr, "Invalid program argument: %s", *argv); exit(1); }
      argv++; argc--;
  }

  // Check input filename
  if (!input_name) {
    fprintf(stderr, "You did not specify an input file name.\n");
    return 0;
  }

  // Check output filename
  if (!output_name) {
    fprintf(stderr, "You did not specify an output file name.\n");
    return 0;
  }

  // Return OK status 
  return 1;
}

int main(int argc, char** argv)
{
	// Check number of arguments
  if (!ParseArgs(argc, argv)) exit(1);
	ply2pcd(input_name); //生成一个此文件夹下的ply.pcd文件,作为Upsamp()的读取文件
	Upsamp();            //生成一个pcd2cloud.pcd文件，用于PCDtoPLYconvertor(output_name)的读取
	PCDtoPLYconvertor(output_name);
	return (0);
}



/*

#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;


//int default_polynomial_order = 2;
//double default_search_radius = 0.0,
//    default_sqr_gauss_param = 0.0;
int default_polynomial_order = 1;
double default_search_radius = 0.1,
    default_sqr_gauss_param = 0.1;


void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s input.pcd output.pcd <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -radius X          = sphere radius to be used for finding the k-nearest neighbors used for fitting (default: ");
  print_value ("%f", default_search_radius); print_info (")\n");
  print_info ("                     -sqr_gauss_param X = parameter used for the distance based weighting of neighbors (recommended = search_radius^2) (default: ");
  print_value ("%f", default_sqr_gauss_param); print_info (")\n");
  print_info ("                     -polynomial_order X = order of the polynomial to be fit (polynomial_order > 1, indicates using a polynomial fit) (default: ");
  print_value ("%d", default_polynomial_order); print_info (")\n");
}

bool
loadCloud (const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud.width * cloud.height); print_info (" points]\n");
  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}

void
compute (const pcl::PCLPointCloud2::ConstPtr &input,
         pcl::PCLPointCloud2 &output,
         double search_radius,
         bool sqr_gauss_param_set,
         double sqr_gauss_param,
         int polynomial_order)
{

  PointCloud<PointXYZ>::Ptr xyz_cloud_pre (new pcl::PointCloud<PointXYZ> ()),
      xyz_cloud (new pcl::PointCloud<PointXYZ> ());
  fromPCLPointCloud2 (*input, *xyz_cloud_pre);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
  // Filter the NaNs from the cloud
  for (size_t i = 0; i < xyz_cloud_pre->size (); ++i)
    if (pcl_isfinite (xyz_cloud_pre->points[i].x))
      xyz_cloud->push_back (xyz_cloud_pre->points[i]);
  xyz_cloud->header = xyz_cloud_pre->header;
  xyz_cloud->height = 1;
  xyz_cloud->width = static_cast<uint32_t> (xyz_cloud->size ());
  xyz_cloud->is_dense = false;
  
  

  PointCloud<PointNormal>::Ptr xyz_cloud_smoothed (new PointCloud<PointNormal> ());


  MovingLeastSquares<PointXYZ, PointNormal> mls;
  mls.setInputCloud (xyz_cloud);
  mls.setSearchRadius (search_radius);
  if (sqr_gauss_param_set) mls.setSqrGaussParam (sqr_gauss_param);
  mls.setPolynomialOrder (polynomial_order);

//  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::SAMPLE_LOCAL_PLANE);
//  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::RANDOM_UNIFORM_DENSITY);
//  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::VOXEL_GRID_DILATION);
  mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointNormal>::NONE);
  mls.setPointDensity (60000 * int (search_radius));   // 300 points in a 5 cm radius
  mls.setUpsamplingRadius (0.025);
  mls.setUpsamplingStepSize (0.015);
  mls.setDilationIterations (2);
  mls.setDilationVoxelSize (0.01f);

  search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ> ());
  mls.setSearchMethod (tree);
  mls.setComputeNormals (true);

  PCL_INFO ("Computing smoothed surface and normals with search_radius %f , sqr_gaussian_param %f, polynomial order %d\n",
            mls.getSearchRadius(), mls.getSqrGaussParam(), mls.getPolynomialOrder());
  TicToc tt;
  tt.tic ();
  mls.process (*xyz_cloud_smoothed);
  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", xyz_cloud_smoothed->width * xyz_cloud_smoothed->height); print_info (" points]\n");
  pcl::PCDWriter writer;
  //writer.write<pcl::PointXYZ>("cloud_filtered3.pcd", *xyz_cloud_smoothed, false);//将点云保存到PCD文件中
  toPCLPointCloud2 (*xyz_cloud_smoothed, output);
}

void
saveCloud (const std::string &filename, const pcl::PCLPointCloud2 &output)
{
  TicToc tt;
  tt.tic ();

  print_highlight ("Saving "); print_value ("%s ", filename.c_str ());

  pcl::io::savePCDFile (filename, output,  Eigen::Vector4f::Zero (),
                        Eigen::Quaternionf::Identity (), false);

  print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points]\n");
}


int
main (int argc, char** argv)
{
  print_info ("Moving Least Squares smoothing of a point cloud. For more information, use: %s -h\n", argv[0]);

  if (argc < 3)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices;
  p_file_indices = parse_file_extension_argument (argc, argv, ".pcd");
  if (p_file_indices.size () != 2)
  {
    print_error ("Need one input PCD file and one output PCD file to continue.\n");
    return (-1);
  }

  // Command line parsing
  double search_radius = default_search_radius;
  double sqr_gauss_param = default_sqr_gauss_param;
  bool sqr_gauss_param_set = true;
  int polynomial_order = default_polynomial_order;

  parse_argument (argc, argv, "-radius", search_radius);
  parse_argument (argc, argv, "-polynomial_order", polynomial_order);
  if (parse_argument (argc, argv, "-sqr_gauss_param", sqr_gauss_param) == -1)
    sqr_gauss_param_set = false;

  // Load the first file
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);
  if (!loadCloud (argv[p_file_indices[0]], *cloud))
    return (-1);

  // Do the smoothing
  pcl::PCLPointCloud2 output;
  compute (cloud, output, search_radius, sqr_gauss_param_set, sqr_gauss_param, polynomial_order);

  // Save into the second file
  saveCloud (argv[p_file_indices[1]], output);
}
*/