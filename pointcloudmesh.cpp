

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/point_representation.h>
#include<pcl/pcl_base.h>
#include<pcl/pcl_config.h>
#include<pcl/pcl_exports.h>
#include<pcl/exceptions.h>
#include<pcl/common/common.h>	
#include<pcl/common/io.h>
#include<pcl/common/colors.h>
#include<pcl/pcl_macros.h>
#include<pcl/io/pcd_io.h>
#include<pcl/PCLPointField.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/passthrough.h>
#include<pcl/filters/voxel_grid_label.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/boost.h>
#include<pcl/visualization/point_cloud_color_handlers.h>  
#include<CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include<CGAL/grid_simplify_point_set.h>
#include<CGAL/IO/read_xyz_points.h>
#include<CGAL/property_map.h>
#include<CGAL/Simple_cartesian.h>
#include<CGAL/Polyhedron_3.h>
#include<CGAL/convex_hull_3.h>
#include<CGAL/Memory_sizer.h>
#include<CGAL/IO/write_ply_points.h>
#include<vector>
#include<fstream>
#include<utility>




// types
typedef CGAL::Exact_predicates_inexact_constructions_kernel  Kernel;  
typedef CGAL::Simple_cartesian<double>   Kernel1;
typedef CGAL::Polyhedron_3<Kernel1> Polyhedron;
typedef Kernel1::Point_3 Point;
typedef Kernel1::Vector_3 Vector;  

using namespace pcl;
using namespace boost;	   


int main(int argc, char** argv) {
	// create pcd objects
	PointCloud<PointXYZ> src;
	PointCloud<PointXYZ> dest;
	PointCloud<PointXYZ> outcloud; 	

	// load pcd data
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("src.pcd", src) == -1)
	{
		PCL_ERROR("read file error");
		return -EXIT_FAILURE;
	}
	   
	std::vector<Point> points;
	std::vector<Vector> normals;

	PointCloud<PointXYZ>::iterator begin = src.points.begin();	 
	for (; begin != src.end(); ++begin)
	{
		Point P (begin->x, begin->y, begin->z);
		points.push_back(P);
	}	
	std::cout << points.size() << " input points" << std::endl;
	std::vector<std::size_t> indices(points.size());

	for (std::size_t i = 0; i < points.size(); ++i) 
	{
		indices[i] = i;	 
	}

	double cell_size = 0.05;
	std::vector<std::size_t>::iterator end;
	end = CGAL::grid_simplify_point_set(indices.begin(),
		indices.end(),CGAL::make_property_map(points), cell_size);

	std::size_t k = end - indices.begin(); 
	std::cerr << "Keep " << k << " of " << indices.size() << "indices" << std::endl;
	{
		std::vector<Point> tmp_points(k);
		for (std::size_t i = 0; i<k; ++i) {
			tmp_points[i] = points[indices[i]];
		}
		points.swap(tmp_points);
	}

	std::cout << points.size() << " points after the simplification" << std::endl;	 

	Polyhedron Pol;   
	CGAL::convex_hull_3(points.begin(), points.end(), Pol);
	std::cout << "saving output to ply " << std::endl;	 	
	std::ofstream f("output.ply");
	CGAL::write_ply_points(f, points.begin(), points.end());  

	////visualization cloud data...
	//boost::shared_ptr<visualization::PCLVisualizer> cloudView(new visualization::PCLVisualizer("3D Viewer"));
	//cloudView->initCameraParameters();

	//int v1(0);
	//cloudView->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	//cloudView->setBackgroundColor(0.0, 0.228, 0.342, v1);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb1(src, 250, 255, 210);
	//cloudView->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
	//cloudView->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_LUT, 1.0f, "Cloud", v1);
	//cloudView->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 1.0f, "Cloud", v1);
	//cloudView->addPointCloud<pcl::PointXYZ>(src, "SrcCloud", v1);

	//int v2(0);
	//cloudView->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	//cloudView->setBackgroundColor(0.0, 0.229, 0.341, v2);
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb2(outcloud, 150, 255, 250);
	//cloudView->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE, 1, "OutCloud");
	//cloudView->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_LUT, 1.0f, "OutCloud", v2);
	//cloudView->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 1.0f, "OutCloud", v2);
	//cloudView->addPointCloud<pcl::PointXYZ>(outcloud, "OutCloud", v2);
	//cloudView->setPosition(120, 10);
	//cloudView->setSize(1000, 750);
	//cloudView->setShowFPS(true);
	//cloudView->setWindowName("Point reduction Filter");

	//while (!cloudView->wasStopped())
	//{
	//	cloudView->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	//}

	//cloudView->close();		

	return EXIT_SUCCESS;
}

