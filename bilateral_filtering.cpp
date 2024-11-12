#include <pcl/point_types.h>
#include <pcl/filters/bilateral.h>
#include <pcl/io/pcd_io.h>

using namespace std;

int main(int argc, char* argv[]){

    if (argc != 3)
    {
        cout << "Usage: " << argv[0] << " <input.pcd> <output.pcd>" << endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1){
        PCL_ERROR("Could not read input file \n");
        return -1;
    }

    cout << "Loaded " << cloud->points.size() << " points from " << argv[1] << endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
    for(const auto& pt : cloud->points){
        pcl::PointXYZI pt_xyzi;
        pt_xyzi.x = pt.x;
        pt_xyzi.y = pt.y;
        pt_xyzi.z = pt.z;
        pt_xyzi.intensity = pt.z;
        cloud_xyzi->points.push_back(pt_xyzi);
    }
    cloud_xyzi->width = cloud->width;
    cloud_xyzi->height = cloud->height;

    pcl::BilateralFilter<pcl::PointXYZI> bilateral_filter;
    bilateral_filter.setInputCloud(cloud_xyzi);
    bilateral_filter.setHalfSize(5);
    bilateral_filter.setStdDev(0.5);
    bilateral_filter.filter(*cloud_filtered);

    cout << "Filtered cloud contains" << cloud_filtered->points.size() << "points." << endl;

    pcl::io::savePCDFileASCII(argv[2], *cloud_filtered);
    cout << "Saved filtered point cloud to" << argv[2] << endl;

    return 0;
}