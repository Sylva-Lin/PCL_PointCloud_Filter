#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

int main(int argc, char *argv[])
{

    if (argc != 3)
    {
        cout << "Usage: " << argv[0] << " <input.pcd> <output.pcd>" << endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
    {
        PCL_ERROR("Could not read input file \n");
        return -1;
    }
    cout << "Loaded" << cloud->points.size() << "points from" << argv[1] << endl;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    cout << "Filtered cloud contains" << cloud_filtered->points.size() << "points." << endl;

    pcl::io::savePCDFileASCII(argv[2], *cloud_filtered);
    cout << "Saved filtered point cloud to" << argv[2] << endl;

    return 0;
}