#include <pcl/point_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>

using namespace std;

int main(int argc, char* argv[]){

    if (argc != 3)
    {
        cout << "Usage: " << argv[0] << " <input.pcd> <output.pcd>" << endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1){
        PCL_ERROR("Could not read input file \n");
        return -1;
    }

    cout << "Loaded " << cloud->points.size() << " points from " << argv[1] << endl;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    coefficients->values.resize(4);
    coefficients->values[0] = 0.0;
    coefficients->values[1] = 0.0;
    coefficients->values[2] = 1.0;
    coefficients->values[3] = 0.0;

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_filtered);

    cout << "Filtered cloud contains" << cloud_filtered->points.size() << "points." << endl;

    pcl::io::savePCDFileASCII(argv[2], *cloud_filtered);
    cout << "Saved filtered point cloud to" << argv[2] << endl;

    return 0;
}