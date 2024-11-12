#include <pcl/point_types.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

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

    pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> gaussian_kernel;
    gaussian_kernel.setSigma(0.2);
    gaussian_kernel.setThresholdRelativeToSigma(4);

    pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
    convolution.setInputCloud(cloud);
    convolution.setKernel(gaussian_kernel);
    convolution.setSearchMethod(boost::make_shared<pcl::search::KdTree<pcl::PointXYZ>>());
    convolution.setRadiusSearch(0.2);
    convolution.convolve(*cloud_filtered);

    cout << "Filtered cloud contains" << cloud_filtered->points.size() << "points." << endl;

    pcl::io::savePCDFileASCII(argv[2], *cloud_filtered);
    cout << "Saved filtered point cloud to" << argv[2] << endl;

    return 0;
}