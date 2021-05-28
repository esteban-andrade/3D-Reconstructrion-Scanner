#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <pcl/common/common.h>
#include <pcl/io/obj_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/flann.h>
#include <pcl/kdtree/impl/io.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/surface/mls.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <pcl/pcl_macros.h>
#include <pcl/conversions.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/impl/vtk_lib_io.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <chrono>
#include <ctime>
#include <boost/thread/thread.hpp>
#include <flann/flann.hpp>
#include <pcl/common/pca.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/integral_image_normal.h>
#include <unsupported/Eigen/MatrixFunctions>
using namespace pcl;

class Reconstruction
{
private:
    bool type_of_imported_file; // 0 = obj 1= ply
    std::string mesh_path;

    pcl::PolygonMesh mesh; //original mesh
    pcl::PLYReader reader_ply;
    pcl::PLYWriter writer_ply;
    pcl::OBJReader reader_obj;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> meshes_for_scale_;
    std::vector<double> scale_;

public:
    Reconstruction();
    Reconstruction(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, std::string); //Only for ply file.
    ~Reconstruction();
    void setInputCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, std::string, bool);
    void normalsDirectionAdjustment(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PolygonMesh::Ptr &);
    void pointCloudOutliersFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);
    void pointCloudPlaneSegmentOperation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);
    void pointCloudClustering(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);
    void pointCloudNormaliseUpscale(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);
    void poissonReconstruction(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PolygonMesh::Ptr &);
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> pointCloudScaleAdjustment(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, bool, bool); //first object is target pcl and second one is reference
    std::vector<double> getScale();
    void cloudToPolyMesh(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &,
                         pcl::PolygonMesh::Ptr &);
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> manualScaleAdjuster(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, double);
    void convertPLYtoPCL(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, std::string);
    void convertPolyMeshtoPLY(pcl::PolygonMesh::Ptr &, std::string);
    void downsamplePointCloudVoxelGrid(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, float, float, float);
    void convertPCLtoPLY(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, std::string);
    void importPLYtoPolymesh(std::string, pcl::PolygonMesh::Ptr&);
    void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, std::string, float, float);
    void pointCloudOutliersFilterManual(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, int, double);
    void pointCloudPlaneSegmentOperationManual(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, double, double);
    void pointCloudCentroidAlignmentAdjusment(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, double, double, double);
    void normalsReAdjustement(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);
    void normalsTesting(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);
    void transformPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);
    bool transformVerification(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);
};
