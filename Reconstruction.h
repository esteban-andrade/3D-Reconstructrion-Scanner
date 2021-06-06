/**
 * @file Reconstruction.h
 * @author Esteban Andrade
 * @brief This is the main class be used to hold all the API used for data processing.
 * @version 0.1
 * @date 2021-06-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

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


/**
 * @brief The Class Reconstruction is used process the target pointcloud. It contain many implemeted algorithms used for filtering, segmentation, clusteting, etc.
 * 
 */
class Reconstruction
{
    /**
     * @privatesection
     * 
     */
private:
    bool type_of_imported_file;     //!< Boolean use to determine the type of file  0 = obj 1= ply
    std::string mesh_path;          //!<  String for mesh path
    pcl::PolygonMesh mesh;          //!< Original Input mesh
    pcl::PLYReader reader_ply;      //!< object used to read ply files
    pcl::PLYWriter writer_ply;      //!< Object used to write ply files
    pcl::OBJReader reader_obj;       //!< Object used to read obj files
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> meshes_for_scale_; //!< vector that holds the scale object and the reference file
    std::vector<double> scale_;                                                  //!< vectot that holds scale in all axis

public:
/**
 * @brief Construct a new Reconstruction object
 * 
 */
    Reconstruction();

    /**
     * @brief Construct a new Reconstruction object
     * @note Only for ply file.
     * @param cloud: pointcloud point cloud used to hold the input mesh
     * @param path_file : string used to hold the path of the input file
     */
    Reconstruction(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, std::string);

    /**
     * @brief Destroy the Reconstruction object
     * 
     */
    ~Reconstruction();

    /**
     * @brief Set the Input Cloud object
     * @note This member class function will be used to input a given Mesh orPointCloud. It designed to work with common.ply & .objformats
     * @param cloud Point cloud used input the point cloud
     * @param path_file : string used to hold the path of the input file
     * @param mesh_type File type (0 obj 1 ply file)
     * 
     */
    void setInputCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, std::string, bool);

    /**
     * @brief Adjust the normals of the point cloud
     * @note This method is designed to input a given Pointcloudand output a different pointcloud with Adjusted Normals. It will get the vertices ofeach face and recompute the normals using the"right hand rule". Furthermore, itwill normalize the normal vector and stack up the new normals in the output pointcloud.
     * @param reference_cloud: input cloud
     * @param target_cloud: output cloud
     * @param mesh: Polymesh used to adjust normals
     */
    void normalsDirectionAdjustment(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PolygonMesh::Ptr &);

    /**
     * @brief This API takes an reference cloud and outputs a processedpoint cloud with filtered data outliers
     * @note This method is based on the StatisticalOutlierRemoval Filter algoritm
     * @param reference_cloud: input cloud
     * @param target_cloud: output cloud
     * 
     */
    void pointCloudOutliersFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);

    /**
     * @brief this API is degined to input a given PointCloud and output a plane segmented point cloud.
     * @note The logic behing this algorithm is based RANSAC. Similarly it uses a Kdtree to search through the pointcloud. Also it uses RANSAC as the segmentation method and Model Type.
     * @param reference_cloud: input cloud
     * @param target_cloud: output cloud 
     */
    void pointCloudPlaneSegmentOperation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);

    /**
     * @brief This API is designed to Input a specific point cloud and runan Euclidean Cluster Extraction (
     * @note The Clusterized point cloudwill output the biggest point cloud cluster as the valid target point cloud. In orderto define a cluster, it is crucial to determine theMinimum Cluster Size and theCluster Tolerance.Once all the clusters are generated, these will be stacked in avector. In this vector, an algorithm will iterate throughout all elements and comparetheir cluster size. Based on this process, it will output the largest cluster as the validoutput point cloud
     * 
     * @param reference_cloud: input cloud
     * @param target_cloud: output cloud
     */
    void pointCloudClustering(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);

    /**
     * @brief 
     * 
     * 
     * @param reference_cloud: input cloud
     * @param target_cloud: output cloud 
     */
    void pointCloudNormaliseUpscale(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);

    /**
     * @brief 
     * 
     */
    void poissonReconstruction(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PolygonMesh::Ptr &);

    /**
     * @brief 
     * 
     * @return std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> 
     */
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> pointCloudScaleAdjustment(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, bool, bool); //first object is target pcl and second one is reference
    
    /**
     * @brief Get the Scale object
     * 
     * @return std::vector<double> 
     */
    std::vector<double> getScale();

    /**
     * @brief 
     * 
     */
    void cloudToPolyMesh(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &,
                         pcl::PolygonMesh::Ptr &);

    /**
     * @brief 
     * 
     * @return std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> 
     */
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> manualScaleAdjuster(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, double);
    
    /**
     * @brief 
     * 
     */
    void convertPLYtoPCL(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, std::string);
    
    /**
     * @brief 
     * 
     */
    void convertPolyMeshtoPLY(pcl::PolygonMesh::Ptr &, std::string);
    
    /**
     * @brief 
     * 
     */
    void downsamplePointCloudVoxelGrid(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, float, float, float);
    
    /**
     * @brief 
     * 
     */
    void convertPCLtoPLY(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, std::string);
    
    /**
     * @brief 
     * 
     */
    void importPLYtoPolymesh(std::string, pcl::PolygonMesh::Ptr&);
    
    
    /**
     * @brief 
     * 
     */
    void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, std::string, float, float);

    /**
     * @brief This API takes an reference cloud and outputs a processedpoint cloud with filtered data outliers
     * @note This method is based on the StatisticalOutlierRemoval Filter algoritm
     *  @param reference_cloud: input cloud
     * @param target_cloud: output cloud
     * @param mean_k  configures the number of points (K) to use for mean distanceestimation.
     * @param std_threshold  Sets the standard deviation multipler threshold
     */
    void pointCloudOutliersFilterManual(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, int, double);
    
    /**
     * @brief 
     * 
     */
    void pointCloudPlaneSegmentOperationManual(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, double, double);
    
    /**
     * @brief 
     * 
     */
    void pointCloudCentroidAlignmentAdjusment(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, double, double, double);
    
    /**
     * @brief 
     * 
     */
    void normalsReAdjustement(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);
    
    /**
     * @brief 
     * 
     */
    void normalsTesting(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);
    
    /**
     * @brief 
     * 
     */
    void transformPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);
    
    /**
     * @brief 
     * 
     * @return true 
     * @return false 
     */
    bool transformVerification(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);
};
