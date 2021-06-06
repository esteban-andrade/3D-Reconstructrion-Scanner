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
     * @brief This API aims to smooth the surface of the PointCloud via MLS (MovingLeastSquares).
     * @note Furthermore, it also upsamples the point cloudto a defined parameter usingRANDOM_UNIFORM_DENSITY as the prefered method. This specific API uses a Kdtree for data searching and it is required to specify theSearch Radiusfor determiningthe k-nearest neighbor for best fitting. This method required an input point cloud,and it will output a smoothed and upsampled point cloud
     * 
     * @param reference_cloud: input cloud
     * @param target_cloud: output cloud 
     */
    void pointCloudNormaliseUpscale(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);

    /**
     * @brief his API will require an input pointcloud and it will generatea reconstructed Polymesh based on the Poisson Reconstruction Algorithm
    *  @param reference_cloud: input cloud
     * @param target_mesh: output mesh  
     */
    void poissonReconstruction(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PolygonMesh::Ptr &);

    /**
     * @brief 
     * 
     * @param input_cloud_1: Target Point cloud
     * @param input_cloud_2: Reference Point cloud
     * @param adjuster the initial pointcloud will be the one that is adjusted Selection
     * @param primaryAxisOnly Axis Selection(Single or multiple)
     * @return std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> of Scaled Point Cloud. The first element is the scaled cloud whereas the second one is the refence point cloud.
     */
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> pointCloudScaleAdjustment(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, bool, bool); //first object is target pcl and second one is reference
    
    /**
     * @brief Get the Scale object
     * 
     * @return std::vector<double> 
     */
    std::vector<double> getScale();

    /**
     * @brief Convert point cloud to polymesh
     * @param ref_cloud Reference Point cloud
     * @param mesh outout mesh
     */
    void cloudToPolyMesh(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &,
                         pcl::PolygonMesh::Ptr &);

    /**
     * @brief Manual Adjuster method
     * @param input_cloud Input Point cloud
     * @param scale required scale
     * @return std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>  output cloud
     */
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> manualScaleAdjuster(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, double);

    /**
     * @brief Convert point cloud to ply file
     * @param target_clopud output Point cloud
     * @param ply_file File and path of file
     */
    void convertPLYtoPCL(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, std::string);

    /**
     * @brief Convert polymesh to ply file
     * @param input input Point cloud
     * @param target_path File and path of file
     */
    void convertPolyMeshtoPLY(pcl::PolygonMesh::Ptr &, std::string);

    /**
     * @brief This API aims to downsample the input point-cloud through a voxel grid filter
     * 
     * @param input_cloud input Point cloud
     * @param target_cloud Output point cloud
     * @param lx x dimension of voxel
     * @param ly y dimension of voxel
     * @param lz x dimension of voxel
     */
    void downsamplePointCloudVoxelGrid(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, float, float, float);

    /**
     * @brief Convert point cloud to ply file
     * @param input input Point cloud
     * @param target_path File and path of file 
     */
    void convertPCLtoPLY(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, std::string);

    /**
     * @brief Imports ply file to polymesh
     * @param path path of ply file 
     * @param target target polymesh
     */
    void importPLYtoPolymesh(std::string, pcl::PolygonMesh::Ptr&);

    /**
     * @brief This API aims to filter a specific point cloud based on the specificaxis limit, which is known as passThroughFilter
     * @param reference_cloud: input cloud
     * @param target_cloud: output cloud 
     * @param field_name axis
     * @param min_limit Minimum limit
     * @param max_limit Maximum Limit
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
     * @brief This API is designed to Input a specific point cloud and runan Euclidean Cluster Extraction (
     * @note The Clusterized point cloudwill output the biggest point cloud cluster as the valid target point cloud. In orderto define a cluster, it is crucial to determine theMinimum Cluster Size and theCluster Tolerance.Once all the clusters are generated, these will be stacked in avector. In this vector, an algorithm will iterate throughout all elements and comparetheir cluster size. Based on this process, it will output the largest cluster as the validoutput point cloud
     * 
     * @param reference_cloud: input cloud
     * @param target_cloud: output cloud
     * @param threshold Threshold value determination
     * @param iteration Number of iterations
     */
    void pointCloudPlaneSegmentOperationManual(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, double, double);

    /**
     * @brief This API aims to align two given pointcloud to its corresponding centroi
     * @param reference_cloud: input cloud
     * @param target_cloud: output cloud
     * @param x_axis x axis radian adjustment
     * @param y_axis y axis radian adjustment
     * @param z_axis z axis radian adjustment
     */
    void pointCloudCentroidAlignmentAdjusment(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, double, double, double);

    /**
     * @brief Normalizes the normals
     * @param reference_cloud: input cloud
     */
    void normalsReAdjustement(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);

    /**
     *  @brief Adjuste normals
     *  @param reference: input cloud
     *  @param target: output cloud
     */
    void normalsTesting(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);

    /**
     *  @brief This API will transform one point cloud based on the input reference point cloud and use that transform pose.
     *  @param reference_for_transform: input cloud. The transform will be extracted from here
     *  @param reference: Input cloud
     *  @param target: output cloud
     */
    void transformPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);

    /**
     * @brief It will verify the transform and check based on a given thershold
     * 
     * @param cloud1:  cloud 1
     * @param cloud2: cloud 2
     * @return true 
     * @return false 
     */
    bool transformVerification(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &);
};
