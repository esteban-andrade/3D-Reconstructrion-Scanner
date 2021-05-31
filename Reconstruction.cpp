#include "Reconstruction.h"

static const int MEAN_K = 100;
static const double STD_DEV_MULTRESH = 2.5; //0.9 //2.5 //1.5
/*************************************************/
static const double PLANE_SEGMENTATION_THRESHOLD = 0.001; //0.01 // 0.001
static const double PLANE_SEGMENTATION_ITER = 10000;      //10000
/*************************************************/
static const double CLUSTER_TOLERANCE = 0.02;
static const int MIN_CLUSTER_SIZE = 100;
/**************************************************/
static const int MLS_POLYNOMIAL_ORDER = 2;
static const double MLS_SEARCH_RADIUS = 0.01;          //0.01
static const int UPSAMPLE_DESIRED_POINT_DENSITY = 100; //100
/**************************************************/
static const int POISSON_DEPTH = 7;     //10 //7
static const int POISSON_MIN_DEPTH = 5; //7 //5
static const float POISSON_SAMPLES_PER_NODE = 30.0f;
static const int POISSON_SOLVER_DIVIDER = 8;
static const int POISSON_ISO_DIVIDER = 8;
static const int POISSON_DEGREE = 2;
static const float POISSON_POINT_WEIGHT = 2.0f;
static const float POISSON_SET_SCALE = 1.0f;
/*************************************************/
static const double pose_error_cm_threshold = 10;

const std::string
currentDateTime()
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}
Reconstruction::Reconstruction() {}

Reconstruction::~Reconstruction() {}

Reconstruction::Reconstruction(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud, std::string path_file)
{

#if OS == 1
    mesh_path = path_file;

    std::cout << "\n"
              << currentDateTime() << "\tWINDOWS DETECTED\n"
              << std::endl;

#elif OS == 0

    mesh_path = path_file;
    std::cout << "\n"
              << currentDateTime() << "\tUBUNTU DETECTED\n"
              << std::endl;
#endif

    // pcl::io::loadPolygonFilePLY(mesh_path, mesh);
    // std::cout << currentDateTime() << "\t.ply File Imported\n"
    //           << std::endl;

    reader_ply.read(mesh_path, mesh);
    std::cout << currentDateTime() << "\t.ply File Imported\n"
              << std::endl;
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
}

void Reconstruction::setInputCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud, std::string path_file, bool mesh_type)
{
#if OS == 1
    mesh_path = "path_file";

    std::cout << "\n"
              << currentDateTime() << "\n\tWINDOWS DETECTED\n"
              << std::endl;

#elif OS == 0

    mesh_path = "path_file";
    std::cout << "\n"
              << currentDateTime() << "\n\tUBUNTU DETECTED\n"
              << std::endl;
#endif

    if (mesh_type == 0)
    {
        pcl::io::loadOBJFile(mesh_path, mesh);
        std::cout << currentDateTime() << "\n\t.obj File Imported\n"
                  << std::endl;
    }
    else if (mesh_type == 1)
    {
        pcl::io::loadPolygonFilePLY(mesh_path, mesh);
        std::cout << currentDateTime() << "\n\t.ply File Imported\n"
                  << std::endl;
    }
    else
    {
        std::cout << "\nNo Valid Input" << std::endl;
    }

    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    std::cout << currentDateTime() << "\tLoaded "
              << cloud->width * cloud->height
              << " data points from Mesh file with the following fields: \t"
              << std::endl;
}

void Reconstruction::normalsDirectionAdjustment(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &reference_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &target_cloud, pcl::PolygonMesh::Ptr &mesh)
{

    std::cout << "\n"
              << currentDateTime() << "\tADJUSTING NORMALS " << std::endl;

    std::vector<int> normalization;
    pcl::Normal normal_to_add;
    target_cloud = reference_cloud;
    for (int i = 0; i < target_cloud->points.size(); i++)
    {
        PointXYZRGBNormal point = target_cloud->points[i];
        target_cloud->at(i).normal_x = 0;
        target_cloud->at(i).normal_y = 0;
        target_cloud->at(i).normal_z = 0;
        normalization.push_back(0);
        normalization.shrink_to_fit();
    }

    for (int i = 0; i < target_cloud->size(); i++)
    {
        unsigned int n1 = mesh->polygons[i].vertices[0];
        unsigned int n2 = mesh->polygons[i].vertices[1];
        unsigned int n3 = mesh->polygons[i].vertices[2];

        // get the vertices coordinates of the face
        Eigen::Vector3f point_1 = target_cloud->points[n1].getVector3fMap();
        Eigen::Vector3f point_2 = target_cloud->points[n2].getVector3fMap();
        Eigen::Vector3f point_3 = target_cloud->points[n3].getVector3fMap();

        // compute the normal using the "right-hand rule"
        Eigen::Vector3f vector_1 = point_2 - point_1;
        Eigen::Vector3f vector_2 = point_3 - point_1;
        Eigen::Vector3f face_normal = vector_1.cross(vector_2);

        // normalize the normal vector
        face_normal = face_normal / face_normal.norm();

        // stack up the new normals

        target_cloud->points[n1].normal_x += face_normal[0];
        target_cloud->points[n1].normal_y += face_normal[1];
        target_cloud->points[n1].normal_z += face_normal[2];

        target_cloud->points[n2].normal_x += face_normal[0];
        target_cloud->points[n2].normal_y += face_normal[1];
        target_cloud->points[n2].normal_z += face_normal[2];

        target_cloud->points[n3].normal_x += face_normal[0];
        target_cloud->points[n3].normal_y += face_normal[1];
        target_cloud->points[n3].normal_z += face_normal[2];

        normalization[n1]++;
        normalization[n2]++;
        normalization[n3]++;
    }

    for (int i = 0; i < target_cloud->size(); i++)
    {
        target_cloud->points[i].normal_x /= normalization[i];
        target_cloud->points[i].normal_y /= normalization[i];
        target_cloud->points[i].normal_z /= normalization[i];
    }

    std::cout << currentDateTime() << "\tLoaded "
              << target_cloud->width * target_cloud->height
              << " data points from Mesh Normalized with the following fields: \t"
              << std::endl;
    normalization.clear();
}

void Reconstruction::pointCloudOutliersFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &reference_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &target_cloud)
{
    std::cout << "\n"
              << currentDateTime() << "\tFILTERING " << std::endl;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal> filter;
    filter.setInputCloud(reference_cloud);
    filter.setMeanK(MEAN_K);
    filter.setStddevMulThresh(STD_DEV_MULTRESH);
    filter.filter(*target_cloud);
    filter.setNegative(false);
    filter.filter(*target_cloud);
    std::cout
        << currentDateTime() << "\tFiltered "
        << target_cloud->width * target_cloud->height
        << " data points from Filtered PointCloud with the following fields: \t"
        << std::endl;
}
void Reconstruction::pointCloudOutliersFilterManual(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &reference_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &target_cloud, int mean_k, double std_threshold)
{
    std::cout << "\n"
              << currentDateTime() << "\tFILTERING " << std::endl;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal> filter;
    filter.setInputCloud(reference_cloud);
    filter.setMeanK(mean_k);
    filter.setStddevMulThresh(std_threshold);
    filter.filter(*target_cloud);
    filter.setNegative(false);
    filter.filter(*target_cloud);
    std::cout
        << currentDateTime() << "\tFiltered "
        << target_cloud->width * target_cloud->height
        << " data points from Filtered PointCloud with the following fields: \t"
        << std::endl;
}

void Reconstruction::pointCloudPlaneSegmentOperation(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &reference_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &target_cloud)
{
    std::cout << "\n"
              << currentDateTime() << "\tSEGMENTING " << std::endl;
    pcl::NormalEstimation<PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
    pcl::SACSegmentationFromNormals<PointXYZRGBNormal, pcl::PointXYZRGBNormal> seg;
    pcl::ExtractIndices<PointXYZRGBNormal> extract;
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    static const double threshold = PLANE_SEGMENTATION_THRESHOLD;
    ne.setSearchMethod(tree2);
    ne.setInputCloud(reference_cloud);
    ne.setKSearch(MEAN_K); //50
    ne.compute(*cloud_normals);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(threshold);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(PLANE_SEGMENTATION_ITER);
    seg.setDistanceThreshold(threshold);
    seg.setInputCloud(reference_cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers_plane, *coefficients_plane);
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
    // Extract the planar inliers from the input cloud
    extract.setInputCloud(reference_cloud);
    extract.setIndices(inliers_plane);
    extract.setNegative(true);
    extract.filter(*target_cloud);
    std::cerr << currentDateTime() << "\tPointCloud without planar component: " << target_cloud->size() << " data points.\t" << std::endl;
}
void Reconstruction::pointCloudPlaneSegmentOperationManual(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &reference_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &target_cloud, double threshold, double iteration)
{
    std::cout << "\n"
              << currentDateTime() << "\tSEGMENTING " << std::endl;
    pcl::NormalEstimation<PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
    pcl::SACSegmentationFromNormals<PointXYZRGBNormal, pcl::PointXYZRGBNormal> seg;
    pcl::ExtractIndices<PointXYZRGBNormal> extract;
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

    ne.setSearchMethod(tree2);
    ne.setInputCloud(reference_cloud);
    ne.setKSearch(MEAN_K); //50
    ne.compute(*cloud_normals);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(threshold);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(iteration);
    seg.setDistanceThreshold(threshold);
    seg.setInputCloud(reference_cloud);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers_plane, *coefficients_plane);
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
    // Extract the planar inliers from the input cloud
    extract.setInputCloud(reference_cloud);
    extract.setIndices(inliers_plane);
    extract.setNegative(true);
    extract.filter(*target_cloud);
    std::cerr << currentDateTime() << "\tPointCloud without planar component: " << target_cloud->size() << " data points.\t" << std::endl;
}

void Reconstruction::pointCloudClustering(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &reference_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &target_cloud)
{
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree3(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> ec;
    tree3->setInputCloud(reference_cloud);
    ec.setClusterTolerance(CLUSTER_TOLERANCE);
    ec.setMinClusterSize(MIN_CLUSTER_SIZE);
    ec.setSearchMethod(tree3);
    ec.setInputCloud(reference_cloud);
    ec.extract(cluster_indices);
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> clusters;
    clusters.clear();
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->push_back((*reference_cloud)[*pit]); //*
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        clusters.push_back(cloud_cluster);
        j++;
    }
    clusters.shrink_to_fit();
    std::cout << "Number of Clusters: " << clusters.size() << "\nChoosing Biggest Cluster as Valid Point Cloud" << std::endl;
    for (size_t i = 0; i < clusters.size(); i++)
    {
        target_cloud = clusters.at(0);
        if (target_cloud->size() <= clusters.at(i)->size())
        {
            target_cloud = clusters.at(i);
            std::cout << "Valid Cluster # " << i + 1 << "\n"
                      << currentDateTime() << "\tAssignning Cluster as Valid Point Cloud" << std::endl;
        }
    }

    clusters.clear();
}

void Reconstruction::pointCloudNormaliseUpscale(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &reference_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &target_cloud)
{
    std::cout << "\n"
              << currentDateTime() << "\tNORMALIZING & UPSCALING" << std::endl;

    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    pcl::MovingLeastSquares<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> mls;
    mls.setComputeNormals(false);
    //static const int polynomial_order = 2;
    static const double search_radius = MLS_SEARCH_RADIUS; //0.05
    static const double sqr_gauss_param = search_radius * search_radius;
    static const int desired_point_density = UPSAMPLE_DESIRED_POINT_DENSITY; //100000

    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::UpsamplingMethod::RANDOM_UNIFORM_DENSITY);
    mls.setInputCloud(reference_cloud);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(MLS_POLYNOMIAL_ORDER);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(search_radius);
    mls.setSqrGaussParam(sqr_gauss_param);
    mls.setPointDensity(desired_point_density);
    mls.process(*target_cloud);
    std::cout << currentDateTime() << "\tSmoothed and Upsampled "
              << target_cloud->width * target_cloud->height
              << " data points from Smoothed file with the following fields: \t"
              << std::endl;
}

void Reconstruction::poissonReconstruction(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &reference_cloud, pcl::PolygonMesh::Ptr &target_mesh)
{
    std::cout
        << "\n"
        << currentDateTime() << "\tPoission  Reconstruction STARTED " << std::endl;
    Poisson<pcl::PointXYZRGBNormal>
        poisson;
    poisson.setDepth(POISSON_DEPTH);
    poisson.setManifold(true);
    poisson.setOutputPolygons(true);
    poisson.setMinDepth(POISSON_MIN_DEPTH);
    poisson.setSamplesPerNode(POISSON_SAMPLES_PER_NODE);
    poisson.setSolverDivide(POISSON_SOLVER_DIVIDER);
    poisson.setIsoDivide(POISSON_ISO_DIVIDER);
    poisson.setDegree(POISSON_DEGREE);
    poisson.setPointWeight(POISSON_POINT_WEIGHT);
    poisson.setScale(POISSON_SET_SCALE);
    poisson.setOutputPolygons(true);

    //poisson.setConfidence(true);

    poisson.setInputCloud(reference_cloud);
    poisson.performReconstruction(*target_mesh);
    std::cout << "\n"
              << currentDateTime() << "\tPoission Valid" << std::endl;
}

bool Reconstruction::transformVerification(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud1, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud2)
{

    std::cout
        << "\n"
        << currentDateTime() << "\tANALYSING............... " << std::endl;

    pcl::PCA<pcl::PointXYZRGBNormal> cloud_analysis1;
    cloud_analysis1.setInputCloud(cloud1);
    Eigen::Matrix3f cloud_analysis1_eigen_values_vector = cloud_analysis1.getEigenVectors();
    Eigen::Vector4f cloud_analysis1_mid_point = cloud_analysis1.getMean();
    Eigen::Matrix4f cloud_analysis1_transform = Eigen::Matrix4f::Identity();
    cloud_analysis1_transform.block<3, 3>(0, 0) = cloud_analysis1_eigen_values_vector;
    cloud_analysis1_transform.block<4, 1>(0, 3) = cloud_analysis1_mid_point;
    std::cout
        << "\n"
        << currentDateTime() << "\tTransform 1\n"
        << cloud_analysis1_transform << std::endl;

    pcl::PCA<pcl::PointXYZRGBNormal> cloud_analysis2;
    cloud_analysis2.setInputCloud(cloud2);
    Eigen::Matrix3f cloud_analysis2_eigen_values_vector = cloud_analysis2.getEigenVectors();
    Eigen::Vector4f cloud_analysis2_mid_point = cloud_analysis2.getMean();
    Eigen::Matrix4f cloud_analysis2_transform = Eigen::Matrix4f::Identity();
    cloud_analysis2_transform.block<3, 3>(0, 0) = cloud_analysis2_eigen_values_vector;
    cloud_analysis2_transform.block<4, 1>(0, 3) = cloud_analysis2_mid_point;

    std::cout
        << "\n"
        << currentDateTime() << "\tTransform 2\n"
        << cloud_analysis2_transform << std::endl;

    Eigen::Vector4f cloud_analysis_translation = cloud_analysis2_mid_point - cloud_analysis1_mid_point;
    double translation_error = cloud_analysis_translation.norm();

    Eigen::Matrix3f cloud_analysis_rotation = cloud_analysis1_eigen_values_vector * cloud_analysis2_eigen_values_vector.inverse();
    Eigen::MatrixLogarithmReturnValue<Eigen::Matrix3f> cloud_analysis_rotation_log = cloud_analysis_rotation.log();
    double rotation_error = cloud_analysis_rotation_log.norm();

    Eigen::Matrix4f transform_analysis = cloud_analysis1_transform * cloud_analysis2_transform.inverse();
    std::cout
        << "\n"
        << currentDateTime() << "\tTranslation Error in cm= " << translation_error * 100 << " \t Rotation Error in cm = "
        << rotation_error * 100 << " \n"
        << "\n\t\t T1*T2` = \n"
        << transform_analysis
        << std::endl;
    if ((translation_error * 100) < pose_error_cm_threshold && (rotation_error * 100) < pose_error_cm_threshold)
    {
        std::cout
            << "\n"
            << currentDateTime() << "\tSUCESSFULL..............\n"
            << std::endl;
        return true;
    }
    else
    {
        std::cout
            << "\n"
            << currentDateTime() << "\tFAILED..............\n"
            << std::endl;
        return false;
    }
}
void Reconstruction::transformPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &reference_for_transform, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &reference, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &target)
{
    pcl::PCA<pcl::PointXYZRGBNormal> cloud;
    cloud.setInputCloud(reference_for_transform);
    Eigen::Matrix3f cloud_eigen_values_vector = cloud.getEigenVectors();
    Eigen::Vector4f cloud_mid_point = cloud.getMean();
    Eigen::Matrix4f cloud_transform = Eigen::Matrix4f::Identity();
    cloud_transform.block<3, 3>(0, 0) = cloud_eigen_values_vector;
    cloud_transform.block<4, 1>(0, 3) = cloud_mid_point;

    pcl::transformPointCloud(*reference, *target, cloud_transform);
}
std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> Reconstruction::pointCloudScaleAdjustment(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &input_cloud_1, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &input_cloud_2, bool adjuster, bool primaryAxisOnly)
{
    std::cout << "\n"
              << currentDateTime() << "\tStarting SCALING" << std::endl;
    pcl::PCA<pcl::PointXYZRGBNormal> cloud_1;
    cloud_1.setInputCloud(input_cloud_1);
    Eigen::Matrix3f cloud1_eigen_values_vector = cloud_1.getEigenVectors();
    Eigen::Vector4f cloud1_mid_point = cloud_1.getMean();
    Eigen::Matrix4f cloud1_transform = Eigen::Matrix4f::Identity();
    cloud1_transform.block<3, 3>(0, 0) = cloud1_eigen_values_vector;
    cloud1_transform.block<4, 1>(0, 3) = cloud1_mid_point;

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_1_adjusted(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::transformPointCloud(*input_cloud_1, *cloud_1_adjusted, cloud1_transform.inverse());
    pcl::PointXYZRGBNormal cloud1_min, cloud1_max;
    pcl::getMinMax3D(*cloud_1_adjusted, cloud1_min, cloud1_max);

    //analyze pointcloud2
    pcl::PCA<pcl::PointXYZRGBNormal>
        cloud_2;
    cloud_2.setInputCloud(input_cloud_2);
    Eigen::Matrix3f cloud2_eigen_values_vector = cloud_2.getEigenVectors();
    Eigen::Vector4f cloud2_mid_point = cloud_2.getMean();
    Eigen::Matrix4f cloud2_transform = Eigen::Matrix4f::Identity();
    cloud2_transform.block<3, 3>(0, 0) = cloud2_eigen_values_vector;
    cloud2_transform.block<4, 1>(0, 3) = cloud2_mid_point;

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud2_adjusted(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::transformPointCloud(*input_cloud_2, *cloud2_adjusted, cloud2_transform.inverse());
    pcl::PointXYZRGBNormal cloud2_min, cloud2_max;
    pcl::getMinMax3D(*cloud2_adjusted, cloud2_min, cloud2_max);
    //apply scaling to oriented reference pointcloud
    double xScale = (cloud2_max.x - cloud2_min.x) / (cloud1_max.x - cloud1_min.x);
    double yScale = (cloud2_max.y - cloud2_min.y) / (cloud1_max.y - cloud1_min.y);
    double zScale = (cloud2_max.z - cloud2_min.z) / (cloud1_max.z - cloud1_min.z);

    if (primaryAxisOnly)
    {
        std::cout << "\n"
                  << currentDateTime() << "\tscale: " << xScale << std::endl;
        scale_.push_back(xScale);
    }
    else
    {
        std::cout << "\n"
                  << currentDateTime() << "\txScale: " << xScale << "\tyScale: " << yScale << "\tzScale: " << zScale << std::endl;
        scale_.push_back(xScale);
        scale_.push_back(yScale);
        scale_.push_back(zScale);
    }

    for (int i = 0; i < cloud_1_adjusted->points.size(); i++)
    {
        if (primaryAxisOnly)
        {
            cloud_1_adjusted->points[i].x = cloud_1_adjusted->points[i].x * xScale;
            cloud_1_adjusted->points[i].y = cloud_1_adjusted->points[i].y * xScale;
            cloud_1_adjusted->points[i].z = cloud_1_adjusted->points[i].z * xScale;
        }
        else
        {
            cloud_1_adjusted->points[i].x = cloud_1_adjusted->points[i].x * xScale;
            cloud_1_adjusted->points[i].y = cloud_1_adjusted->points[i].y * yScale;
            cloud_1_adjusted->points[i].z = cloud_1_adjusted->points[i].z * zScale;
        }
    }

    if (adjuster)
    {
        meshes_for_scale_.push_back(cloud_1_adjusted); // the initial pointcloud will be the one that is adjusted
        meshes_for_scale_.push_back(cloud2_adjusted);
        std::cout << "\n"
                  << currentDateTime() << "\tTarget  Pointclouds Scaled and Aligned and stored" << std::endl;
    }
    else
    {

        meshes_for_scale_.push_back(cloud_1_adjusted); // we will be retrieving the first input pointcloud adjusted
        std::cout << "\n"
                  << currentDateTime() << "\tTarget Pointcloud Scaled and stored" << std::endl;
    }
    std::cout << "\n"
              << currentDateTime() << "\tSCALING FINISHED\n"
              << std::endl;
    meshes_for_scale_.shrink_to_fit();
    return meshes_for_scale_;
    cloud_1_adjusted->clear();
    cloud2_adjusted->clear();
}

std::vector<double> Reconstruction::getScale()
{
    return scale_;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> Reconstruction::manualScaleAdjuster(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &input_cloud, double scale)
{

    std::cout << currentDateTime() << "\nStarting Manual Scaling"
              << std::endl;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr result_mesh(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    scale_.push_back(scale);
    pcl::copyPointCloud(*input_cloud, *result_mesh);

    for (int i = 0; i < result_mesh->points.size(); i++)
    {
        result_mesh->points[i].x = result_mesh->points[i].x * scale_.front();
        result_mesh->points[i].y = result_mesh->points[i].y * scale_.front();
        result_mesh->points[i].z = result_mesh->points[i].z * scale_.front();
    }
    meshes_for_scale_.push_back(result_mesh); //
    meshes_for_scale_.shrink_to_fit();
    return meshes_for_scale_;
    std::cout << currentDateTime() << "\tManual Scaling finished\n"
              << std::endl;
}

void Reconstruction::convertPLYtoPCL(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &target_clopud, std::string ply_file)
{

    reader_ply.read(ply_file, *target_clopud);
    std::cout << currentDateTime() << "\t" << ply_file << "\tImported PLY and Converted to PCL\n"
              << std::endl;
}

void Reconstruction::convertPolyMeshtoPLY(pcl::PolygonMesh::Ptr &input, std::string target_path)
{
    pcl::io::savePolygonFilePLY(target_path, *input);
    std::cout << currentDateTime() << "\t" << target_path << "\tConverted PolyMesh to PLY\n"
              << std::endl;
}

void Reconstruction::downsamplePointCloudVoxelGrid(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &target_cloud, float lx, float ly, float lz)
{
    std::cerr << currentDateTime() << "\tStarting Downsampling\n"
              << currentDateTime() << "\tPointCloud before Downsampling: " << input_cloud->width * input_cloud->height
              << " data points (" << pcl::getFieldsList(*input_cloud) << ").\n"
              << std::endl;

    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());

    pcl::toPCLPointCloud2(*input_cloud, *cloud);

    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(lx, ly, lz);
    voxel.filter(*cloud_filtered);
    voxel.setDownsampleAllData(true);

    pcl::fromPCLPointCloud2(*cloud_filtered, *target_cloud);
    std::cerr << currentDateTime() << "\tPointCloud After Downsampling: " << target_cloud->width * target_cloud->height
              << " data points (" << pcl::getFieldsList(*target_cloud) << ").\n"
              << std::endl;
}

void Reconstruction::convertPCLtoPLY(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &input, std::string target_path)
{
    writer_ply.write(target_path, *input);
    std::cout << currentDateTime() << "\t" << target_path << "\tConverted PCL to PLY\n"
              << std::endl;
}

void Reconstruction::passThroughFilter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &reference_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &target_cloud, std::string field_name, float min_limit, float max_limit)
{
    pcl::PassThrough<pcl::PointXYZRGBNormal> Pass;
    Pass.setInputCloud(reference_cloud);
    Pass.setFilterFieldName(field_name);
    Pass.setFilterLimits(min_limit, max_limit);
    Pass.filter(*target_cloud);
}

void Reconstruction::pointCloudCentroidAlignmentAdjusment(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &reference_cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &target_cloud, double x_axis, double y_axis, double z_axis)
{
    pcl::PCA<pcl::PointXYZRGBNormal> cloud_pca;
    cloud_pca.setInputCloud(reference_cloud);
    Eigen::Matrix3f cloud_eigen_values_vector = cloud_pca.getEigenVectors();
    Eigen::Vector4f cloud_mid_point = cloud_pca.getMean();
    Eigen::Matrix4f cloud_transform = Eigen::Matrix4f::Identity();
    cloud_transform.block<3, 3>(0, 0) = cloud_eigen_values_vector;
    cloud_transform.block<4, 1>(0, 3) = cloud_mid_point;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tester(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    tester = reference_cloud;

    //Eigen::Affine3f transform(Eigen::Affine3f::Identity());
    Eigen::Affine3f transform;
    transform.matrix() = cloud_transform;

    // transform.rotate(Eigen::AngleAxisf((z_axis), Eigen::Vector3f::UnitZ()));
    // transform.rotate(Eigen::AngleAxisf((x_axis), Eigen::Vector3f::UnitX()));
    // transform.rotate(Eigen::AngleAxisf((y_axis), Eigen::Vector3f::UnitY()));
    // Eigen::Vector4f centroid(Eigen::Vector4f::Zero());
    // pcl::compute3DCentroid(*tester, centroid);
    // Eigen::Vector4f centroid_new(Eigen::Vector4f::Zero());
    // centroid_new.head<3>() = transform.rotation() * centroid.head<3>();
    //transform.translation() = centroid.head<3>() + centroid_new.head<3>();

    transform.linear() = (Eigen::Matrix3f)Eigen::AngleAxisf(x_axis, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(y_axis, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(z_axis, Eigen::Vector3f::UnitZ());

    pcl::transformPointCloud(*tester, *target_cloud, transform);
    tester->clear();
}

void Reconstruction::cloudToPolyMesh(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &ref_cloud,
                                     pcl::PolygonMesh::Ptr &mesh)
{

    pcl::toPCLPointCloud2(*ref_cloud, mesh->cloud);
}

void Reconstruction::importPLYtoPolymesh(std::string path, pcl::PolygonMesh::Ptr &target)
{
    reader_ply.read(path, *target);
}

void Reconstruction::normalsReAdjustement(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &reference_cloud)
{

    std::cout << "\n"
              << currentDateTime() << "\tADJUSTING NORMALS " << std::endl;

    std::vector<int> normalization;
    pcl::Normal normal_to_add;
    for (int i = 0; i < reference_cloud->points.size(); i++)
    {
        PointXYZRGBNormal point = reference_cloud->points[i];
        reference_cloud->at(i).normal_x = 0;
        reference_cloud->at(i).normal_y = 0;
        reference_cloud->at(i).normal_z = 0;
        normalization.push_back(0);
    }

    for (int i = 0; i < reference_cloud->size(); i++)
    {
        unsigned int n1 = mesh.polygons[i].vertices[0];
        unsigned int n2 = mesh.polygons[i].vertices[1];
        unsigned int n3 = mesh.polygons[i].vertices[2];

        // get the vertices coordinates of the face
        Eigen::Vector3f point_1 = reference_cloud->points[n1].getVector3fMap();
        Eigen::Vector3f point_2 = reference_cloud->points[n2].getVector3fMap();
        Eigen::Vector3f point_3 = reference_cloud->points[n3].getVector3fMap();

        // compute the normal using the "right-hand rule"
        Eigen::Vector3f vector_1 = point_2 - point_1;
        Eigen::Vector3f vector_2 = point_3 - point_1;
        Eigen::Vector3f face_normal = vector_1.cross(vector_2);

        face_normal = face_normal / face_normal.norm();

        // normalize the normal vector
        face_normal = face_normal / face_normal.norm();

        // stack up the new normals

        reference_cloud->points[n1].normal_x += face_normal[0];
        reference_cloud->points[n1].normal_y += face_normal[1];
        reference_cloud->points[n1].normal_z += face_normal[2];

        reference_cloud->points[n2].normal_x += face_normal[0];
        reference_cloud->points[n2].normal_y += face_normal[1];
        reference_cloud->points[n2].normal_z += face_normal[2];

        reference_cloud->points[n3].normal_x += face_normal[0];
        reference_cloud->points[n3].normal_y += face_normal[1];
        reference_cloud->points[n3].normal_z += face_normal[2];

        normalization[n1]++;
        normalization[n2]++;
        normalization[n3]++;
    }

    for (int i = 0; i < reference_cloud->size(); i++)
    {
        reference_cloud->points[i].normal_x /= normalization[i];
        reference_cloud->points[i].normal_y /= normalization[i];
        reference_cloud->points[i].normal_z /= normalization[i];
    }

    std::cout << currentDateTime() << "\tLoaded "
              << reference_cloud->width * reference_cloud->height
              << " data points from Mesh Normalized with the following fields: \t"
              << std::endl;

    normalization.clear();
}

void Reconstruction::normalsTesting(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &reference, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &target)
{
    pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> ne;

    ne.setInputCloud(reference);
    ne.setSearchSurface(reference);
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree4(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    ne.setSearchMethod(tree4);
    ne.setRadiusSearch(0.03);
    //ne.setKSearch(5);
    //ne.setViewPoint(0.0f,0.0f,0.0f);
    std::cout << currentDateTime() << "\tCreating Adjusted Normals "
              << " .............................................................\t"
              << std::endl;
    ne.compute(*cloud_normals);

    target = reference;
    std::cout << currentDateTime() << "\tAdjusting Normals of target PCL "
              << " .............................................................\t"
              << std::endl;

    for (int i = 0; i < target->points.size(); i++)
    {

        target->at(i).normal_x = cloud_normals->at(i).normal_x;
        target->at(i).normal_y = cloud_normals->at(i).normal_x;
        target->at(i).normal_z = cloud_normals->at(i).normal_x;
    }

    cloud_normals->clear();
}