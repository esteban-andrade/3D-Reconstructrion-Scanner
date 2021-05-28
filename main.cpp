#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "Reconstruction.h"
#include "icp.h"

int main(int argc, char *argv[])
{

    std::string original_mesh = argv[1];
    std::string scale_reference_file = argv[2];
    std::string save_files_string = argv[3];
    bool save_files;
    if (save_files_string == "yes")
    {
        save_files = true;
    }
    else
    {
        save_files = false;
    }

    std::cout << "STARTING " << argv[0] << std::endl;
    std::cout << "\nOriginal Mesh Imported Path\t" << argv[1] << std::endl;
    std::cout << "\nMesh used for Scaling Imported Path\t" << argv[2] << std::endl;
    std::cout << "\nSaving cloud of pipeline -->\t" << argv[3] << std::endl;

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_adjusted(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_segmented_plane(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_segmented_and_clusterized(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_segmented_and_clusterized_filtered(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_segmented_and_clusterized_filtered_new(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr smoothed_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointcloud_reference_scale(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> meshes_scale;
    std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> meshes_processing_scale_alignment;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PolygonMesh::Ptr polymesh(new pcl::PolygonMesh);
    pcl::PolygonMesh::Ptr polymesh_refined(new pcl::PolygonMesh);
    pcl::PolygonMesh::Ptr poisson_mesh(new pcl::PolygonMesh);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_aligned_new(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_aligned_new_transformed(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr smoothed_cloud_refined(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    Reconstruction ReconstructionMain(cloud, original_mesh);
    Reconstruction helper;
    Reconstruction Lidar;

    /*************LIDAR **************/

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filtered_ref_cloud1(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filtered_ref_cloud2(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filtered_outliers_ref_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr segmented_ref_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cluster_ref_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cluster_ref_cloud_aligned(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    /****************************/
#pragma omp parallel sections
    {

#pragma omp section
        {
            /*************LIDAR **************/
            Lidar.convertPLYtoPCL(pointcloud_reference_scale, scale_reference_file);                    //used for lidar
            Lidar.passThroughFilter(pointcloud_reference_scale, filtered_ref_cloud1, "x", -1.5f, 1.5f); //lidar removal
            Lidar.passThroughFilter(filtered_ref_cloud1, filtered_ref_cloud2, "z", -2.5f, 0.0f);        //lidar removal
            Lidar.pointCloudOutliersFilterManual(filtered_ref_cloud2, filtered_outliers_ref_cloud, 100, 0.9);
            Lidar.pointCloudPlaneSegmentOperationManual(filtered_outliers_ref_cloud, segmented_ref_cloud, 0.015, 100000); //0.01
            Lidar.pointCloudClustering(segmented_ref_cloud, cluster_ref_cloud);

            /****************************/
        }

#pragma omp section
        {
            ReconstructionMain.importPLYtoPolymesh(original_mesh, polymesh);
            ReconstructionMain.normalsDirectionAdjustment(cloud, cloud_adjusted, polymesh);

            ReconstructionMain.pointCloudOutliersFilter(cloud_adjusted, cloud_filtered);

            ReconstructionMain.pointCloudPlaneSegmentOperation(cloud_filtered, cloud_segmented_plane);
            ReconstructionMain.pointCloudClustering(cloud_segmented_plane, cloud_segmented_and_clusterized);
            meshes_processing_scale_alignment = ReconstructionMain.pointCloudScaleAdjustment(cloud_segmented_and_clusterized, cluster_ref_cloud, true, true);
            meshes_processing_scale_alignment.shrink_to_fit();
            ReconstructionMain.pointCloudCentroidAlignmentAdjusment(meshes_processing_scale_alignment.at(0), cloud_aligned, M_1_PI, 0, M_PI); //x,y,z radians
            ReconstructionMain.passThroughFilter(cloud_aligned, cloud_segmented_and_clusterized_filtered, "x", -0.78f, 2.0f);                 //0.81
            cloud_segmented_and_clusterized_filtered_new = cloud_segmented_and_clusterized_filtered;
            meshes_scale = helper.pointCloudScaleAdjustment(cloud_segmented_and_clusterized_filtered_new, cluster_ref_cloud, true, true);
            meshes_scale.shrink_to_fit();
            helper.pointCloudCentroidAlignmentAdjusment(meshes_scale.at(0), cloud_aligned_new, 0, 0, 0); //x,y,z radians
            Lidar.transformPointCloud(cluster_ref_cloud, meshes_scale.at(1), cluster_ref_cloud_aligned);
            helper.transformPointCloud(cluster_ref_cloud, cloud_aligned_new, cloud_aligned_new_transformed);
            ReconstructionMain.pointCloudNormaliseUpscale(cloud_aligned_new, smoothed_cloud);

            ReconstructionMain.transformPointCloud(cluster_ref_cloud, smoothed_cloud, smoothed_cloud_refined);
            ReconstructionMain.transformVerification(smoothed_cloud_refined, cluster_ref_cloud_aligned);
            ReconstructionMain.poissonReconstruction(smoothed_cloud_refined, poisson_mesh);
        }
    }
    if (save_files == true)
    {
        std::cout << "\n***************Saving clouds into \t-->Meshes Directory********************\n"
                  << std::endl;

        helper.convertPCLtoPLY(cloud_filtered, "../Meshes/cloud_with_outliers_removed.ply");
        helper.convertPCLtoPLY(cloud_segmented_plane, "../Meshes/cloud_with_plane_segmented.ply");
        helper.convertPCLtoPLY(cloud_segmented_and_clusterized, "../Meshes/cloud_clusterised.ply");
        helper.convertPCLtoPLY(cloud_segmented_and_clusterized_filtered_new, "../Meshes/cloud_segmented_and_clusterized_filtered_new.ply");
        helper.convertPCLtoPLY(cluster_ref_cloud, "../Meshes/processed_reference_cloud.ply");
        helper.convertPCLtoPLY(cluster_ref_cloud_aligned, "../Meshes/processed_reference_cloud_aligned.ply");
        helper.convertPCLtoPLY(cloud_aligned_new_transformed, "../Meshes/target_cloud_scaled_aligned.ply");
        helper.convertPolyMeshtoPLY(poisson_mesh, "../Meshes/pcl_poisson_mesh.ply");
        helper.convertPCLtoPLY(smoothed_cloud, "../Meshes/smoothed.ply");
        helper.convertPCLtoPLY(smoothed_cloud_refined, "../Meshes/smoothed_cloud_refined.ply");
        }
    else
    {
        std::cout << "\n***************Not Saving Files.\t Please Review Existing Ones********************\n"
                  << std::endl;
    }

    

    const std::string windowsName = "Point Cloud Viewer";
    std::cout << "\n***************Starting\t----->" << windowsName << "********************\n"
              << std::endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer>
        viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setSize(1280, 1024); // Visualiser window size
    viewer->initCameraParameters();
    int v1(0);
    int v2(0);
    int v3(0);
    int v4(0);
    int v5(0);
    int v6(0);
    int v7(0);
    int v8(0);
    //createViewPort (double xmin, double ymin, double xmax, double ymax, int &viewport);
    viewer->createViewPort(0, 0, 0.25, 0.5, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->addText("1 .RAW MESH", 10, 10, 1, 1, 1, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> mesh1(cloud_adjusted);
    viewer->addPointCloud<PointXYZRGBNormal>(cloud_adjusted, "Pointcloud 1", v1);
    viewer->addPointCloudNormals<PointXYZRGBNormal>(cloud_adjusted, 10, 0.01f, "PointcloudNormals 1", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Pointcloud 1");
    viewer->createViewPortCamera(v1);

    viewer->createViewPort(0.25, 0, 0.5, 0.5, v2);
    viewer->setBackgroundColor(0, 0, 0, v2);
    viewer->addText("2 .FILTERED MESH", 10, 10, 1, 1, 1, "v2 text", v2);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> mesh2(cloud_filtered);
    viewer->addPointCloud<PointXYZRGBNormal>(cloud_filtered, "Pointcloud 2", v2);
    viewer->addPointCloudNormals<PointXYZRGBNormal>(cloud_filtered, 10, 0.01f, "PointcloudNormals 2", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Pointcloud 2");
    viewer->createViewPortCamera(v2);

    viewer->createViewPort(0.5, 0, 0.75, 0.5, v3);
    viewer->setBackgroundColor(0, 0, 0, v3);
    viewer->addText("3 .SEGMENTED", 10, 10, 1, 1, 1, "v3 text", v3);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> mesh3(cloud_segmented_plane);
    viewer->addPointCloud<PointXYZRGBNormal>(cloud_segmented_plane, "Pointcloud 3", v3);
    viewer->addPointCloudNormals<PointXYZRGBNormal>(cloud_segmented_plane, 10, 0.01f, "PointcloudNormals 3", v3);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Pointcloud 3");
    viewer->createViewPortCamera(v3);

    viewer->createViewPort(0.75, 0, 1, 0.5, v4);
    viewer->setBackgroundColor(0, 0, 0, v4);
    viewer->addText("4 .CLUSTERIZED", 10, 10, 1, 1, 1, "v4 text", v4);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> mesh4(cloud_segmented_and_clusterized);
    viewer->addPointCloud<PointXYZRGBNormal>(cloud_segmented_and_clusterized, "Pointcloud 4", v4);
    viewer->addPointCloudNormals<PointXYZRGBNormal>(cloud_segmented_and_clusterized, 10, 0.01f, "PointcloudNormal 4", v4);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Pointcloud 4");
    viewer->createViewPortCamera(v4);

    viewer->createViewPort(0.67, 0.5, 1, 1, v8);
    viewer->setBackgroundColor(0, 0, 0, v8);
    viewer->addText("5. GREEN-> SCALED PCL", 10, 10, 1, 1, 1, "v8A text", v8);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> mesh8a(cloud_aligned_new_transformed, 0, 255, 0);
    viewer->addPointCloud<PointXYZRGBNormal>(cloud_aligned_new_transformed, mesh8a, "Pointcloud 8a", v8);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Pointcloud 8a");

    viewer->addText("5. RED-> REFERENCE PCL", 10, 1, 1, 1, 1, "v8b text", v8);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> mesh8b(cluster_ref_cloud_aligned, 255, 0, 0);
    viewer->addPointCloud<PointXYZRGBNormal>(cluster_ref_cloud_aligned, mesh8b, "Pointcloud 8b", v8);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Pointcloud 8b");

    viewer->createViewPortCamera(v8);

    viewer->createViewPort(0.33, 0.5, 0.67, 1, v5);
    viewer->setBackgroundColor(0, 0, 0, v5);
    viewer->addText("6 .SMOOTHED NORMALS & RANDOM UNIFORM DENSITY UPSAMPLED", 10, 10, 1, 1, 1, "v5 text", v5);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> mesh5(smoothed_cloud_refined);
    viewer->addPointCloud<PointXYZRGBNormal>(smoothed_cloud_refined, "Pointcloud 5", v5);
    viewer->addPointCloudNormals<PointXYZRGBNormal>(smoothed_cloud_refined, 10, 0.05f, "PointcloudNormals 5", v5);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Pointcloud 5");
    viewer->createViewPortCamera(v5);

    viewer->createViewPort(0, 0.5, 0.33, 1, v6);
    viewer->setBackgroundColor(0, 0, 0, v6);
    viewer->addText("7 . POISSON RECONSTRUCTED", 10, 10, 1, 1, 1, "v6 text", v6);
    viewer->addPolygonMesh(*poisson_mesh, "Pointcloud 6", v6);
    viewer->createViewPortCamera(v6);

    viewer->addCoordinateSystem(1); // blue = z axis  , Green  = y axis   Red = x axis

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return EXIT_SUCCESS;
}
