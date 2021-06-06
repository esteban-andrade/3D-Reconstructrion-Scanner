/**
 * @file icp.h
 * @author Esteban Andrade
 * @brief SCALE ICP adjusted class
 * @version 0.1
 * @date 2021-06-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <fstream>
#include <cstdio>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/gp3.h>

//#include <cv2.h>
//#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"
#include <iostream>
using namespace cv;
using namespace std;
using namespace matlab::engine;


#include "rply.h"

/**
 * @brief Class to handle points 
 * 
 */
class aPoint
{
public:
    double x;
    double y;
    double z;
};

/**
 * @brief Class to handle 3D points
 * 
 */
class Points3D
{
public:
    aPoint *point3d;
    aPoint *pointcolor;
    aPoint *pointnormal;
    int point_num;
};

/**
 * @brief Class to handle Point vectors
 * 
 */
class Points3Dvec
{
public:
    vector<vector<double>> points;
    vector<vector<double>> colors;
    vector<vector<double>> normals;
};

/**
 * @brief Main ICP class
 * 
 */
class icp
{
private:
   std::string file_name_path_;
   int bin;
   double scale_;
public:
    /**
 * @brief Construct a new icp object
 * @param plyname name of plyfile
 * @param bin Image
 */
    icp(std::string, int bin);

    /**
     * @brief Construct a new icp object
     * 
     */
    icp();

    /**
     * @brief Destroy the icp object
     * 
     */
    ~icp();

    /**
     * @brief Start Process generation files
     * @param plyname name of plyfile
     * @param bin Image
     * @param bin 
     */
    void processGeneration(std::string, int bin);

    /**
     * @brief Obtain the ICP scale
     * @param files- vector of files names of processGeneration output
     * @return double (scale)
     */
    double getICPscale(std::vector<std::string> &);
};
