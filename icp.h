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

class aPoint
{
public:
    double x;
    double y;
    double z;
};

class Points3D
{
public:
    aPoint *point3d;
    aPoint *pointcolor;
    aPoint *pointnormal;
    int point_num;
};

class Points3Dvec
{
public:
    vector<vector<double>> points;
    vector<vector<double>> colors;
    vector<vector<double>> normals;
};

class icp
{
private:
   std::string file_name_path_;
   int bin;
   double scale_;
public:
    icp(std::string, int bin);
    icp();
    ~icp();
    void processGeneration(std::string, int bin);
     double getICPscale(std::vector<std::string> &);
};
