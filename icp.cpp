#include "icp.h"

string cdw;
string eignvector;
vector<double> widthes;
double dot(aPoint a, aPoint b);
void minuss(aPoint a, aPoint b, aPoint &ans);
static int vertex_cb(p_ply_argument argument);
int readPointnum(aPoint *points, aPoint *normal, const char *input_ply);

int readPLY(aPoint *points, aPoint *normal, aPoint *color, const char *input_ply);

void writeTofile(float w, Mat data);
Mat spinImage(aPoint p, aPoint n,
              int selectnum, int pointnum, aPoint *points,
              double w, int bin);

int begin(int bin, string plyname);
float meshresolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
double *MinMaxDist(const Mat &points);
Points3Dvec read3Dpoints(const std::string &plyFilePath);

icp::icp(std::string plyname, int bin)
{
    if (widthes.size() > 0)
        widthes.clear();

    cdw = plyname + ".cdw";
    eignvector = plyname + ".eig";

    if (remove(cdw.c_str()) == -1)
        cerr << "Could not delete " << cdw << endl;
    else
        cout << "Deleted successed." << endl;
    if (remove(eignvector.c_str()) == -1)
        cerr << "Could not delete " << eignvector << endl;
    else
        cout << "Deleted successed." << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    int selectnum;
    int pointnum = 0;
    Points3Dvec pointvec;
    Mat points;
    if (plyname.find(".ply") == plyname.length() - 4)
    {
        Points3Dvec pointvec = read3Dpoints(plyname);
        pointnum = pointvec.points.size();
        points = Mat(3, pointnum, CV_64F);

        Cloud->resize(pointnum);
        normals->resize(pointnum);

        for (int i = 0; i < pointvec.points.size(); i++)
        {
            points.at<double>(0, i) = pointvec.points.at(i).at(0);
            points.at<double>(1, i) = pointvec.points.at(i).at(1);
            points.at<double>(2, i) = pointvec.points.at(i).at(2);

            Cloud->points.at(i).x = pointvec.points.at(i).at(0);
            Cloud->points.at(i).y = pointvec.points.at(i).at(1);
            Cloud->points.at(i).z = pointvec.points.at(i).at(2);

            normals->points.at(i).normal_x = pointvec.normals.at(i).at(0);
            normals->points.at(i).normal_y = pointvec.normals.at(i).at(1);
            normals->points.at(i).normal_z = pointvec.normals.at(i).at(2);
        }
        cout << "finish reading file " << plyname << "." << endl;
    }
    else
    {
        cout << "the file format should be ply!!" << endl;
        exit(1);
    }

    //     cout << "Calculating minimun and maxmum number of point cloud points distances." << endl;
    //     double * minmax = MinMaxDist(points);
    //     cout << minmax[0] << "\t" << minmax[1] << endl;

    cout << "bin: " << bin << endl;

    cout << "Calculating mesh resolution...." << endl;
    double meshreso = meshresolution(Cloud, normals);
    cout << "mesh resolution: " << meshreso << endl;

    int j;
    double scaledmesh = meshreso;
    int magtitu = 10;

    for (j = 0; j < 4; j++)
    {
        scaledmesh *= magtitu;
        if (scaledmesh >= 10)
            break;
    }
    double firstbit = int(scaledmesh);
    //cout << firstbit << endl;
    double a = 1.0;

    for (int i = 0; i <= j; i++)
    {
        firstbit /= 10;
        a /= 10;
    }
    //cout << firstbit << " " << a<< endl;

    firstbit *= 1;

    for (double w = firstbit; w < firstbit * 9.9; w += firstbit)
    {
        //cout<< w << "A ";
        widthes.push_back(double(w));
    }

    for (double w = firstbit * 10; w < firstbit * 99; w += firstbit * 10)
    {
        //cout<< w << "B ";
        widthes.push_back(double(w));
    }

    for (double w = firstbit * 100; w < firstbit * 999; w += firstbit * 100)
    {
        //cout<< w << "C ";
        widthes.push_back(double(w));
    }

    for (int i = 0; i < widthes.size(); i++)
    {
        cout << widthes.at(i) << "\t";
    }
    cout << endl;
    begin(bin, plyname);
};
icp::icp(){}
icp::~icp() {}

void icp::processGeneration(std::string plyname, int bin)
{
    if (widthes.size() > 0)
        widthes.clear();

    cdw = plyname + ".cdw";
    eignvector = plyname + ".eig";

    if (remove(cdw.c_str()) == -1)
        cerr << "Could not delete " << cdw << endl;
    else
        cout << "Deleted successed." << endl;
    if (remove(eignvector.c_str()) == -1)
        cerr << "Could not delete " << eignvector << endl;
    else
        cout << "Deleted successed." << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    int selectnum;
    int pointnum = 0;
    Points3Dvec pointvec;
    Mat points;
    if (plyname.find(".ply") == plyname.length() - 4)
    {
        Points3Dvec pointvec = read3Dpoints(plyname);
        pointnum = pointvec.points.size();
        points = Mat(3, pointnum, CV_64F);

        Cloud->resize(pointnum);
        normals->resize(pointnum);

        for (int i = 0; i < pointvec.points.size(); i++)
        {
            points.at<double>(0, i) = pointvec.points.at(i).at(0);
            points.at<double>(1, i) = pointvec.points.at(i).at(1);
            points.at<double>(2, i) = pointvec.points.at(i).at(2);

            Cloud->points.at(i).x = pointvec.points.at(i).at(0);
            Cloud->points.at(i).y = pointvec.points.at(i).at(1);
            Cloud->points.at(i).z = pointvec.points.at(i).at(2);

            normals->points.at(i).normal_x = pointvec.normals.at(i).at(0);
            normals->points.at(i).normal_y = pointvec.normals.at(i).at(1);
            normals->points.at(i).normal_z = pointvec.normals.at(i).at(2);
        }
        cout << "finish reading file " << plyname << "." << endl;
    }
    else
    {
        cout << "the file format should be ply!!" << endl;
        exit(1);
    }

    //     cout << "Calculating minimun and maxmum number of point cloud points distances." << endl;
    //     double * minmax = MinMaxDist(points);
    //     cout << minmax[0] << "\t" << minmax[1] << endl;

    cout << "bin: " << bin << endl;

    cout << "Calculating mesh resolution...." << endl;
    double meshreso = meshresolution(Cloud, normals);
    cout << "mesh resolution: " << meshreso << endl;

    int j;
    double scaledmesh = meshreso;
    int magtitu = 10;

    for (j = 0; j < 4; j++)
    {
        scaledmesh *= magtitu;
        if (scaledmesh >= 10)
            break;
    }
    double firstbit = int(scaledmesh);
    //cout << firstbit << endl;
    double a = 1.0;

    for (int i = 0; i <= j; i++)
    {
        firstbit /= 10;
        a /= 10;
    }
    //cout << firstbit << " " << a<< endl;

    firstbit *= 1;

    for (double w = firstbit; w < firstbit * 9.9; w += firstbit)
    {
        //cout<< w << "A ";
        widthes.push_back(double(w));
    }

    for (double w = firstbit * 10; w < firstbit * 99; w += firstbit * 10)
    {
        //cout<< w << "B ";
        widthes.push_back(double(w));
    }

    for (double w = firstbit * 100; w < firstbit * 999; w += firstbit * 100)
    {
        //cout<< w << "C ";
        widthes.push_back(double(w));
    }

    for (int i = 0; i < widthes.size(); i++)
    {
        cout << widthes.at(i) << "\t";
    }
    cout << endl;
    begin(bin, plyname);
};

double icp::getICPscale(std::vector<std::string> &files)
{
    // Start MATLAB engine synchronously
    std::unique_ptr<MATLABEngine> matlabPtr = startMATLAB();

    //Create MATLAB data array factory
    matlab::data::ArrayFactory factory;

    // Create a vector of input arguments
    std::vector<matlab::data::Array> args({

        factory.createCharArray(files.at(0)),
        factory.createCharArray(files.at(1)),

        factory.createScalar<int32_t>(100)});

    // Call MATLAB sqrt function on the data array
    matlab::data::TypedArray<double> result = matlabPtr->feval(u"ScaleRatioICP", args);
    double scale = result[0];
    scale_ = scale;
    std::cout << "SCALE: " << scale << std::endl;
    return scale_;
}

// const std::string currentDateTime()
// {
//     time_t now = time(0);
//     struct tm tstruct;
//     char buf[80];
//     tstruct = *localtime(&now);
//     // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
//     // for more information about date/time format
//     strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

//     return buf;
// }

//inner product of two 3D points
double dot(aPoint a, aPoint b)
{
    return (a.x * b.x + a.y * b.y + a.z * b.z);
}

//substraction of two 3D points
void minuss(aPoint a, aPoint b, aPoint &ans)
{
    ans.x = a.x - b.x;
    ans.y = a.y - b.y;
    ans.z = a.z - b.z;
}

static int vertex_cb(p_ply_argument argument)
{
    void *pdata;
    long indexCoord;

    ply_get_argument_user_data(argument, &pdata, &indexCoord);

    aPoint *points = *((aPoint **)pdata);

    long index;
    ply_get_argument_element(argument, NULL, &index);

    if (indexCoord == 1)
    {
        points[index].x = ply_get_argument_value(argument);
    }
    if (indexCoord == 2)
    {
        points[index].y = ply_get_argument_value(argument);
    }
    if (indexCoord == 3)
    {
        points[index].z = ply_get_argument_value(argument);
    }

    return 1;
}

int readPointnum(aPoint *points, aPoint *normal, const char *input_ply)
{
    int pointnum = 0;

    p_ply ply = ply_open(input_ply, NULL);
    if (!ply)
        return 1;
    if (!ply_read_header(ply))
        return 1;

    long nvertices =
        ply_set_read_cb(ply, "vertex", "x", vertex_cb, &points, 1);
    pointnum = int(nvertices);

    points = new aPoint[pointnum];
    normal = new aPoint[pointnum];

    ply_close(ply);

    return pointnum;
}

int readPLY(aPoint *points, aPoint *normal, aPoint *color, const char *input_ply)
{
    p_ply ply = ply_open(input_ply, NULL);
    if (!ply)
        return 1;
    if (!ply_read_header(ply))
        return 1;

    ply_set_read_cb(ply, "vertex", "x", vertex_cb, &points, 1);
    ply_set_read_cb(ply, "vertex", "y", vertex_cb, &points, 2);
    ply_set_read_cb(ply, "vertex", "z", vertex_cb, &points, 3);
    ply_set_read_cb(ply, "vertex", "nx", vertex_cb, &normal, 1);
    ply_set_read_cb(ply, "vertex", "ny", vertex_cb, &normal, 2);
    ply_set_read_cb(ply, "vertex", "nz", vertex_cb, &normal, 3);
    ply_set_read_cb(ply, "vertex", "diffuse_red", vertex_cb, &color, 1);
    ply_set_read_cb(ply, "vertex", "diffuse_green", vertex_cb, &color, 2);
    ply_set_read_cb(ply, "vertex", "diffuse_blue", vertex_cb, &color, 3);

    if (!points || !normal || !color)
    {
        cout << "Error: new aPoint.\n"
             << endl;
        if (!points)
            delete[] points;
        if (!normal)
            delete[] normal;
        if (!color)
            delete[] color;
        return 1;
    }

    if (!ply_read(ply))
        return 1; // read entire data at once
    ply_close(ply);

    return 0;
}

Points3Dvec read3Dpoints(const std::string &plyFilePath)
{
    aPoint *points = NULL;
    aPoint *normal = NULL;
    aPoint *color = NULL;
    int pointnum = 0;
    pointnum = readPointnum(points, normal, plyFilePath.c_str());
    color = new aPoint[pointnum];
    points = new aPoint[pointnum];
    normal = new aPoint[pointnum];
    readPLY(points, normal, color, plyFilePath.c_str());
    Points3Dvec points3dvec;

    for (int i = 0; i < pointnum; i++)
    {
        vector<double> tmp;
        tmp.push_back(points[i].x);
        tmp.push_back(points[i].y);
        tmp.push_back(points[i].z);
        points3dvec.points.push_back(tmp);

        vector<double> tmp1;
        tmp1.push_back(normal[i].x);
        tmp1.push_back(normal[i].y);
        tmp1.push_back(normal[i].z);
        points3dvec.normals.push_back(tmp1);

        vector<double> tmp2;
        tmp2.push_back(color[i].x);
        tmp2.push_back(color[i].y);
        tmp2.push_back(color[i].z);
        points3dvec.colors.push_back(tmp2);
    }
    delete[] points;
    delete[] normal;
    delete[] color;
    return points3dvec;
}

void writeTofile(float w, Mat data)
{
    ofstream outfile(eignvector.c_str(), ios::app);
    if (!outfile)
    {
        cout << "!outfile" << endl;
        exit(1);
    }
    outfile << w << "\t";

    for (int i = 0; i < data.rows; i++)
    {
        outfile << data.at<float>(i, 0) << "\t";
    }
    outfile << endl;
    outfile.close();

    float sumdata = 0.0f;

    for (int i = 0; i < data.rows; i++)
    {
        sumdata += data.at<float>(i, 0);
    }
    ofstream outfile1(cdw.c_str(), ios::app);
    if (!outfile1)
    {
        cout << "!outfile1" << endl;
        exit(1);
    }
    outfile1 << w << "\t";
    float sumeigen = 0;

    for (int i = 0; i < data.rows; i++)
    {
        sumeigen += data.at<float>(i, 0);
        outfile1 << sumeigen / sumdata << "\t";
    }
    outfile1 << endl;
    outfile1.close();
}

//calculate spin images
//return 1xN matrix
Mat spinImage(aPoint p, aPoint n,
              int selectnum, int pointnum, aPoint *points,
              double w, int bin)
{
    Mat spintmp = Mat::zeros(1, bin * bin, CV_32F);

    for (int i = 0; i < pointnum; i++)
    {
        if (i != selectnum)
        {
            double alph, beta;
            int place;

            alph = (points[i].x - p.x) * (points[i].x - p.x) + (points[i].y - p.y) * (points[i].y - p.y) + (points[i].z - p.z) * (points[i].z - p.z);
            aPoint c;
            minuss(points[i], p, c);
            beta = dot(n, c);
            if ((-w / 2) <= beta && (beta < w / 2))
            {
                alph -= beta * beta;
                alph = sqrt(alph);
                if ((0 <= alph) && (alph < w))
                {
                    beta = w / 2 - beta;
                    place = int(beta / (w / bin)) * bin + int(alph / (w / bin));
                    //spin[place] += 1;
                    spintmp.at<float>(0, place) += 1;
                }
            }
        }
    }
    return spintmp;
}

int begin(int bin, string plyname)
{
    int selectnum;
    int pointnum = 0;
    aPoint *points = NULL;
    aPoint *normal = NULL;
    if (plyname.find(".ply") == plyname.length() - 4)
    {
        Points3Dvec pointvec = read3Dpoints(plyname);
        pointnum = pointvec.points.size();
        points = new aPoint[pointnum];
        normal = new aPoint[pointnum];

        for (int i = 0; i < pointnum; i++)
        {
            points[i].x = pointvec.points.at(i).at(0);
            points[i].y = pointvec.points.at(i).at(1);
            points[i].z = pointvec.points.at(i).at(2);

            normal[i].x = pointvec.normals.at(i).at(0);
            normal[i].y = pointvec.normals.at(i).at(1);
            normal[i].z = pointvec.normals.at(i).at(2);
        }
        cout << "finish reading file " << plyname << "." << endl;
    }
    else
    {
        cout << "Your input is not PLY file, system will exist!" << endl;
        exit(1);
    }

    int limiter = pointnum;
    aPoint p, n;
    widthes.shrink_to_fit();
    vector<Mat> pcaset(widthes.size());
    for (int i = 0; i < widthes.size(); i++)
    {
        pcaset[i] = Mat::zeros(limiter, bin * bin, CV_32F);
    }
    //generate spin images for all 3D poings
    cerr << "begin with spin image generation....." << limiter - 1 << "<-limiter\t" << widthes.size() << "<-widthes\t" << bin * bin << "<-bin^2\t" << endl;

    for (selectnum = 0; selectnum < limiter; selectnum++)
    {
        p.x = points[selectnum].x;
        p.y = points[selectnum].y;
        p.z = points[selectnum].z;
        n.x = normal[selectnum].x;
        n.y = normal[selectnum].y;
        n.z = normal[selectnum].z;

        for (int i = 0; i < widthes.size(); i++)
        {
            //cout << "processing " << i << " " <<widthes.at(i) << endl;
            Mat spin; // = Mat::zeros(1, bin * bin, CV_32F);
            spin = spinImage(p, n, selectnum, pointnum, points, widthes.at(i), bin);

            for (int j = 0; j < bin * bin; j++)
            {
                pcaset[i].at<float>(selectnum, j) = spin.at<float>(0, j);
            }
            //cout << widthes.at(i) << "\t" <<spin<< endl;
            //cout << pcaset[i].row(selectnum) << endl;
            /*cout << "here++++++" << selectnum<< " " << limiter << endl;*/
        }
        // std::cout
        // 	<< currentDateTime() << "\tIter " << selectnum + 1 << "\tRemaining " <<(limiter-selectnum) << std::endl;
    }
    cerr << "end with spin image generation....." << endl;

    for (int i = 0; i < widthes.size(); i++)
    {

        PCA pca(pcaset[i], // pass the data
                Mat(),     // we do not have a pre-computed mean vector, so let the PCA engine to compute it
                //CV_PCA_DATA_AS_ROW, // indicate that the vectors are stored as matrix rows (use CV_PCA_DATA_AS_COL if the vectors are the matrix columns)
                0,
                pcaset[i].rows); // specify, how many principal components to retain
        //cout << pca.eigenvalues << endl;
        writeTofile(widthes.at(i), pca.eigenvalues);
    }
    cerr << "calculating over!" << endl;
    //cout << pcaset[widthes.size()-1] << endl;
    delete[] points;
    delete[] normal;
    return 1;
}

float meshresolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(100);

    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);
    float mesh_resolution = 0.0f;
    //cout << triangles.polygons.size() << endl;

    //saveCloud("out.vtk", triangles);

    for (int i = 0; i < triangles.polygons.size(); i++)
    {
        //     cout << "polygons " << i <<endl;
        //     cout << ":               " << cloud->points[triangles.polygons[i].vertices[0]].x
        //     << " " << cloud->points[triangles.polygons[i].vertices[0]].y
        //     << " " << cloud->points[triangles.polygons[i].vertices[0]].z << endl;
        //     cout << ":               " << cloud->points[triangles.polygons[i].vertices[1]].x
        //     << " " << cloud->points[triangles.polygons[i].vertices[1]].y
        //     << " " << cloud->points[triangles.polygons[i].vertices[1]].z << endl;
        //     cout << ":               " << cloud->points[triangles.polygons[i].vertices[2]].x
        //     << " " << cloud->points[triangles.polygons[i].vertices[2]].y
        //     << " " << cloud->points[triangles.polygons[i].vertices[2]].z << endl;
        Mat a = Mat(3, 1, CV_32F);
        a.at<float>(0, 0) = cloud->points[triangles.polygons[i].vertices[0]].x;
        a.at<float>(1, 0) = cloud->points[triangles.polygons[i].vertices[0]].y;
        a.at<float>(2, 0) = cloud->points[triangles.polygons[i].vertices[0]].z;
        Mat b = Mat(3, 1, CV_32F);
        b.at<float>(0, 0) = cloud->points[triangles.polygons[i].vertices[1]].x;
        b.at<float>(1, 0) = cloud->points[triangles.polygons[i].vertices[1]].y;
        b.at<float>(2, 0) = cloud->points[triangles.polygons[i].vertices[1]].z;
        Mat c = Mat(3, 1, CV_32F);
        c.at<float>(0, 0) = cloud->points[triangles.polygons[i].vertices[2]].x;
        c.at<float>(1, 0) = cloud->points[triangles.polygons[i].vertices[2]].y;
        c.at<float>(2, 0) = cloud->points[triangles.polygons[i].vertices[2]].z;
        Mat a_b = a - b;
        Mat b_c = b - c;
        Mat c_a = c - a;
        Mat ab = a_b.t() * a_b;
        Mat bc = b_c.t() * b_c;
        Mat ca = c_a.t() * c_a;
        //cout << ab << endl;
        mesh_resolution += std::sqrt(ab.at<float>(0, 0));
        mesh_resolution += std::sqrt(bc.at<float>(0, 0));
        mesh_resolution += std::sqrt(ca.at<float>(0, 0));
        //cout << "mesh resolution " << mesh_resolution << endl;
    }
    return mesh_resolution / (triangles.polygons.size() * 3);
}

double *MinMaxDist(const Mat &points)
{
    double *minmax = new double[2];
    double maxnum = 1e-10;
    double minnum = 10e+10;

    for (int i = 0; i < points.cols; i++)
    {

        for (int j = 0; j < points.cols; j++)
        {
            if (i != j)
            {
                Mat normvec = points.col(i) - points.col(j);
                double numtmp = sqrt(normvec.at<double>(0, 0) * normvec.at<double>(0, 0) + normvec.at<double>(1, 0) * normvec.at<double>(1, 0) + normvec.at<double>(2, 0) * normvec.at<double>(2, 0));
                //cout << numtmp << endl;
                if (numtmp > maxnum)
                {
                    maxnum = numtmp;
                }
                if (numtmp < minnum)
                {
                    minnum = numtmp;
                }
            }
        }
    }
    minmax[0] = minnum;
    minmax[1] = maxnum;
    return minmax;
}