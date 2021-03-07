#include "opencv2/core/cvstd_wrapper.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <opencv2/objdetect.hpp>


#include <igl/opengl/glfw/Viewer.h>
#include <igl/triangle/triangulate.h>
#include <igl/harmonic.h>


#define NOMINMAX
#include <igl/boundary_conditions.h>
#include <igl/copyleft/tetgen/tetrahedralize.h>
#include <igl/copyleft/tetgen/mesh_with_skeleton.h>
#include <igl/colon.h>
#include <igl/directed_edge_orientations.h>
#include <igl/directed_edge_parents.h>
#include <igl/forward_kinematics.h>
#include <igl/PI.h>
#include <igl/partition.h>
#include <igl/mat_max.h>
#include <igl/lbs_matrix.h>
#include <igl/slice.h>
#include <igl/deform_skeleton.h>
#include <igl/dqs.h>
#include <igl/lbs_matrix.h>
#include <igl/columnize.h>
#include <igl/readDMAT.h>
#include <igl/readOBJ.h>
#include <igl/arap.h>
#include <igl/arap_dof.h>
#include <igl/bbw.h>
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>



typedef
std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
RotationList;

const Eigen::RowVector3d sea_green(70. / 255., 252. / 255., 167. / 255.);
Eigen::MatrixXd V, U, M;
//Eigen::MatrixXi F;
//Eigen::VectorXi S, b;
Eigen::VectorXi b;
Eigen::MatrixXd L;
//Eigen::RowVector3d mid;
double anim_t = 0.0;
double anim_t_dir = 0.03;
double bbd = 1.0;
bool resolve = true;
//igl::ARAPData arap_data, arap_grouped_data;
igl::ArapDOFData<Eigen::MatrixXd, double> arap_dof_data;
Eigen::SparseMatrix<double> Aeq;

enum ModeType
{
    MODE_TYPE_ARAP = 0,
    MODE_TYPE_ARAP_GROUPED = 1,
    MODE_TYPE_ARAP_DOF = 2,
    NUM_MODE_TYPES = 4
};

ModeType mode = MODE_TYPE_ARAP;

bool pre_draw(igl::opengl::glfw::Viewer & viewer)
{
    using namespace Eigen;
    using namespace std;
    if (resolve)
    {
        MatrixXd bc(b.size(), V.cols());
        //VectorXd Beq(3 * b.size());
        VectorXd Beq(2 * b.size());
        for (int i = 0; i < b.size(); i++)
        {
            bc.row(i) = V.row(b(i));
            switch (i % 4)
            {
            case 2:
                bc(i, 0) += 0.15*bbd*sin(1.5*anim_t);
                bc(i, 1) += 0.15*bbd*(1. - cos(0.5*anim_t));
                break;
            case 1:
                bc(i, 1) += 0.10*bbd*sin(1.5*anim_t*(i + 1));
                //bc(i, 2) += 0.10*bbd*(1. - cos(1.*anim_t*(i + 1)));
                break;
            case 0:
                bc(i, 0) += 0.20*bbd*sin(4.*anim_t*(i + 1));
                break;
            }
            /*Beq(3 * i + 0) = bc(i, 0);
            Beq(3 * i + 1) = bc(i, 1);
            Beq(3 * i + 2) = bc(i, 2);*/
            Beq(2 * i + 0) = bc(i, 0);
            Beq(2 * i + 1) = bc(i, 1);
        }

        VectorXd L0 = L;
        arap_dof_update(arap_dof_data, Beq, L0, 30, 0, L);
        const auto & Ucol = M * L;
        U.col(0) = Ucol.block(0 * U.rows(), 0, U.rows(), 1);
        U.col(1) = Ucol.block(1 * U.rows(), 0, U.rows(), 1);
        //U.col(2) = Ucol.block(2 * U.rows(), 0, U.rows(), 1);


        viewer.data().set_vertices(U);
        viewer.data().set_points(bc, sea_green);
        viewer.data().compute_normals();
        if (viewer.core.is_animating)
        {
            anim_t += anim_t_dir;
        }
        else
        {
            resolve = false;
        }


    }
    return false;
}

bool key_down(igl::opengl::glfw::Viewer &viewer, unsigned char key, int mods)
{
    switch (key)
    {
    case '0':
        anim_t = 0;
        resolve = true;
        return true;
    case '.':
        mode = (ModeType)(((int)mode + 1) % ((int)NUM_MODE_TYPES - 1));
        resolve = true;
        return true;
    case ',':
        mode = (ModeType)(((int)mode - 1) % ((int)NUM_MODE_TYPES - 1));
        resolve = true;
        return true;
    case ' ':
        viewer.core.is_animating = !viewer.core.is_animating;
        if (viewer.core.is_animating)
        {
            resolve = true;
        }
        return true;
    }
    return false;
}


using namespace Eigen;
using namespace std;

int getMaxAreaContourId(vector <vector<cv::Point>> contours) {
    double maxArea = 0;
    int maxAreaContourId = -1;
    for (int j = 0; j < contours.size(); j++) {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = j;
        } // End if
    } // End for
    return maxAreaContourId;
} // End function



vector<cv::Point_<int>> extract_contour_from_image(char* path)
{
    using namespace cv;
    Mat raw = imread(path);
    Mat src;
    flip(raw, src, 0);
    int r = 3 * 16 + 2, g = 4 * 16 + 10, b = 8 * 16 + 7;
    Mat binary_image = abs(src - Scalar(b, g, r));
    Mat src_gray;
    cvtColor(binary_image, src_gray, COLOR_BGR2GRAY);
    Mat canny_output;
    threshold(src_gray, canny_output, 1, 255, CV_THRESH_BINARY);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
    //Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    Scalar color = Scalar(166, 12, 89);
    int max_area_contour_index = getMaxAreaContourId(contours);
    drawContours(drawing, contours, max_area_contour_index, color, 2, LINE_8, hierarchy, 0);

#ifdef VERBOSE
     imshow("raw", raw);
     imshow("src", src);
     waitKey(0);
#endif

    return contours[max_area_contour_index];
}

void triangulate_contour(vector<cv::Point_<int>> input_contour, Eigen::MatrixXd input_control_points,  Eigen::MatrixXd& output_vertices, Eigen::MatrixXi& output_edges)
{
    int input_contour_size = input_contour.size();
    // Input polygon
    Eigen::MatrixXd input_vertices;
    Eigen::MatrixXi input_edges;
    Eigen::MatrixXd input_holes;

    // Create the boundary of a square
    
    input_vertices.resize(input_contour_size + 21, 2);
    input_edges.resize(input_contour_size, 2);

    for (size_t i = 0; i < input_contour_size; i++)
    {
        input_vertices.row(i) << input_contour[i].x, input_contour[i].y;// , 0;
        input_edges.row(i) << i, i + 1;
    }
    input_edges.row(input_contour_size - 1) << input_contour_size - 1, 0;

    for (size_t i = input_contour_size; i < input_contour_size + 21; i++)
    {
        input_vertices.row(i) << input_control_points.row(i - input_contour_size);
        //circle(drawing, Point(V2.row(i)[0], V2.row(i)[1]), 3, Scalar(0, 255, 25), -1);
    }
    /*imshow("Contours", drawing);
    waitKey(0);*/
  
    // Triangulate the interior
    // a0.005 means that the area of each triangle should
    // not be greater than 0.005
    // q means that no angles will be smaller than 20 degrees
    // for a detailed set of commands please refer to:
    // https://www.cs.cmu.edu/~quake/triangle.switch.html
    igl::triangle::triangulate(input_vertices, input_edges, input_holes, "a100q", output_vertices, output_edges);


    // Plot the mesh with pseudocolors
  /* igl::opengl::glfw::Viewer viewer;
   viewer.data().set_mesh(V, F);
   viewer.data().add_points(control_points, sea_green);
   viewer.launch();*/
}

void calculate_2d_mesh_bbw(Eigen::MatrixXd boundry_vertices_weights, Eigen::MatrixXi input_bone_edges, Eigen::MatrixXd input_bone_points, Eigen::MatrixXd input_vertices, Eigen::MatrixXi input_edges, VectorXi& boundry_indecies , Eigen::MatrixXd& output_bbw)
{
    bool res = igl::boundary_conditions(input_vertices, input_edges, input_bone_points, VectorXi(), input_bone_edges, MatrixXi(), boundry_indecies, boundry_vertices_weights);
    cout << res << endl << boundry_vertices_weights << endl;
    // compute BBW weights matrix
    igl::BBWData bbw_data;
    // only a few iterations for sake of demo
    bbw_data.active_set_params.max_iter = 50;
    bbw_data.verbosity = 2;
    res = igl::bbw(input_vertices, input_edges, boundry_indecies, boundry_vertices_weights, bbw_data, output_bbw);
    cout << res << endl << output_bbw.size() << endl;
}

void skeleton_inputs(Eigen::MatrixXi& BE, Eigen::MatrixXd& control_points)
{
    control_points.resize(21, 2);
    BE.resize(20, 2);

    control_points <<
        528, 182, //0, //palm 0
        571, 205, //0, //thumb 1
        622, 311, //0, //thumb 2
        653, 357, //0, //thumb 3
        679, 414 - 8, //0, //thumb 4
        575, 362, //0, //index 5
        587, 441, //0, //index 6 
        594, 488, //0, //index 7 
        600, 529 - 5, //0, //index 8 
        531, 361, //0, //middle 9 
        530, 445, //0, //middle 10 
        531, 496, //0, //middle 11
        532, 541 - 9, //0, //middle 12 
        489, 358,// 0, //ring 13
        474, 430,// 0, //ring 14
        466, 475,// 0, //ring 15
        458, 522 - 9,// 0, //ring 16
        450, 349,// 0, //pinky 17 
        423, 404,// 0,//pinky 18
        410, 436,// 0, //pinky 19
        396, 474 - 9,// 0; //pinky 20

        BE << 0, 1, 1, 2, 2, 3, 3, 4,
        0, 5, 5, 6, 6, 7, 7, 8,
        0, 9, 9, 10, 10, 11, 11, 12,
        0, 13, 13, 14, 14, 15, 15, 16,
        0, 17, 17, 18, 18, 19, 19, 20; //skeleton connectivity
}

int main(int argc, char *argv[])
{
    vector<cv::Point_<int>> contour = extract_contour_from_image("C:\\Users\\ofir\\Desktop\\leap_hand_example_full_res.PNG");

    Eigen::MatrixXi BE;
    Eigen::VectorXi bi;
    Eigen::MatrixXd control_points;
    Eigen::MatrixXd V;
    Eigen::MatrixXd U;
    Eigen::MatrixXi F;

    skeleton_inputs(BE, control_points);

    triangulate_contour(contour, control_points, V, F);

    Eigen::MatrixXd W;
    calculate_2d_mesh_bbw(control_points, BE, control_points, V, F,b,W);

    /* igl::readOBJ("C:\\Users\\ofir\\Desktop\\libigl\\tutorial\\data\\armadillo.obj", V, F);
    U = V;
    igl::readDMAT("C:\\Users\\ofir\\Desktop\\libigl\\tutorial\\data\\armadillo-weights.dmat", W);
    ofstream myfile;
    myfile.open("dmat.txt");
    myfile << W << endl;*/

    U = V; // probably for updates..
    igl::lbs_matrix_column(V, W, M);

    // Cluster according to weights
    VectorXi G;
    {
        VectorXi S;
        VectorXd D;
        igl::partition(W, 21, G, S, D);
        //igl::partition(W, 50, G, S, D);
    }

    //// vertices corresponding to handles (those with maximum weight)
    //{
    //    VectorXd maxW;
    //    igl::mat_max(W, 1, maxW, b);
    //}

    // Precomputation for FAST
    std::cout << "Initializing Fast Automatic Skinning Transformations..." << endl;
    // number of weights
    const int m = W.cols();
    //Aeq.resize(m * 3, m * 3 * (3 + 1));
    Aeq.resize(m * 2, m * 2 * (2 + 1));
    vector<Triplet<double> > ijv;
    for (int i = 0; i < m; i++)
    {
        RowVector3d homo;
        //RowVector4d homo;
        homo << V.row(b(i)), 1.;
        //for (int d = 0; d < 3; d++)
        for (int d = 0; d < 2; d++)
        {
            //for (int c = 0; c < (3 + 1); c++)
            for (int c = 0; c < (2 + 1); c++)
            {
                ijv.push_back(Triplet<double>(3 * i + d, i + c * m * 3 + d * m, homo(c)));
            }
        }
    }
    Aeq.setFromTriplets(ijv.begin(), ijv.end());
    igl::arap_dof_precomputation(V, F, M, G, arap_dof_data);
    igl::arap_dof_recomputation(VectorXi(), Aeq, arap_dof_data);
    // Initialize
    //MatrixXd Istack = MatrixXd::Identity(3, 3 + 1).replicate(1, m);
    MatrixXd Istack = MatrixXd::Identity(2, 2 + 1).replicate(1, m);
    igl::columnize(Istack, m, 2, L);


    // bounding box diagonal
    bbd = (V.colwise().maxCoeff() - V.colwise().minCoeff()).norm();

    // Plot the mesh with pseudocolors
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(U, F);
    //viewer.data().add_points(igl::slice(V, b, 1), sea_green);
    //viewer.data().show_lines = false;
    viewer.callback_pre_draw = &pre_draw;
    viewer.callback_key_down = &key_down;
    viewer.core.is_animating = false;
    viewer.core.animation_max_fps = 30.;
    std::cout <<
        "Press [space] to toggle animation." << endl <<
        "Press '0' to reset pose." << endl <<
        "Press '.' to switch to next deformation method." << endl <<
        "Press ',' to switch to previous deformation method." << endl;
    viewer.launch();
}


//#pragma region deformation tries
//
//  ////read skeleton from unity
//  //fileHandle = CreateFileW(TEXT(L"\\\\.\\pipe\\my-very-cool-pipe-example"), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);
//
//  //ULONG read = 0;
//  //// read from pipe server
//  //char* buffer = new char[2000];
//  //int counter = 0;
//  //while (counter++ < 100)
//  //{
//  //    memset(buffer, 0, 2000);
//  //    ReadString(buffer);
//  //    std::cout << "#######" << counter << "#######" << buffer << "\r\n";
//  //    Sleep(30);
//  //}
//  //
//  //return 0;
//
//  using namespace Eigen;
//  using namespace std;
//  using namespace cv;
//
//  //Eigen::MatrixXd C3,V3;
//  //Eigen::MatrixXi BE3,T3,F3;
//  //igl::readTGF("C:\\Users\\ofir\\Desktop\\libigl\\tutorial\\data\\hand.tgf", C3, BE3);
//  //cout << C3 << endl;
//  //cout << BE3 << endl;
//
//
//
//  //igl::readMESH("C:\\Users\\ofir\\Desktop\\libigl\\tutorial\\data\\hand.mesh", V3, T3, F3);
//  //
//
//
//  //// List of boundary indices (aka fixed value indices into VV)
//  //VectorXi b3;
//  //// List of boundary conditions of each weight function
//  //MatrixXd bc3;
//  //bool res3 = igl::boundary_conditions(V3, T3, C3, VectorXi(), BE3, MatrixXi(), b3, bc3);
//  //cout << res3 << endl << bc3 << endl;
//  //int a = 9090;
//  //return 0;
//
//
//
//  // Create the boundary of a square
//  control_points.resize(21, 2);
//  BE.resize(20, 2);
//  control_points_index.resize(21, 1);
//  V.resize(contour_size , 2);
//  E.resize(contour_size , 2);
//
//  control_points <<
//      528, 182, //0, //palm 0
//      571, 205, //0, //thumb 1
//      622, 311, //0, //thumb 2
//      653, 357, //0, //thumb 3
//      679, 414, //0, //thumb 4
//      575, 362, //0, //index 5
//      587, 441, //0, //index 6 
//      594, 488, //0, //index 7 
//      600, 529, //0, //index 8 
//      531, 361, //0, //middle 9 
//      530, 445, //0, //middle 10 
//      531, 496, //0, //middle 11
//      532, 541, //0, //middle 12 
//      489, 358,// 0, //ring 13
//      474, 430,// 0, //ring 14
//      466, 475,// 0, //ring 15
//      458, 522,// 0, //ring 16
//      450, 349,// 0, //pinky 17 
//      423, 404,// 0,//pinky 18
//      410, 436,// 0, //pinky 19
//      396, 474,// 0; //pinky 20
//
//  BE << 0, 1, 1, 2, 2, 3, 3, 4,
//      0, 5, 5, 6, 6, 7, 7, 8,
//      0, 9, 9, 10, 10, 11, 11, 12,
//      0, 13, 13, 14, 14, 15, 15, 16,
//      0, 17, 17, 18, 18, 19, 19, 20; //skeleton connectivity
//
//  for (size_t i = 0; i < contour_size; i++)
//  {
//      V.row(i) << contour[i].x, contour[i].y;// , 0;
//      E.row(i) << i, i + 1;
//  }
//  E.row(contour_size - 1) << contour_size - 1, 0;
//
///*  for (size_t i = contour_size; i < contour_size + 21; i++)
//  {
//      V.row(i) << control_points.row(i - contour_size);
//      E.row(i) << i, i;
//      control_points_index.row(i - contour_size) << i;
//  }*/
//
//  //V.row(contour.size()-1) << contour[contour.size() - 1].x, contour[contour.size() - 1].y;
//  /*control_points_moved << V.row(0) + RowVector3d(0, 50, 0);*/
//
//
//  // Triangulate the interior
//  // a0.005 means that the area of each triangle should
//  // not be greater than 0.005
//  // q means that no angles will be smaller than 20 degrees
//  // for a detailed set of commands please refer to:
//  // https://www.cs.cmu.edu/~quake/triangle.switch.html
//  
//
//  igl::triangle::triangulate(V, E, H, "a100q", V2, F2);
//
//
//
//  // List of boundary indices (aka fixed value indices into VV)
//  VectorXi bi;
//  // List of boundary conditions of each weight function
//  MatrixXd bc, W, zeros;
//  //to 3d
// /* zeros.setZero(V2.rows(), 1);
//  V2.conservativeResize(V2.rows(), V2.cols() + 1);
//  V2.col(V2.cols() - 1) = zeros;
//
//  zeros.setZero(control_points.rows(), 1);
//  control_points.conservativeResize(control_points.rows(), control_points.cols() + 1);
//  control_points.col(control_points.cols() - 1) = zeros;*/
//
//  //// Tetrahedralized interior
//  //Eigen::MatrixXd TV;
//  //Eigen::MatrixXi TT;
//  //Eigen::MatrixXi TF;
//  //// Tetrahedralize the interior
//  ////igl::copyleft::tetgen::tetrahedralize(V2, F2, "pq1.414Y", TV, TT, TF);
//  ////igl::harmonic(V2, F2, control_points, control_points_moved, 2., U);
//  // Mesh with samples on skeleton
//  // New vertices of tet mesh, V prefaces VV
//  //MatrixXd VV;
//  //// Tetrahedra
//  //MatrixXi TT;
//  //// New surface faces FF
//  //MatrixXi FF;
//
//  //if (!igl::copyleft::tetgen::mesh_with_skeleton(V2, F2, control_points, VectorXi(), BE, MatrixXi(), 10, VV, TT, FF))
//  //{
//  //    cout << "Tetgen Failed!" << endl;
//  //    return 1;
//  //}
//
//  //
//  //igl::boundary_conditions(V2, F2, control_points, VectorXi(), BE, MatrixXi(), bi, bc);
//  bool res = igl::boundary_conditions(V2, F2, control_points, VectorXi(), BE, MatrixXi(), bi, bc);
//  cout << res << endl << bc << endl;
//  // compute BBW weights matrix
//  igl::BBWData bbw_data;
//  igl::mosek::MosekData md;
//  // only a few iterations for sake of demo
//  bbw_data.active_set_params.max_iter = 8;
//  bbw_data.verbosity = 2;
//  if (!igl::mosek::bbw(V2, F2, bi, bc, bbw_data,md, W))
//  //if (!igl::bbw(TV, TT, bi, bc, bbw_data, W))
//  {
//      return EXIT_FAILURE;
//  }
//
//  // Plot the generated mesh
//  igl::opengl::glfw::Viewer viewer;
//  viewer.data().set_mesh(V2, F2);
//  viewer.launch();
//
//  return 0;  
//#pragma endregion
//

#define VERBOSE

int trian()
{
    using namespace cv;
    Mat raw = imread("C:\\Users\\ofir\\Desktop\\leap_hand_example_full_res.PNG");
    Mat src;
    flip(raw, src, 0);
    int r = 3 * 16 + 2, g = 4 * 16 + 10, b = 8 * 16 + 7;
    Mat binary_image = abs(src - Scalar(b, g, r));
    Mat src_gray;
    cvtColor(binary_image, src_gray, COLOR_BGR2GRAY);
    Mat canny_output;
    threshold(src_gray, canny_output, 1, 255, CV_THRESH_BINARY);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
    //Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
    Scalar color = Scalar(166, 12, 89);
    int max_area_contour_index = getMaxAreaContourId(contours);
    drawContours(drawing, contours, max_area_contour_index, color, 2, LINE_8, hierarchy, 0);


    auto contour = contours[max_area_contour_index];
    int contour_size = contour.size();
    /* imshow("raw", raw);
     waitKey(0);
     imshow("src", src);
     waitKey(0);*/



     // Input polygon
    Eigen::MatrixXd V, bc, W;
    Eigen::MatrixXi E, BE;
    VectorXi bi;
    Eigen::MatrixXd H;
    Eigen::MatrixXd control_points;
    Eigen::MatrixXi control_points_index;

    // Triangulated interior
    Eigen::MatrixXd V2, U;
    Eigen::MatrixXi F2;

    // Create the boundary of a square
    control_points.resize(21, 2);
    BE.resize(20, 2);
    control_points_index.resize(21, 1);
    V.resize(contour_size + 21, 2);
    E.resize(contour_size, 2);

    control_points <<
        528, 182, //0, //palm 0
        571, 205, //0, //thumb 1
        622, 311, //0, //thumb 2
        653, 357, //0, //thumb 3
        679, 414 - 8, //0, //thumb 4
        575, 362, //0, //index 5
        587, 441, //0, //index 6 
        594, 488, //0, //index 7 
        600, 529 - 5, //0, //index 8 
        531, 361, //0, //middle 9 
        530, 445, //0, //middle 10 
        531, 496, //0, //middle 11
        532, 541 - 9, //0, //middle 12 
        489, 358,// 0, //ring 13
        474, 430,// 0, //ring 14
        466, 475,// 0, //ring 15
        458, 522 - 9,// 0, //ring 16
        450, 349,// 0, //pinky 17 
        423, 404,// 0,//pinky 18
        410, 436,// 0, //pinky 19
        396, 474 - 9,// 0; //pinky 20

        BE << 0, 1, 1, 2, 2, 3, 3, 4,
        0, 5, 5, 6, 6, 7, 7, 8,
        0, 9, 9, 10, 10, 11, 11, 12,
        0, 13, 13, 14, 14, 15, 15, 16,
        0, 17, 17, 18, 18, 19, 19, 20; //skeleton connectivity

    for (size_t i = 0; i < contour_size; i++)
    {
        V.row(i) << contour[i].x, contour[i].y;// , 0;
        E.row(i) << i, i + 1;
    }
    E.row(contour_size - 1) << contour_size - 1, 0;

    for (size_t i = contour_size; i < contour_size + 21; i++)
    {
        V.row(i) << control_points.row(i - contour_size);
        //E.row(i) << i, i;
        control_points_index.row(i - contour_size) << i;

        circle(drawing, Point(V.row(i)[0], V.row(i)[1]), 3, Scalar(0, 255, 25), -1);
    }
    imshow("Contours", drawing);
    waitKey(0);
    //V.row(contour.size()-1) << contour[contour.size() - 1].x, contour[contour.size() - 1].y;
    /*control_points_moved << V.row(0) + RowVector3d(0, 50, 0);*/


    // Triangulate the interior
    // a0.005 means that the area of each triangle should
    // not be greater than 0.005
    // q means that no angles will be smaller than 20 degrees
    // for a detailed set of commands please refer to:
    // https://www.cs.cmu.edu/~quake/triangle.switch.html


    igl::triangle::triangulate(V, E, H, "a100q", V2, F2);

    // Plot the mesh with pseudocolors
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V2, F2);
    viewer.data().add_points(control_points, sea_green);
    viewer.launch();
    //return 0;
    bool res = igl::boundary_conditions(V2, F2, control_points, VectorXi(), BE, MatrixXi(), bi, bc);
    cout << res << endl << bc << endl;
    // compute BBW weights matrix
    igl::BBWData bbw_data;
    // only a few iterations for sake of demo
    bbw_data.active_set_params.max_iter = 50;
    bbw_data.verbosity = 2;
    if (!igl::bbw(V2, F2, bi, bc, bbw_data, W))
        //if (!igl::bbw(TV, TT, bi, bc, bbw_data, W))
    {
        return EXIT_FAILURE;
    }

    return 0;
}