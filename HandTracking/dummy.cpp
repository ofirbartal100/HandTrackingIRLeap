//#include <igl/opengl/glfw/Viewer.h>
//#include <igl/triangle/triangulate.h>
//#include <igl/harmonic.h>

//#define NOMINMAX
//#include <igl/boundary_conditions.h>
//#include <igl/copyleft/tetgen/tetrahedralize.h>
//#include <igl/copyleft/tetgen/mesh_with_skeleton.h>
//#include <igl/colon.h>
//#include <igl/directed_edge_orientations.h>
//#include <igl/directed_edge_parents.h>
//#include <igl/forward_kinematics.h>
//#include <igl/PI.h>
//#include <igl/partition.h>
//#include <igl/mat_max.h>
//#include <igl/lbs_matrix.h>
//#include <igl/slice.h>
//#include <igl/deform_skeleton.h>
//#include <igl/dqs.h>
//#include <igl/lbs_matrix.h>
//#include <igl/columnize.h>
//#include <igl/readDMAT.h>
//#include <igl/readOBJ.h>
//#include <igl/arap.h>
//#include <igl/arap_dof.h>
//#include <igl/opengl/glfw/Viewer.h>
//#include <Eigen/Geometry>
//#include <Eigen/StdVector>

//
//typedef
//std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
//RotationList;
//
//const Eigen::RowVector3d sea_green(70. / 255., 252. / 255., 167. / 255.);
//Eigen::MatrixXd V, U, M;
//Eigen::MatrixXi F;
//Eigen::VectorXi S, b;
//Eigen::MatrixXd L;
//Eigen::RowVector3d mid;
//double anim_t = 0.0;
//double anim_t_dir = 0.03;
//double bbd = 1.0;
//bool resolve = true;
//igl::ARAPData arap_data, arap_grouped_data;
//igl::ArapDOFData<Eigen::MatrixXd, double> arap_dof_data;
//Eigen::SparseMatrix<double> Aeq;
//
//enum ModeType
//{
//    MODE_TYPE_ARAP = 0,
//    MODE_TYPE_ARAP_GROUPED = 1,
//    MODE_TYPE_ARAP_DOF = 2,
//    NUM_MODE_TYPES = 4
//};
//
//ModeType mode = MODE_TYPE_ARAP;
//
//bool pre_draw(igl::opengl::glfw::Viewer & viewer)
//{
//    using namespace Eigen;
//    using namespace std;
//    if (resolve)
//    {
//        MatrixXd bc(b.size(), V.cols());
//        VectorXd Beq(3 * b.size());
//        for (int i = 0; i < b.size(); i++)
//        {
//            bc.row(i) = V.row(b(i));
//            switch (i % 4)
//            {
//            case 2:
//                bc(i, 0) += 0.15*bbd*sin(1.5*anim_t);
//                bc(i, 1) += 0.15*bbd*(1. - cos(0.5*anim_t));
//                break;
//            case 1:
//                bc(i, 1) += 0.10*bbd*sin(1.5*anim_t*(i + 1));
//                bc(i, 2) += 0.10*bbd*(1. - cos(1.*anim_t*(i + 1)));
//                break;
//            case 0:
//                bc(i, 0) += 0.20*bbd*sin(4.*anim_t*(i + 1));
//                break;
//            }
//            Beq(3 * i + 0) = bc(i, 0);
//            Beq(3 * i + 1) = bc(i, 1);
//            Beq(3 * i + 2) = bc(i, 2);
//        }
//
//        VectorXd L0 = L;
//        arap_dof_update(arap_dof_data, Beq, L0, 30, 0, L);
//        const auto & Ucol = M * L;
//        U.col(0) = Ucol.block(0 * U.rows(), 0, U.rows(), 1);
//        U.col(1) = Ucol.block(1 * U.rows(), 0, U.rows(), 1);
//        U.col(2) = Ucol.block(2 * U.rows(), 0, U.rows(), 1);
//
//
//        viewer.data().set_vertices(U);
//        viewer.data().set_points(bc, sea_green);
//        viewer.data().compute_normals();
//        if (viewer.core.is_animating)
//        {
//            anim_t += anim_t_dir;
//        }
//        else
//        {
//            resolve = false;
//        }
//
//
//    }
//    return false;
//}
//
//bool key_down(igl::opengl::glfw::Viewer &viewer, unsigned char key, int mods)
//{
//    switch (key)
//    {
//    case '0':
//        anim_t = 0;
//        resolve = true;
//        return true;
//    case '.':
//        mode = (ModeType)(((int)mode + 1) % ((int)NUM_MODE_TYPES - 1));
//        resolve = true;
//        return true;
//    case ',':
//        mode = (ModeType)(((int)mode - 1) % ((int)NUM_MODE_TYPES - 1));
//        resolve = true;
//        return true;
//    case ' ':
//        viewer.core.is_animating = !viewer.core.is_animating;
//        if (viewer.core.is_animating)
//        {
//            resolve = true;
//        }
//        return true;
//    }
//    return false;
//}
//
//int main(int argc, char *argv[])
//{
//    using namespace Eigen;
//    using namespace std;
//    igl::readOBJ("C:\\Users\\ofir\\Desktop\\libigl\\tutorial\\data\\elephant.obj", V, F);
//    U = V;
//    MatrixXd W;
//    igl::readDMAT("C:\\Users\\ofir\\Desktop\\libigl\\tutorial\\data\\elephant-weights.dmat", W);
//    ofstream myfile;
//    myfile.open("dmat.txt");
//    myfile << W << endl;
//    igl::lbs_matrix_column(V, W, M);
//
//    // Cluster according to weights
//    VectorXi G;
//    {
//        VectorXi S;
//        VectorXd D;
//        igl::partition(W, 50, G, S, D);
//    }
//
//    // vertices corresponding to handles (those with maximum weight)
//    {
//        VectorXd maxW;
//        igl::mat_max(W, 1, maxW, b);
//    }
//
//    // Precomputation for FAST
//    cout << "Initializing Fast Automatic Skinning Transformations..." << endl;
//    // number of weights
//    const int m = W.cols();
//    Aeq.resize(m * 3, m * 3 * (3 + 1));
//    vector<Triplet<double> > ijv;
//    for (int i = 0; i < m; i++)
//    {
//        RowVector4d homo;
//        homo << V.row(b(i)), 1.;
//        for (int d = 0; d < 3; d++)
//        {
//            for (int c = 0; c < (3 + 1); c++)
//            {
//                ijv.push_back(Triplet<double>(3 * i + d, i + c * m * 3 + d * m, homo(c)));
//            }
//        }
//    }
//    Aeq.setFromTriplets(ijv.begin(), ijv.end());
//    igl::arap_dof_precomputation(V, F, M, G, arap_dof_data);
//    igl::arap_dof_recomputation(VectorXi(), Aeq, arap_dof_data);
//    // Initialize
//    MatrixXd Istack = MatrixXd::Identity(3, 3 + 1).replicate(1, m);
//    igl::columnize(Istack, m, 2, L);
//
//
//    // bounding box diagonal
//    bbd = (V.colwise().maxCoeff() - V.colwise().minCoeff()).norm();
//
//    // Plot the mesh with pseudocolors
//    igl::opengl::glfw::Viewer viewer;
//    viewer.data().set_mesh(U, F);
//    viewer.data().add_points(igl::slice(V, b, 1), sea_green);
//    viewer.data().show_lines = false;
//    viewer.callback_pre_draw = &pre_draw;
//    viewer.callback_key_down = &key_down;
//    viewer.core.is_animating = false;
//    viewer.core.animation_max_fps = 30.;
//    cout <<
//        "Press [space] to toggle animation." << endl <<
//        "Press '0' to reset pose." << endl <<
//        "Press '.' to switch to next deformation method." << endl <<
//        "Press ',' to switch to previous deformation method." << endl;
//    viewer.launch();
//}

//
//
//int getMaxAreaContourId(vector <vector<cv::Point>> contours) {
//    double maxArea = 0;
//    int maxAreaContourId = -1;
//    for (int j = 0; j < contours.size(); j++) {
//        double newArea = cv::contourArea(contours.at(j));
//        if (newArea > maxArea) {
//            maxArea = newArea;
//            maxAreaContourId = j;
//        } // End if
//    } // End for
//    return maxAreaContourId;
//} // End function

#pragma region deformation tries

//  ////read skeleton from unity
//  //fileHandle = CreateFileW(TEXT(L"\\\\.\\pipe\\my-very-cool-pipe-example"), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);

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

//  using namespace Eigen;
//  using namespace std;
//  using namespace cv;

//  //Eigen::MatrixXd C3,V3;
//  //Eigen::MatrixXi BE3,T3,F3;
//  //igl::readTGF("C:\\Users\\ofir\\Desktop\\libigl\\tutorial\\data\\hand.tgf", C3, BE3);
//  //cout << C3 << endl;
//  //cout << BE3 << endl;



//  //igl::readMESH("C:\\Users\\ofir\\Desktop\\libigl\\tutorial\\data\\hand.mesh", V3, T3, F3);
//  //


//  //// List of boundary indices (aka fixed value indices into VV)
//  //VectorXi b3;
//  //// List of boundary conditions of each weight function
//  //MatrixXd bc3;
//  //bool res3 = igl::boundary_conditions(V3, T3, C3, VectorXi(), BE3, MatrixXi(), b3, bc3);
//  //cout << res3 << endl << bc3 << endl;
//  //int a = 9090;
//  //return 0;



//  // Create the boundary of a square
//  control_points.resize(21, 2);
//  BE.resize(20, 2);
//  control_points_index.resize(21, 1);
//  V.resize(contour_size , 2);
//  E.resize(contour_size , 2);

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

//  BE << 0, 1, 1, 2, 2, 3, 3, 4,
//      0, 5, 5, 6, 6, 7, 7, 8,
//      0, 9, 9, 10, 10, 11, 11, 12,
//      0, 13, 13, 14, 14, 15, 15, 16,
//      0, 17, 17, 18, 18, 19, 19, 20; //skeleton connectivity

//  for (size_t i = 0; i < contour_size; i++)
//  {
//      V.row(i) << contour[i].x, contour[i].y;// , 0;
//      E.row(i) << i, i + 1;
//  }
//  E.row(contour_size - 1) << contour_size - 1, 0;

///*  for (size_t i = contour_size; i < contour_size + 21; i++)
//  {
//      V.row(i) << control_points.row(i - contour_size);
//      E.row(i) << i, i;
//      control_points_index.row(i - contour_size) << i;
//  }*/

//  //V.row(contour.size()-1) << contour[contour.size() - 1].x, contour[contour.size() - 1].y;
//  /*control_points_moved << V.row(0) + RowVector3d(0, 50, 0);*/


//  // Triangulate the interior
//  // a0.005 means that the area of each triangle should
//  // not be greater than 0.005
//  // q means that no angles will be smaller than 20 degrees
//  // for a detailed set of commands please refer to:
//  // https://www.cs.cmu.edu/~quake/triangle.switch.html
//  

//  igl::triangle::triangulate(V, E, H, "a100q", V2, F2);



//  // List of boundary indices (aka fixed value indices into VV)
//  VectorXi bi;
//  // List of boundary conditions of each weight function
//  MatrixXd bc, W, zeros;
//  //to 3d
// /* zeros.setZero(V2.rows(), 1);
//  V2.conservativeResize(V2.rows(), V2.cols() + 1);
//  V2.col(V2.cols() - 1) = zeros;

//  zeros.setZero(control_points.rows(), 1);
//  control_points.conservativeResize(control_points.rows(), control_points.cols() + 1);
//  control_points.col(control_points.cols() - 1) = zeros;*/

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

//  //if (!igl::copyleft::tetgen::mesh_with_skeleton(V2, F2, control_points, VectorXi(), BE, MatrixXi(), 10, VV, TT, FF))
//  //{
//  //    cout << "Tetgen Failed!" << endl;
//  //    return 1;
//  //}

//  //
  ////igl::boundary_conditions(V2, F2, control_points, VectorXi(), BE, MatrixXi(), bi, bc);
  //bool res = igl::boundary_conditions(V2, F2, control_points, VectorXi(), BE, MatrixXi(), bi, bc);
  //cout << res << endl << bc << endl;
  //// compute BBW weights matrix
  //igl::BBWData bbw_data;
  //igl::mosek::MosekData md;
  //// only a few iterations for sake of demo
  //bbw_data.active_set_params.max_iter = 8;
  //bbw_data.verbosity = 2;
  //if (!igl::mosek::bbw(V2, F2, bi, bc, bbw_data,md, W))
  ////if (!igl::bbw(TV, TT, bi, bc, bbw_data, W))
  //{
  //    return EXIT_FAILURE;
  //}

//  // Plot the generated mesh
//  igl::opengl::glfw::Viewer viewer;
//  viewer.data().set_mesh(V2, F2);
//  viewer.launch();

//  return 0;  
#pragma endregion
