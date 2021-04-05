#pragma once
#include "opencv2/core/cvstd_wrapper.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <opencv2/objdetect.hpp>

#include <igl/png/writePNG.h>
#include <igl/png/readPNG.h>
#include <igl_stb_image.cpp>
#include <igl/cat.h>
#include <igl/cotmatrix.h>
#include <igl/matrix_to_list.h>
#include <igl/parula.h>
#include <igl/remove_unreferenced.h>
#include <igl/triangle/triangulate.h>
#include <igl/slice.h>
#include <igl/grad.h>
#include <igl/PI.h>
#include <igl/partition.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include <Eigen/OrderingMethods>



class FrameDeformation
{
    void LaplacianDeformationOperator(Eigen::MatrixXd& constraint_vertex_values)
    {
        Eigen::MatrixXd delta_coordinates_with_constraints;

        delta_coordinates = laplacian * U;
        igl::cat(1, delta_coordinates, constraint_vertex_values, delta_coordinates_with_constraints);

        auto least_squares_x = solver.solve(delta_coordinates_with_constraints.col(0));
        auto least_squares_y = solver.solve(delta_coordinates_with_constraints.col(1));
        U.col(0) << Eigen::MatrixXd(least_squares_x); //inplace change
        U.col(1) << Eigen::MatrixXd(least_squares_y);
        // Send new positions, update normals, recenter
        //viewer.data().set_mesh(U, F);
    }

    int getMaxAreaContourId(std::vector<std::vector<cv::Point>> contours) {
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

    std::vector<cv::Point> extract_contour_from_image(cv::Mat& raw)
    {
        using namespace cv;
        Mat src;
        flip(raw, src, 0);
        int r = 3 * 16 + 2, g = 4 * 16 + 10, b = 8 * 16 + 7;
        Mat binary_image = abs(src - Scalar(b, g, r));
        Mat src_gray;
        cvtColor(binary_image, src_gray, COLOR_BGR2GRAY);
        Mat canny_output;
        threshold(src_gray, canny_output, 1, 255, CV_THRESH_BINARY);
        std::vector<std::vector<Point>> contours;
        std::vector<Vec4i> hierarchy;
        findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        int max_area_contour_index = getMaxAreaContourId(contours);
        return contours[max_area_contour_index];
    }

    void ImageMatToEigen(cv::Mat image,
        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& R,
        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& G,
        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& B)
    {

        using namespace cv;
        int cols, rows, n;
        cols = image.cols;
        rows = image.rows;
        R.resize(cols, rows);
        G.resize(cols, rows);
        B.resize(cols, rows);

        for (unsigned i = 0; i < rows; ++i) {
            for (unsigned j = 0; j < cols; ++j) {
                cv::Vec3b pixel = image.at<cv::Vec3b>(i, j);
                R(j, rows - 1 - i) = (unsigned char)pixel[2];
                G(j, rows - 1 - i) = (unsigned char)pixel[1];
                B(j, rows - 1 - i) = (unsigned char)pixel[0];
            }
        }
    }

    void triangulate_contour(std::vector<cv::Point2f> input_contour, Eigen::MatrixXd input_control_points, Eigen::MatrixXd& output_vertices, Eigen::MatrixXi& output_edges)
    {
        int input_contour_size = input_contour.size();
        // Input polygon
        Eigen::MatrixXd input_vertices;
        Eigen::MatrixXi input_edges;
        Eigen::MatrixXd input_holes;

        // Create the boundary of a square

        input_vertices.resize(input_contour_size + num_of_handle_points, 2);
        input_edges.resize(input_contour_size, 2);

        for (size_t i = 0; i < input_contour_size; i++)
        {
            input_vertices.row(i) << input_contour[i].x, input_contour[i].y;
            input_edges.row(i) << i, i + 1;
        }
        input_edges.row(input_contour_size - 1) << input_contour_size - 1, 0;

        for (size_t i = input_contour_size; i < input_contour_size + num_of_handle_points; i++)
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

        /*int triangles_count = E_out.rows();
        output_edges.resize(triangles_count*3, 2);
        for (size_t i =0;i< triangles_count;i++)
        {
            auto tt = E_out.row(i);
            output_edges.row(3 * i) << tt[0], tt[1];
            output_edges.row(3 * i + 1) << tt[1], tt[2];
            output_edges.row(3 * i + 2) << tt[2], tt[0];
        }*/

        // Plot the mesh with pseudocolors
      /* igl::opengl::glfw::Viewer viewer;
       viewer.data().set_mesh(V, F);
       viewer.data().add_points(control_points, sea_green);
       viewer.launch();*/
    }

    void vector_points_to_matrix(std::vector<cv::Point2f> vector, Eigen::MatrixXd& matrix)
    {
        int num_control_points = (int)vector.size();
        matrix.resize(num_control_points, 2); //2D points
        for (int i = 0; i < num_control_points; i++)
        {
            matrix.row(i) << vector[i].x, vector[i].y;
        }
    }

    std::vector<cv::Point2f> _contour;
    int len_contour;
    // Allocate temporary buffers

    Eigen::MatrixXd _control_points,_control_target_points;
    Eigen::SparseMatrix<double>  _control_target_points_laplacian_indexes;
    //int num_of_handle_points = 21;
    int num_of_handle_points = 1;

    Eigen::SparseMatrix<double> laplacian,laplacian_with_constraints;
    Eigen::MatrixXd  delta_coordinates;

    // Solve (L+C) U` = delta+C
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    bool isInitialized = false;

public:
    Eigen::MatrixXi F;
    Eigen::MatrixXd V, U, UV;
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> _R, _G, _B/*, _A*/;

    FrameDeformation()
    {
        _control_target_points.resize(num_of_handle_points, 2);
    }

    void DeformMesh(std::vector<cv::Point2f> control_points_targets)
    {
        if (!isInitialized) return;

        for (int i = 0; i < num_of_handle_points; i++)
        {
            _control_target_points.row(i) << control_points_targets[i].x, control_points_targets[i].y;
        }
        LaplacianDeformationOperator(_control_target_points);

        // Send new positions, update normals, recenter
        //viewer.data().set_mesh(U, F);

        //save to mat
    }

    void SetMeshFromFrame(cv::Mat& frame, std::vector<cv::Point2f> control_points)
    {
        //inputs
        vector_points_to_matrix(control_points, _control_points);
        cout << _control_points << endl;
        auto contour = extract_contour_from_image(frame);
        _contour.clear();
        for(cv::Point point : contour)
        {
            _contour.push_back(cv::Point2f(point.x, point.y));
            
        }
        for (cv::Point p : control_points)
        {
            cv::circle(frame, cv::Point(p.x,frame.rows-p.y), 3, cv::Scalar(0, 0, 255), -1);
        }
        
        cv::imshow("frame", frame);
        cv::waitKey(0);
        //to mesh
        triangulate_contour(_contour, _control_points, V, F);
        len_contour = (int)_contour.size();
        cout << len_contour << endl;

        U = V;
        //viewer.data().set_mesh(U, F);

        // Draw texture
        UV = U;
        UV.col(0) /= frame.cols;
        UV.col(1) /= frame.rows;
        //viewer.data().set_uv(UV);
        ImageMatToEigen(frame, _R, _G, _B);
        //viewer.data().set_texture(_R, _G, _B);
        //viewer.data().show_texture = true;

        //viewer.launch();

        //set control handels
        _control_target_points_laplacian_indexes.resize(num_of_handle_points, V.rows());
        std::vector<Eigen::Triplet<double>> ijv;
        for (int i = 0; i < num_of_handle_points; i++)
        {
            int vertex_index_in_laplacian = len_contour + i; //since handle points is right after contour points (triangulation)
            ijv.push_back(Eigen::Triplet<double>(i, vertex_index_in_laplacian, 1));
        }
        _control_target_points_laplacian_indexes.setFromTriplets(ijv.begin(), ijv.end());

        // Compute Laplace-Beltrami operator: #V by #V
        igl::cotmatrix(U, F, laplacian);
        igl::cat(1, laplacian, _control_target_points_laplacian_indexes, laplacian_with_constraints);
        solver.compute(laplacian_with_constraints);

        isInitialized = true;
    }

    void UpdateTextureAndHandles(cv::Mat& frame, std::vector<cv::Point2f> control_points)
    {
        if (!isInitialized) return;

        //inputs

        DeformMesh(control_points);
        vector_points_to_matrix(control_points, _control_points);

        // Draw texture
        UV = U;
        UV.col(0) /= frame.cols;
        UV.col(1) /= frame.rows;

        ImageMatToEigen(frame, _R, _G, _B);
        /*viewer.data().set_uv(UV);
        viewer.data().set_texture(R, G, B);*/
    }
};