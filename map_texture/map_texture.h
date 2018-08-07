//
// Created by xhb on 18-8-6.
//

#ifndef INC_3D_FINGER_VEIN_V3_MAP_TEXTURE_H
#define INC_3D_FINGER_VEIN_V3_MAP_TEXTURE_H

#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <numeric>

#define pi M_PI

using namespace std;
using namespace Eigen;

namespace map_texture{
    extern vector<vector<uchar>> texture_3D;
    extern cv::Mat texture_img;
    extern vector<vector<double>> x_3D;
    extern vector<vector<double>> y_3D;
    extern vector<vector<double>> z_3D;

    void set_params(vector<MatrixXf> coor, double n);
    void map_texture(vector<vector<double>> X_arr, vector<double> z_arr, vector<cv::Mat> img_list, vector<vector<int>> edge_arr, vector<MatrixXf> P_list);
    int compute_edge_index(double rx, double ry, Vector3f norm_p_contact, double step);
    vector<uchar> one_ellipse_texture(vector<int> edge_idx_list, vector<cv::Mat> img_list, vector<vector<int>> edge_arr, double step,
                                             double rx, double ry, Matrix3f T, double z_arr, vector<MatrixXf> P_list);
}


#endif //INC_3D_FINGER_VEIN_V3_MAP_TEXTURE_H
