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

#include "../reconstruction/coor_plot.h"
#include "../utils/utils.h"

#define pi M_PI

using namespace std;
using namespace Eigen;

namespace map_texture{
    extern vector<vector<uchar>> texture_3D;
    extern cv::Mat texture_img;
    extern vector<vector<float>> x_3D;
    extern vector<vector<float>> y_3D;
    extern vector<vector<float>> z_3D;
    // 投影纹理图
    extern cv::Mat texture_prj;
    // 投影深度图（原始）
    extern vector<vector<float>> depth_prj;
    // 投影深度图（归一化）
    extern cv::Mat depth_prj_img;

    void set_params(vector<MatrixXf> coor, float n);

    void map_texture(vector<vector<float>> X_arr, vector<float> z_arr, vector<cv::Mat> img_list, vector<vector<int>> edge_arr, vector<MatrixXf> P_list);

    int compute_edge_index(float rx, float ry, Vector3f norm_p_contact, float step);

    vector<uchar> one_ellipse_texture(vector<int> edge_idx_list, vector<cv::Mat> img_list, vector<vector<int>> edge_arr, float step,
                                             float rx, float ry, Matrix3f T, float z_arr, vector<MatrixXf> P_list);

    void projection(float n_theta, float n_len);

    Vector3f rotateArbitraryAxis(Vector3f xyz_src, Vector3f axis, float theta);

    Vector2f cart2pol(float x, float y);

    template<class T>
    vector<size_t> sort_index(vector<T> & a);

    void sort_rows( vector<float> &theta_3D, vector<float> &rho_3D, vector<float> &z_new, vector<uchar> &texture_new, int col);

    cv::Mat mat2gray(vector<vector<float>> mat);

    void projection_2(float n_theta, float n_len);

    template<class T>
    T find_min_nonzero(vector<T> & src);
}


#endif //INC_3D_FINGER_VEIN_V3_MAP_TEXTURE_H
