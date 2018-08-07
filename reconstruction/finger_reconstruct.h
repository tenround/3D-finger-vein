//
// Created by xhb on 18-7-27.
//

#ifndef INC_3D_FINGER_VEIN_FINGER_RECONSTRUCT_H
#define INC_3D_FINGER_VEIN_FINGER_RECONSTRUCT_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <opencv2/opencv.hpp>

#include "ellipse_optimize.h"
#include "sub_ellipse_optimize.h"
#include "matplotlibcpp.h"
#include "coor_plot.h"
#include "../utils/utils.h"

//#define pi 3.1415926
#define pi M_PI

using namespace Eigen;
using namespace std;

class Finger_Reconstruct {
public:
    // 变量
    double L;
    Eigen::MatrixXf C1_M1, C1_M2, C2_M1, C2_M2, C3_M1, C3_M2;
    Eigen::MatrixXf C_M1, C1_M2_, C2_M2_, C3_M2_;
    Eigen::MatrixXf H1, H2, H3;
    Eigen::MatrixXf P1, P2, P3;
    Eigen::MatrixXf coor_c1, coor_c2, coor_c3;

    double k1, k2, k3, k4, k5, k6;
    double bias1, bias2, bias3, bias4, bias5, bias6;

    vector<MatrixXf> coor_c;
    vector<double> params;

    vector<vector<double>> X_arr;
    vector<double> z_arr;

    vector<vector<double>> X_3D;
    vector<vector<double>> Y_3D;
    vector<vector<double>> Z_3D;

    // 函数
    Finger_Reconstruct();       // 构造函数
    void set_cam_param();       // 设置3个相机的内外参以及其他参数
    void print_cam_param();     // 打印相机参数
    cv::Mat img_rectify(cv::Mat src, Eigen::MatrixXf H);    // 图像校正
    void reconstruct(vector<vector<int>> edge_arr);     // 三维重建
    void calc_param(int I1_u, int I1_b, int I2_u, int I2_b, int I3_u, int I3_b);    // 计算坐标系以及约束线参数
    void convert_3d_coor(double n);

private:
    // global b c;
    // global init_center_x init_center_y init_radius;
    double b, c;
    double init_center_x, init_center_y, init_radius;
    double poi_var;




};


#endif //INC_3D_FINGER_VEIN_FINGER_RECONSTRUCT_H
