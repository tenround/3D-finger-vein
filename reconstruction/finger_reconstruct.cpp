//
// Created by xhb on 18-7-27.
//

#include "finger_reconstruct.h"

static Eigen::MatrixXf matrix_slice(Eigen::MatrixXf m, int start_row, int end_row, int start_col, int end_col);

Finger_Reconstruct::Finger_Reconstruct()
{

}

void Finger_Reconstruct::set_cam_param()
{
    L = 68;

    // C1_M1 = [730.41670   0  286.57077   0;   0   720.22414   243.39453   0; 0 0 1 0];
    C1_M1 = Eigen::MatrixXf::Zero(3, 4);
//    C1_M1(0, 0) = 730.41670;
//    C1_M1(0, 2) = 286.57077;
//    C1_M1(1, 1) = 720.22414;
//    C1_M1(1, 2) = 243.39453;
//    C1_M1(2, 2) = 1;
    C1_M1 << 730.41670, 0, 286.57077, 0,
             0, 720.22414, 243.39453, 0,
             0, 0, 1, 0;

    // C1_M2 = [0 0 1 0; -1 0 0 0; 0 1 0 L; 0 0 0 1];
    C1_M2 = Eigen::MatrixXf::Zero(4, 4);
//    C1_M2(0, 2) = 1;
//    C1_M2(1, 0) = -1;
//    C1_M2(2, 1) = 1;
//    C1_M2(2, 3) = L;
//    C1_M2(3, 3) = 1;
    C1_M2 << 0, 0, 1, 0,
            -1, 0, 0, 0,
             0, 1, 0, L,
             0, 0, 0, 1;

    Eigen::MatrixXf Rx, Ry, Rz;
    double fai_x, fai_y, fai_z;

    fai_x = pi / 60;
    // Rx = [1, 0, 0, 0; 0, cos(fai_x), sin(fai_x), 0; 0, -sin(fai_x), cos(fai_x), 0; 0, 0, 0, 1];
    Rx = Eigen::MatrixXf::Zero(4, 4);
//    Rx(0, 0) = 1;
//    Rx(1, 1) = cos(fai_x);
//    Rx(1, 2) = sin(fai_x);
//    Rx(2, 1) = -sin(fai_x);
//    Rx(2, 2) = cos(fai_x);
//    Rx(3, 3) = 1;
    Rx << 1, 0, 0, 0,
          0, cos(fai_x), sin(fai_x), 0,
          0, -sin(fai_x), cos(fai_x), 0,
          0, 0, 0, 1;
//    fai_y = -pi/200;
    // Ry = [cos(fai_y), 0, -sin(fai_y), 0; 0, 1, 0, 0; sin(fai_y), 0, cos(fai_y), 0; 0, 0, 0, 1];
//    Ry = Eigen::MatrixXf::Zero(4, 4);
//    Ry(0, 0) = cos(fai_y);
//    Ry(0, 2) = -sin(fai_y);
//    Ry(1, 1) = 1;
//    Ry(2, 0) = sin(fai_y);
//    Ry(2, 2) = cos(fai_y);
//    Ry(3, 3) = 1;
    // Rz = [cos(fai_z), sin(fai_z), 0, 0; -sin(fai_z), cos(fai_z), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
//    fai_z = pi/80;
//    Rz = Eigen::MatrixXf::Zero(4, 4);
//    Rz(0, 0) = cos(fai_z);
//    Rz(0, 1) = sin(fai_z);
//    Rz(1, 0) = -sin(fai_z);
//    Rz(1, 1) = cos(fai_z);
//    Rz(2, 2) = 1;
//    Rz(3, 3) = 1;
    C1_M2 = Rx * C1_M2;


    // C2_M1 = [726.35911   0   332.2673   0;  0  715.74161   234.1517   0;   0 0 1 0];
    C2_M1 = Eigen::MatrixXf::Zero(3, 4);
//    C2_M1(0, 0) = 726.35911;
//    C2_M1(0, 2) = 332.2673;
//    C2_M1(1, 1) = 715.74161;
//    C2_M1(1, 2) = 234.1517;
//    C2_M1(2, 2) = 1;
    C2_M1 << 726.35911, 0, 332.2673, 0,
             0, 715.74161, 234.1517, 0,
             0, 0, 1, 0;

    // C2_M2 = [0 0 1 0; 0.5   -0.866   0   0; -0.866   -0.5   0 L; 0 0 0 1];
    C2_M2 = Eigen::MatrixXf::Zero(4, 4);
//    C2_M2(0, 2) = 1;
//    C2_M2(1, 0) = 0.5;
//    C2_M2(1, 1) = -0.886;
//    C2_M2(2, 0) = -0.886;
//    C2_M2(2, 1) = -0.5;
//    C2_M2(2, 3) = L;
//    C2_M2(3, 3) = 1;
    C2_M2 << 0, 0, 1, 0,
             0.5, -0.866, 0, 0,
             -0.866, -0.5, 0, L,
             0, 0, 0, 1;

    // Rx、Ry、Rz、fai_x、fai_y、fai_z前面已经定义过了，重新初始化即可
    fai_x = -pi / 100;
    Rx = Eigen::MatrixXf::Zero(4, 4);
    // Rx = [1, 0, 0, 0; 0, cos(fai_x), sin(fai_x), 0; 0, -sin(fai_x), cos(fai_x), 0; 0, 0, 0, 1];
//    Rx(0, 0) = 1;
//    Rx(1, 1) = cos(fai_x);
//    Rx(1, 2) = sin(fai_x);
//    Rx(2, 1) = -sin(fai_x);
//    Rx(2, 2) = cos(fai_x);
//    Rx(3, 3) = 1;
    Rx << 1, 0, 0, 0,
          0, cos(fai_x), sin(fai_x), 0,
          0, -sin(fai_x), cos(fai_x), 0,
          0, 0, 0, 1;
    fai_y = pi / 100;
    // Ry = [cos(fai_y), 0, -sin(fai_y), 0; 0, 1, 0, 0; sin(fai_y), 0, cos(fai_y), 0; 0, 0, 0, 1];
    Ry = Eigen::MatrixXf::Zero(4, 4);
//    Ry(0, 0) = cos(fai_y);
//    Ry(0, 2) = -sin(fai_y);
//    Ry(1, 1) = 1;
//    Ry(2, 0) = sin(fai_y);
//    Ry(2, 2) = cos(fai_y);
//    Ry(3, 3) = 1;
    Ry << cos(fai_y), 0, -sin(fai_y), 0,
          0, 1, 0, 0,
          sin(fai_y), 0, cos(fai_y), 0,
          0, 0, 0, 1;
    C2_M2 = Ry*Rx*C2_M2;


    // C3_M1 = [710.75252   0  325.54501  0; 0 705.26936  262.79649  0;  0 0 1 0];
    C3_M1 = Eigen::MatrixXf::Zero(3, 4);
//    C3_M1(0, 0) = 710.75252;
//    C3_M1(0, 2) = 325.54501;
//    C3_M1(1, 1) = 705.26936;
//    C3_M1(1, 2) = 262.79649;
//    C3_M1(2, 2) = 1;
    C3_M1 << 710.75252, 0, 325.54501, 0,
             0, 705.26936, 262.79649, 0,
             0, 0, 1, 0;

    // C3_M2 = [ 0 0 1 0; 0.5  0.866  0 0; 0.866 -0.5 0 L; 0 0 0 1];
    C3_M2 = Eigen::MatrixXf::Zero(4, 4);
//    C3_M2(0, 2) = 1;
//    C3_M2(1, 0) = 0.5;
//    C3_M2(1, 1) = 0.866;
//    C3_M2(2, 0) = 0.866;
//    C3_M2(2, 1) = -0.5;
//    C3_M2(2, 3) = L;
//    C3_M2(3, 3) = 1;
    C3_M2 << 0, 0, 1, 0,
             0.5, 0.866, 0, 0,
             0.866, -0.5, 0, L,
             0, 0, 0, 1;

    fai_x = -pi/800;
    // Rx = [1, 0, 0, 0; 0, cos(fai_x), sin(fai_x), 0; 0, -sin(fai_x), cos(fai_x), 0; 0, 0, 0, 1];
    Rx = Eigen::MatrixXf::Zero(4, 4);
//    Rx(0, 0) = 1;
//    Rx(1, 1) = cos(fai_x);
//    Rx(1, 2) = sin(fai_x);
//    Rx(2, 1) = -sin(fai_x);
//    Rx(2, 2) = cos(fai_x);
//    Rx(3, 3) = 1;
    Rx << 1, 0, 0, 0,
          0, cos(fai_x), sin(fai_x), 0,
          0, -sin(fai_x), cos(fai_x), 0,
          0, 0, 0, 1;
    fai_y = pi/75;
    // Ry = [cos(fai_y), 0, -sin(fai_y), 0; 0, 1, 0, 0; sin(fai_y), 0, cos(fai_y), 0; 0, 0, 0, 1];
    Ry = Eigen::MatrixXf::Zero(4, 4);
//    Ry(0, 0) = cos(fai_y);
//    Ry(0, 2) = -sin(fai_y);
//    Ry(1, 1) = 1;
//    Ry(2, 0) = sin(fai_y);
//    Ry(2, 2) = cos(fai_y);
//    Ry(3, 3) = 1;
    Ry << cos(fai_y), 0, -sin(fai_y), 0,
          0, 1, 0, 0,
          sin(fai_y), 0, cos(fai_y), 0,
          0, 0, 0, 1;
    C3_M2 = Ry*Rx*C3_M2;


    // C_M1 = [720, 0, 320; 0, 720, 240; 0, 0, 1];
    C_M1 = Eigen::MatrixXf::Zero(3, 3);
//    C_M1(0, 0) = 720;
//    C_M1(0, 2) = 320;
//    C_M1(1, 1) = 720;
//    C_M1(1, 2) = 240;
//    C_M1(2, 2) = 1;
    C_M1 << 720, 0, 320,
            0, 720, 240,
            0, 0, 1;

    // C1_M2_ = [0 0 1 0; -1 0 0 0; 0 1 0 L; 0 0 0 1];
    C1_M2_ = Eigen::MatrixXf::Zero(4, 4);
//    C1_M2_(0, 2) = 1;
//    C1_M2_(1, 0) = -1;
//    C1_M2_(2, 1) = 1;
//    C1_M2_(2, 3) = L;
//    C1_M2_(3, 3) = 1;
    C1_M2_ << 0, 0, 1, 0,
             -1, 0, 0, 0,
              0, 1, 0, L,
              0, 0, 0, 1;
    // C2_M2_ = [0 0 1 0; 0.5   -0.866   0   0; -0.866   -0.5   0 L; 0 0 0 1];
    C2_M2_ = Eigen::MatrixXf::Zero(4, 4);
//    C2_M2_(0, 2) = 1;
//    C2_M2_(1, 0) = 0.5;
//    C2_M2_(1, 1) = -0.886;
//    C2_M2_(2, 0) = -0.866;
//    C2_M2_(2, 1) = -0.5;
//    C2_M2_(2, 3) = L;
//    C2_M2_(3, 3) = 1;
    C2_M2_ << 0, 0, 1, 0,
              0.5, -0.866, 0, 0,
              -0.866, -0.5, 0, L,
              0, 0, 0, 1;
    // C3_M2_ = [ 0 0 1 0; 0.5  0.866  0 0; 0.866 -0.5 0 L; 0 0 0 1];
    C3_M2_ = Eigen::MatrixXf::Zero(4, 4);
//    C3_M2_(0, 2) = 1;
//    C3_M2_(1, 0) = 0.5;
//    C3_M2_(1, 1) = 0.866;
//    C3_M2_(2, 0) = 0.866;
//    C3_M2_(2, 1) = -0.5;
//    C3_M2_(2, 3) = L;
//    C3_M2_(3, 3) = 1;
    C3_M2_ << 0, 0, 1, 0,
              0.5, 0.866, 0, 0,
              0.866, -0.5, 0, L,
              0, 0, 0, 1;

    // H1 = C_M1 * C1_M2_(1:3, 1:3) * C1_M2(1:3, 1:3)' * inv(C1_M1(:,1:3));
//    Eigen::MatrixXf temp_C1_M1 = matrix_slice(C1_M1, 1, 3, 1, 3);
//    Eigen::MatrixXf temp_C1_M2 = matrix_slice(C1_M2, 1, 3, 1, 3);
//    Eigen::MatrixXf temp_C1_M2_ = matrix_slice(C1_M2_, 1, 3, 1, 3);
//    H1 = C_M1 * temp_C1_M2_ * temp_C1_M2.transpose() * temp_C1_M1.inverse();
    H1 = C_M1 * C1_M2_.block(0, 0, 3, 3) * C1_M2.block(0, 0, 3, 3).transpose() * C1_M1.block(0, 0, 3, 3).inverse();

    // H2 = C_M1 * C2_M2_(1:3, 1:3) * C2_M2(1:3, 1:3)' * inv(C2_M1(:,1:3));
//    Eigen::MatrixXf temp_C2_M1 = matrix_slice(C2_M1, 1, 3, 1, 3);
//    Eigen::MatrixXf temp_C2_M2 = matrix_slice(C2_M2, 1, 3, 1, 3);
//    Eigen::MatrixXf temp_C2_M2_ = matrix_slice(C2_M2_, 1, 3, 1, 3);
//    H2 = C_M1 * temp_C2_M2_ * temp_C2_M2.transpose() * temp_C2_M1.inverse();
    H2 = C_M1 * C2_M2_.block(0, 0, 3, 3) * C2_M2.block(0, 0, 3, 3).transpose() * C2_M1.block(0, 0, 3, 3).inverse();

    // H3 = C_M1 * C3_M2_(1:3, 1:3) * C3_M2(1:3, 1:3)' * inv(C3_M1(:,1:3));
//    Eigen::MatrixXf temp_C3_M1 = matrix_slice(C3_M1, 1, 3, 1, 3);
//    Eigen::MatrixXf temp_C3_M2 = matrix_slice(C3_M2, 1, 3, 1, 3);
//    Eigen::MatrixXf temp_C3_M2_ = matrix_slice(C3_M2_, 1, 3, 1, 3);
//    H3 = C_M1 * temp_C3_M2_ * temp_C3_M2.transpose() * temp_C3_M1.inverse();
    H3 = C_M1 * C3_M2_.block(0, 0, 3, 3) * C3_M2.block(0, 0, 3, 3).transpose() * C3_M1.block(0, 0, 3, 3).inverse();

    // C_M1 = [C_M1, zeros(3,1)];
    Eigen::MatrixXf temp_C_M1 = Eigen::MatrixXf::Zero(3, 4);
    temp_C_M1(0, 0) = C_M1(0, 0);  temp_C_M1(0, 1) = C_M1(0, 1);  temp_C_M1(0, 2) = C_M1(0, 2);
    temp_C_M1(1, 0) = C_M1(1, 0);  temp_C_M1(1, 1) = C_M1(1, 1);  temp_C_M1(1, 2) = C_M1(1, 2);
    temp_C_M1(2, 0) = C_M1(2, 0);  temp_C_M1(2, 1) = C_M1(2, 1);  temp_C_M1(2, 2) = C_M1(2, 2);
    // P1 = C_M1*C1_M2_;
    // P2 = C_M1*C2_M2_;
    // P3 = C_M1*C3_M2_;
    P1 = temp_C_M1 * C1_M2_;
    P2 = temp_C_M1 * C2_M2_;
    P3 = temp_C_M1 * C3_M2_;

//    coor_c1 = [ 0, -68 ];
//    coor_c2 = [ 58.8897, 34 ];
//    coor_c3 = [ -58.8897, 34 ];
    coor_c1 = Eigen::MatrixXf::Zero(1, 2);
    coor_c1(0, 0) = 0;           coor_c1(0, 1) = -68;
    coor_c2 = Eigen::MatrixXf::Zero(1, 2);
    coor_c2(0, 0) = 58.8897;     coor_c2(0, 1) = 34;
    coor_c3 = Eigen::MatrixXf::Zero(1, 2);
    coor_c3(0, 0) = -58.8897;    coor_c3(0, 1) = 34;
}

void Finger_Reconstruct::print_cam_param()
{
    cout << "C1_M1:" << endl << C1_M1 << endl << endl;
    cout << "C1_M2:" << endl << C1_M2 << endl << endl;
    cout << "C2_M1:" << endl << C2_M1 << endl << endl;
    cout << "C2_M2:" << endl << C2_M2 << endl << endl;
    cout << "C3_M1:" << endl << C3_M1 << endl << endl;
    cout << "C3_M2:" << endl << C3_M2 << endl << endl;

    cout << "C_M1:" << endl << C_M1 << endl << endl;
    cout << "C1_M2_:" << endl << C1_M2_ << endl << endl;
    cout << "C2_M2_:" << endl << C2_M2_ << endl << endl;
    cout << "C3_M2_:" << endl << C3_M2_ << endl << endl;

    cout << "H1:" << endl << H1 << endl << endl;
    cout << "H2:" << endl << H2 << endl << endl;
    cout << "H3:" << endl << H3 << endl << endl;

    cout << "P1:" << endl << P1 << endl << endl;
    cout << "P2:" << endl << P2 << endl << endl;
    cout << "P3:" << endl << P3 << endl << endl;

    cout << "coor_c1:" << endl << coor_c1 << endl << endl;
    cout << "coor_c2:" << endl << coor_c2 << endl << endl;
    cout << "coor_c3:" << endl << coor_c3 << endl << endl;
}

static Eigen::MatrixXf matrix_slice(Eigen::MatrixXf m, int start_row, int end_row, int start_col, int end_col)
{
    Eigen::MatrixXf res;
    res = Eigen::MatrixXf::Zero((end_row - start_row + 1), (end_col - start_col + 1));
    int i = start_row - 1;
    int j = start_col - 1;
    for(i=start_row - 1; i<end_row; i++) {
        for (j = start_col - 1; j < end_col; j++) {
            res(i, j) = m(i, j);
        }
    }

    return res;
}

cv::Mat Finger_Reconstruct::img_rectify(cv::Mat src, Eigen::MatrixXf H)
{
    // 原始图相关大小
    int rows = src.rows;    // 480
    int cols = src.cols;    // 640

    // 计算一系列x、y、z，并求出起对应图像中的坐标
    Eigen::MatrixXf x = Eigen::MatrixXf::Zero(1, rows * cols);
    Eigen::MatrixXf y = Eigen::MatrixXf::Zero(1, rows * cols);
    for(int r=1;r<=rows;r++)
    {
        for(int c=1;c<=cols;c++)
        {
            x(0, (r-1) * cols + c - 1) = r;
            y(0, (r-1) * cols + c - 1) = c;
//            x(r-1, c-1) = r;
//            y(r-1, c-1) = c;
        }
    }
    Eigen::MatrixXf z = Eigen::MatrixXf::Ones(1, rows * cols);
    Eigen::MatrixXf xyz = Eigen::MatrixXf::Zero(3, rows * cols);
    xyz << y, x, z;
    Eigen::MatrixXf coor = H.inverse() * xyz;
    // 遍历coor，将其转换为整数
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<rows * cols;j++)
        {
            coor(i, j) = round(coor(i, j));
        }
    }
//    cv::Mat dst;
//    dst.create(cv::Size(cols, rows), CV_8UC1);
//    dst = src.clone();

    cv::Mat dst(rows, cols, CV_8UC1, cv::Scalar(0));
    for(int r=1;r<=rows;r++)
    {
        for(int c=1;c<=cols;c++)
        {
            if(coor(0, (r-1) * cols + c - 1)>0 && coor(1, (r-1) * cols + c - 1)>0 &&
                    coor(0, (r-1) * cols + c - 1) <= 640 && coor(1, (r-1) * cols + c - 1) <= 480)
            {
                 dst.at<uchar>(r-1, c-1) = src.at<uchar>(coor(1, (r-1) * cols + c - 1), coor(0, (r-1) * cols + c - 1));
            }
        }
    }

    return dst;
}

void Finger_Reconstruct::reconstruct(vector<vector<int>> edge_arr)
{
    int min_y = 91;
    int max_y = 490;
    float interval = 1;
    // global b c;
    // global init_center_x init_center_y init_radius;
    int N_len = edge_arr[0].size();
    int step = 40;
    bool disp_flag = false;
    bool save_flag = false;

    // 索引表
    vector<int> i_list;
    for(int i=0;i<N_len;i+=step)
    {
        i_list.push_back(i);
    }
    i_list.push_back(N_len-1);

    int i;
    int I1_u, I1_b, I2_u, I2_b, I3_u, I3_b;
    vector<double> x(3);
    vector<double> last_x;
    vector<double> rx_list, ry_list, diff_c_list;
    double last_rx, last_ry;
    vector<vector<double>> X;
    vector<double> z;
    coor_plot cp = coor_plot();
    for(int idx=0;idx<i_list.size();idx++)
    {
        i = i_list[idx];    // 从索引表中取出索引值

        I1_u = edge_arr[0][i];  I1_b = edge_arr[1][i];
        I2_u = edge_arr[2][i];  I2_b = edge_arr[3][i];
        I3_u = edge_arr[4][i];  I3_b = edge_arr[5][i];

        // 计算约束线以及初始椭圆
        calc_param(I1_u, I1_b, I2_u, I2_b, I3_u, I3_b);

        // 绘制坐标系
        if(disp_flag || save_flag)
        {
            cp.create_figure();
            cp.plot_constraint_line(k1, bias1);
            cp.plot_constraint_line(k2, bias2);
            cp.plot_constraint_line(k3, bias3);
            cp.plot_constraint_line(k4, bias4);
            cp.plot_constraint_line(k5, bias5);
            cp.plot_constraint_line(k6, bias6);

        }

//        cout << idx << endl;
//        cout << i << endl;
        // idx为循环索引，i为图像采样点横坐标

        if(idx == 0) {      // 第一次先求解整个椭圆
            // 设置坐标系以及全局变量
            ellipse_opt::set_params(coor_c, params);

            // 给定优化问题的初始值
            vector<double> x0(5);
            x0[0] = 1.8;  x0[1] = 0;  x0[2] = -2 * init_center_x;
            x0[3] = -2 * init_center_y;  x0[4] = -init_radius;
            vector<double> x1(5);
            x1[0] = 2.0;  x1[1] = 0;  x1[2] = -2 * init_center_x;
            x1[3] = -2 * init_center_y;  x1[4] = -init_radius;

            last_x = x0;

//            cout << "优化前 k1：" << k1 << endl;
//            cout << "优化前 k2：" << k2 << endl;
//            cout << "优化前 k3：" << k3 << endl;
//            cout << "优化前 k4：" << k4 << endl;
//            cout << "优化前 k5：" << k5 << endl;
//            cout << "优化前 k6：" << k6 << endl;

//            cout << "优化前 b：" << x0[0] << endl;
//            cout << "优化前 c：" << x0[1] << endl;
//            cout << "优化前 d：" << x0[2] << endl;
//            cout << "优化前 e：" << x0[3] << endl;
//            cout << "优化前 r：" << x0[4] << endl;

            // 求解优化问题
            vector<double> temp = ellipse_opt::solve_3(x0, x1);
            b = temp[0];  c = temp[1];
            x[0] = temp[2];
            x[1] = temp[3];
            x[2] = temp[4];

            // 绘制初始椭圆
            if(disp_flag || save_flag)
            {
//                vector<double> ellipse_params(5);
//                ellipse_params[0] = x0[0];
//                ellipse_params[1] = x0[1];
//                ellipse_params[2] = x0[2];
//                ellipse_params[3] = x0[3];
//                ellipse_params[4] = x0[4];
//                cp.plot_ellipse(ellipse_params);
            }
        }
        else {              // 之后锁定椭圆的b、c参数，求解剩余的参数
            // 设置坐标系以及全局变量
            vector<double> b_c;
            b_c.push_back(b);
            b_c.push_back(c);
            sub_ellipse_opt::set_params(coor_c, params, b_c);

            // 给定优化问题的初始值
            vector<double> x0(3);
            x0 = x;
            vector<double> x1(3);
            x1[0] = -2 * init_center_x;
            x1[1] = -2 * init_center_y;
            x1[2] = -init_radius;

            last_x = x0;

            // 求解优化问题
            vector<double> temp = sub_ellipse_opt::solve_3(x0, x1);
            x = temp;
        }

        double d = x[0], e = x[1], f = x[2];
        double m = (b*d - c*e) / (-c*c + b);
        double n = (e-c*d) / (-c*c + b);
        m=m/2; n = n/2;
        double alpha = atan(2*c/(b-1)) / 2;
        if (alpha>pi/4)
            alpha = pi/2 - alpha;
        else if (alpha < -pi/4)
            alpha = -pi/2 - alpha;
        double aa = cos(alpha)*(cos(alpha) - c*sin(alpha)) - sin(alpha)*(c*cos(alpha) - b*sin(alpha));
        double bb = cos(alpha)*(b*cos(alpha) + c*sin(alpha)) + sin(alpha)*(sin(alpha) + c*cos(alpha));
        double ff = f + n*(b*n - e + c*m) + m*(m - d + c*n) - n*(c*cos(alpha) - b*sin(alpha)) - m*(cos(alpha) - c*sin(alpha)) - n*(b*cos(alpha) + c*sin(alpha)) - m*(sin(alpha) + c*cos(alpha));
        double rx = sqrt(-ff/aa);
        double ry = sqrt(-ff/bb);
        double ecc = rx / ry;
        double diff_c = (m - init_center_x)*(m - init_center_x) + (n - init_center_y)*(n - init_center_y);

//        cout << "m: " << m << endl;
//        cout << "init_center_x: " << init_center_x << endl;
//        cout << "n: " << n << endl;
//        cout << "init_center_y: " << init_center_y << endl;
//        cout << "rx: " << rx << endl;
//        cout << "ry: " << ry << endl;
//        cout << "diff_c: " << diff_c << endl;

        // 绘制椭圆
        if(disp_flag || save_flag)
        {
            vector<double> ellipse_params(5);
            ellipse_params[0] = b;
            ellipse_params[1] = c;
            ellipse_params[2] = x[0];
            ellipse_params[3] = x[1];
            ellipse_params[4] = x[2];
            cp.plot_ellipse(ellipse_params);
        }

        // 显示图像
        if(disp_flag)
        {
            cp.show();
        }

        // 保存图像
        if(save_flag)
        {
            string idx_str;
            int2str(idx, idx_str);
            cp.save(idx_str);
        }

        if(save_flag || disp_flag)
        {
            cp.close();
        }

        X.push_back(x);

        z.push_back(i + min_y);
    }

    cout << "重建成功" << endl;

    // 插值操作
//    vector<double> z_arr;
    vector<double> b_arr, c_arr;
    vector<double> d_arr_src, e_arr_src, f_arr_src;
    vector<double> d_arr, e_arr, f_arr;
    for(int i=z[0];i<=z[z.size()-1];i++)
    {
        z_arr.push_back((double)i);
        b_arr.push_back(b);
        c_arr.push_back(c);
    }
    for(int i=0;i<X.size();i++)
    {
        vector<double> temp = X[i];
        d_arr_src.push_back(temp[0]);
        e_arr_src.push_back(temp[1]);
        f_arr_src.push_back(temp[2]);
    }
    d_arr = interp1_linear(d_arr_src, z, z_arr);
    e_arr = interp1_linear(e_arr_src, z, z_arr);
    f_arr = interp1_linear(f_arr_src, z, z_arr);

//    vector<vector<double>> X_arr;
    X_arr.push_back(b_arr);
    X_arr.push_back(c_arr);
    X_arr.push_back(d_arr);
    X_arr.push_back(e_arr);
    X_arr.push_back(f_arr);
}


void Finger_Reconstruct::calc_param(int I1_u, int I1_b, int I2_u, int I2_b, int I3_u, int I3_b)
{
    Eigen::Matrix3f m1_2, m2_2, m3_2;

    // m1_2 = [-1, 0, 0; 0, 1, 68; 0, 0, 1];
    // m2_2 = [0.5, -0.866, 0; -0.866, -0.5, 68; 0, 0, 1];
    // m3_2 = [0.5, 0.866, 0; 0.866, -0.5, 68; 0, 0, 1];
    m1_2 << -1, 0, 0,
             0, 1, 68,
             0, 0, 1;
    m2_2 <<  0.5, -0.866, 0,
             -0.866, -0.5, 68,
             0, 0, 1;
    m3_2 <<  0.5, 0.866, 0,
             0.866, -0.5, 68,
             0, 0, 1;

    Eigen::MatrixXf temp = Eigen::MatrixXf::Zero(3, 1);
    temp << (I1_u - 240),
            720,
            1;
    Eigen::MatrixXf P = m1_2.inverse() * temp;
    // 1号相机
    double x_c = coor_c1(0, 0);
    double y_c = coor_c1(0, 1);
    double x_u = P(0, 0);
    double y_u = P(1, 0);
    k1 = (y_u - y_c) / (x_u - x_c);
    bias1 = y_c - k1 * x_c;
//    Eigen::MatrixXf L1 = Eigen::MatrixXf::Zero(1, 3);
    Vector3f L1;
    L1 << k1, -1, bias1;

    temp << (I1_b - 240),
            720,
            1;
    P = m1_2.inverse() * temp;
//    x_c = coor_c1(0, 0);
//    y_c = coor_c1(0, 1);
    x_u = P(0, 0);
    y_u = P(1, 0);
    k2 = (y_u - y_c) / (x_u - x_c);
    bias2 = y_c - k2 * x_c;
//    Eigen::MatrixXf L2 = Eigen::MatrixXf::Zero(1,3);
    Vector3f L2;
    L2 << k2, -1, bias2;

    // 2号相机
    temp << (I2_u - 240),
            720,
            1;
    P = m2_2.inverse() * temp;
    x_c = coor_c2(0, 0);
    y_c = coor_c2(0, 1);
    x_u = P(0, 0);
    y_u = P(1, 0);
    k3 = (y_u - y_c) / (x_u - x_c);
    bias3 = y_c - k3 * x_c;
//    Eigen::MatrixXf L3 = Eigen::MatrixXf::Zero(1, 3);
    Vector3f L3;
    L3 << k3, -1, bias3;

    temp << (I2_b - 240),
            720,
            1;
    P = m2_2.inverse() * temp;
//    x_c = coor_c2(0, 0);
//    y_c = coor_c2(0, 1);
    x_u = P(0, 0);
    y_u = P(1, 0);
    k4 = (y_u - y_c) / (x_u - x_c);
    bias4 = y_c - k4 * x_c;
//    Eigen::MatrixXf L4 = Eigen::MatrixXf::Zero(1, 3);
    Vector3f L4;
    L4 << k4, -1, bias4;

    // 3号相机
    temp << (I3_u - 240),
            720,
            1;
    P = m3_2.inverse() * temp;
    x_c = coor_c3(0, 0);
    y_c = coor_c3(0, 1);
    x_u = P(0, 0);
    y_u = P(1, 0);
    k5 = (y_u - y_c) / (x_u - x_c);
    bias5 = y_c - k5 * x_c;
//    Eigen::MatrixXf L5 = Eigen::MatrixXf::Zero(1, 3);
    Vector3f L5;
    L5 << k5, -1, bias5;

    temp << (I3_b - 240),
            720,
            1;
    P = m3_2.inverse() * temp;
//    x_c = coor_c3(0, 0);
//    y_c = coor_c3(0, 1);
    x_u = P(0, 0);
    y_u = P(1, 0);
    k6 = (y_u - y_c) / (x_u - x_c);
    bias6 = y_c - k6 * x_c;
//    Eigen::MatrixXf L6 = Eigen::MatrixXf::Zero(1, 3);
    Vector3f L6;
    L6 << k6, -1, bias6;

    // 求交点
    Vector3f poi1, poi2, poi3, poi4, poi5, poi6;
    poi1 = L1.cross(L4);
    // poi1 = poi1 / poi1(2);
//    poi2 = L1.cross(L6);  poi2 = poi2 / poi2(0, 2);
//    poi3 = L3.cross(L6);  poi3 = poi3 / poi3(0, 2);
//    poi4 = L2.cross(L3);  poi4 = poi4 / poi4(0, 2);
//    poi5 = L2.cross(L5);  poi5 = poi5 / poi5(0, 2);
//    poi6 = L4.cross(L5);  poi6 = poi6 / poi6(0, 2);
    poi2 = L1.cross(L6);  poi2 = poi2 / poi2(2);
    poi3 = L3.cross(L6);  poi3 = poi3 / poi3(2);
    poi4 = L2.cross(L3);  poi4 = poi4 / poi4(2);
    poi5 = L2.cross(L5);  poi5 = poi5 / poi5(2);
    poi6 = L4.cross(L5);  poi6 = poi6 / poi6(2);
//    cout << "poi1: " << poi1 << endl;
//    cout << "poi2: " << poi2 << endl;
//    cout << "poi3: " << poi3 << endl;
//    cout << "poi4: " << poi4 << endl;
//    cout << "poi5: " << poi5 << endl;
//    cout << "poi6: " << poi6 << endl;

    Vector3f diagonal1, diagonal2, diagonal3;
    diagonal1 = poi1.cross(poi4);
    diagonal2 = poi2.cross(poi5);
    diagonal3 = poi3.cross(poi6);

    Vector3f p_center_1, p_center_2, p_center_3;
    p_center_1 = diagonal1.cross(diagonal2);
    p_center_2 = diagonal1.cross(diagonal3);
    p_center_3 = diagonal2.cross(diagonal3);
    p_center_1 = p_center_1 / p_center_1(2);
    p_center_2 = p_center_2 / p_center_2(2);
    p_center_3 = p_center_3 / p_center_3(2);
//    cout << "p_center_1: " << p_center_1 << endl;
//    cout << "p_center_2: " << p_center_2 << endl;
//    cout << "p_center_3: " << p_center_3 << endl;

    Vector3f tmp = (p_center_1 + p_center_2 + p_center_3) / 3;
//    init_center_x = tmp(0, 0);
//    init_center_y = tmp(0, 1);

    init_center_x = tmp(0);
    init_center_y = tmp(1);
//    init_center_x = p_center_3(0);
//    init_center_y = p_center_3(1);


//    RowVector3f d1 = p_center_1 - tmp;
//    d1 = d1.array().square();
//    RowVector3f d2 = p_center_2 - tmp;
//    d2 = d2.array().square();
//    RowVector3f d3 = p_center_3 - tmp;
//    d3 = d3.array().square();
//    poi_var = (d1 + d2 + d3).sum();

    double dist1, dist2, dist3, dist4, dist5, dist6;
//    dist1 = (poi1(0, 0) - init_center_x) * (poi1(0, 0) - init_center_x) +
//            (poi1(0, 1) - init_center_y) * (poi1(0, 1) - init_center_y);
//    dist2 = (poi2(0, 0) - init_center_x) * (poi2(0, 0) - init_center_x) +
//            (poi2(0, 1) - init_center_y) * (poi2(0, 1) - init_center_y);
//    dist3 = (poi3(0, 0) - init_center_x) * (poi3(0, 0) - init_center_x) +
//            (poi3(0, 1) - init_center_y) * (poi3(0, 1) - init_center_y);
//    dist4 = (poi4(0, 0) - init_center_x) * (poi4(0, 0) - init_center_x) +
//            (poi4(0, 1) - init_center_y) * (poi4(0, 1) - init_center_y);
//    dist5 = (poi5(0, 0) - init_center_x) * (poi5(0, 0) - init_center_x) +
//            (poi5(0, 1) - init_center_y) * (poi5(0, 1) - init_center_y);
//    dist6 = (poi6(0, 0) - init_center_x) * (poi6(0, 0) - init_center_x) +
//            (poi6(0, 1) - init_center_y) * (poi6(0, 1) - init_center_y);
    dist1 = (poi1(0) - init_center_x) * (poi1(0) - init_center_x) +
            (poi1(1) - init_center_y) * (poi1(1) - init_center_y);
    dist2 = (poi2(0) - init_center_x) * (poi2(0) - init_center_x) +
            (poi2(1) - init_center_y) * (poi2(1) - init_center_y);
    dist3 = (poi3(0) - init_center_x) * (poi3(0) - init_center_x) +
            (poi3(1) - init_center_y) * (poi3(1) - init_center_y);
    dist4 = (poi4(0) - init_center_x) * (poi4(0) - init_center_x) +
            (poi4(1) - init_center_y) * (poi4(1) - init_center_y);
    dist5 = (poi5(0) - init_center_x) * (poi5(0) - init_center_x) +
            (poi5(1) - init_center_y) * (poi5(1) - init_center_y);
    dist6 = (poi6(0) - init_center_x) * (poi6(0) - init_center_x) +
            (poi6(1) - init_center_y) * (poi6(1) - init_center_y);
//    dist1 = sqrt(dist1);
//    dist2 = sqrt(dist2);
//    dist3 = sqrt(dist3);
//    dist4 = sqrt(dist4);
//    dist5 = sqrt(dist5);
//    dist6 = sqrt(dist6);

    vector<double> dist_list;
    dist_list.push_back(dist1);
    dist_list.push_back(dist2);
    dist_list.push_back(dist3);
    dist_list.push_back(dist4);
    dist_list.push_back(dist5);
    dist_list.push_back(dist6);
    vector<double>::iterator p = min_element(dist_list.begin(), dist_list.end());
//    vector<double>::iterator p = max_element(dist_list.begin(), dist_list.end());
    init_radius = (*p) - 10;
//    init_radius = (*p);
//    cout << "init_radius: " << init_radius << endl;

    // 存入vector中
    coor_c.clear();
    coor_c.push_back(coor_c1);
    coor_c.push_back(coor_c2);
    coor_c.push_back(coor_c3);
    params.clear();
    params.push_back(k1);
    params.push_back(k2);
    params.push_back(k3);
    params.push_back(k4);
    params.push_back(k5);
    params.push_back(k6);
    params.push_back(bias1);
    params.push_back(bias2);
    params.push_back(bias3);
    params.push_back(bias4);
    params.push_back(bias5);
    params.push_back(bias6);
    params.push_back(init_center_x);
    params.push_back(init_center_y);
    params.push_back(init_radius);

//    cout << "k1: " << k1 << endl;
//    cout << "k2: " << k2 << endl;
//    cout << "k3: " << k3 << endl;
//    cout << "k4: " << k4 << endl;
//    cout << "k5: " << k5 << endl;
//    cout << "k6: " << k6 << endl;
}

// input: X_arr, z_arr
// output: X_3D, Y_3D, Z_3D
void Finger_Reconstruct::convert_3d_coor(double n)
{
    vector<double> b_arr, c_arr, d_arr, e_arr, f_arr;
    b_arr = X_arr[0];
    c_arr = X_arr[1];
    d_arr = X_arr[2];
    e_arr = X_arr[3];
    f_arr = X_arr[4];

//    vector<vector<double>> X_3D;
//    vector<vector<double>> Y_3D;
//    vector<vector<double>> Z_3D;

    int arr_size = z_arr.size();
    double b, c, d, e, f, z;
    double alpha;  // 旋转角度
    double x0, y0;
    double aa, bb, ff, rx, ry;
    Eigen::Matrix2f R;
    double theta;
//    double n = 50;
    double step = (2*M_PI) / (n - 1);
    double ellipse_x, ellipse_y;
    vector<double> ellipse_x_list, ellipse_y_list, z_list;
    Eigen::Vector2f ellipse_xy;
    Eigen::Vector2f Rotation_ellipse;
    for(int i=0; i<arr_size; i++)
    {
        b = b_arr[i];
        c = c_arr[i];
        d = d_arr[i];
        e = e_arr[i];
        f = f_arr[i];
        z = z_arr[i];

        alpha = atan(2*c/(b-1)) / 2;
        if(alpha > pi/4)
        {
            alpha = pi/2 - alpha;
        }
        else if(alpha < -pi/4)
        {
            alpha = -pi/2 - alpha;
        }

        x0 = (b*d - c*e) / (-c*c + b);
        y0 = (e - c*d) / (-c*c + b);
        x0 = x0 / 2;  y0 = y0 / 2;

        aa = cos(alpha)*(cos(alpha) - c*sin(alpha)) - sin(alpha)*(c*cos(alpha) - b*sin(alpha));
        bb = cos(alpha)*(b*cos(alpha) + c*sin(alpha)) + sin(alpha)*(sin(alpha) + c*cos(alpha));
        ff = f + y0*(b*y0 - e + c*x0) + x0*(x0 - d + c*y0) - y0*(c*cos(alpha) - b*sin(alpha)) - x0*(cos(alpha) - c*sin(alpha)) - y0*(b*cos(alpha) + c*sin(alpha)) - x0*(sin(alpha) + c*cos(alpha));
        rx = sqrt(-ff/aa); // 长轴
        ry = sqrt(-ff/bb); // 短轴

        R << cos(alpha), sin(alpha),
                -sin(alpha), cos(alpha);

        for(theta=0; theta<=2*M_PI; theta+=step)
        {
            ellipse_x = -x0 + rx * cos(theta);
            ellipse_y = -y0 + ry * sin(theta);
            ellipse_xy << ellipse_x, ellipse_y;
            Rotation_ellipse = R * ellipse_xy;

            // 旋转后得到椭圆的实际坐标
            ellipse_x = Rotation_ellipse(0);
            ellipse_y = Rotation_ellipse(1);

            ellipse_x_list.push_back(ellipse_x);
            ellipse_y_list.push_back(ellipse_y);
            z_list.push_back(z);
        }

        X_3D.push_back(ellipse_x_list);
        Y_3D.push_back(ellipse_y_list);
        Z_3D.push_back(z_list);
        ellipse_x_list.clear();
        ellipse_y_list.clear();
        z_list.clear();
    }

}