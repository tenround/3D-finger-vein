//
// Created by xhb on 18-8-6.
//

#include "map_texture.h"

namespace map_texture {
    vector<vector<uchar>> texture_3D;
    cv::Mat texture_img;
    vector<vector<double>> x_3D;
    vector<vector<double>> y_3D;
    vector<vector<double>> z_3D;

    Eigen::MatrixXf coor_c1, coor_c2, coor_c3;
    int n_ellipse;
}

void map_texture::set_params(vector<MatrixXf> coor, double n)
{
    map_texture::coor_c1 = coor[0];
    map_texture::coor_c2 = coor[1];
    map_texture::coor_c3 = coor[2];

    map_texture::n_ellipse = (int)n;
}

void map_texture::map_texture(vector<vector<double>> X_arr, vector<double> z_arr, vector<cv::Mat> img_list, vector<vector<int>> edge_arr, vector<MatrixXf> P_list)
{
//    cv::Mat img_1 = img_list[0];
//    cv::Mat img_2 = img_list[1];
//    cv::Mat img_3 = img_list[2];

    vector<double> b_arr = X_arr[0];
    vector<double> c_arr = X_arr[1];
    vector<double> d_arr = X_arr[2];
    vector<double> e_arr = X_arr[3];
    vector<double> f_arr = X_arr[4];

    // 将z_arr转换到三维坐标下
    for(int i=0;i<z_arr.size();i++)
    {
        double temp = (z_arr[i] - 320.0) / 720 * 68;
        z_arr[i] = temp;
    }


    vector<int> u_1 = edge_arr[0];
    vector<int> b_1 = edge_arr[1];
    vector<int> u_2 = edge_arr[2];
    vector<int> b_2 = edge_arr[3];
    vector<int> u_3 = edge_arr[4];
    vector<int> b_3 = edge_arr[5];

    // 手指轴向长度
    int n_len = z_arr.size();

    // 使用Mat来保存映射的纹理图像
    // cv::Mat vein_texture;
    texture_img.create(n_ellipse, n_len, CV_8UC1);

    // 遍历手指的每个截面，计算切点对应的index
    // 共有视野部分加权求和，非共有视野部分直接映射
    for(int i=0; i<n_len; i++)
    {
        double b = b_arr[i];
        double c = c_arr[i];
        double d = d_arr[i];
        double e = d_arr[i];
        double f = f_arr[i];

        // 6组切点坐标
        double x1, y1;
        double x2, y2;
        double x3, y3;
        double x4, y4;
        double x5, y5;
        double x6, y6;

        // 6跟切线的斜率
        double k1_temp;
        double k2_temp;
        double k3_temp;
        double k4_temp;
        double k5_temp;
        double k6_temp;

        // 求相机1的两个切点
        double xc = map_texture::coor_c1(0, 0);
        double yc = map_texture::coor_c1(0, 1);
        k1_temp = -(2*sqrt((4*f*c*c - 2*c*d*e + b*d*d + e*e - 4*b*f) * (xc*xc + 2*c*xc*yc + d*xc + b*yc*yc + e*yc + f)) \
               + 4*c*f - d*e - 2*e*xc + 4*c*c*xc*yc - 2*b*d*yc + 2*c*d*xc + 2*c*e*yc - 4*b*xc*yc) / \
               (- 4*c*c*xc*xc - 4*c*e*xc - e*e + 4*b*xc*xc + 4*b*d*xc + 4*b*f);
        x1 = -(d + e*k1_temp + 2*c*(yc - k1_temp*xc) + 2*b*k1_temp*(yc - k1_temp*xc))/(2*b*k1_temp*k1_temp + 4*c*k1_temp + 2);
        y1 = k1_temp * (x1 - xc) + yc;

        k2_temp = (2*sqrt((4*f*c*c - 2*c*d*e + b*d*d + e*e - 4*b*f)*(xc*xc + 2*c*xc*yc + d*xc + b*yc*yc + e*yc + f)) \
               - 4*c*f + d*e + 2*e*xc - 4*c*c*xc*yc + 2*b*d*yc - 2*c*d*xc - 2*c*e*yc + 4*b*xc*yc) /
                  (- 4*c*c*xc*xc - 4*c*e*xc - e*e + 4*b*xc*xc + 4*b*d*xc + 4*b*f);
        x2 = -(d + e*k2_temp + 2*c*(yc - k2_temp*xc) + 2*b*k2_temp*(yc - k2_temp*xc))/(2*b*k2_temp*k2_temp + 4*c*k2_temp + 2);
        y2 = k2_temp * (x2 - xc) + yc;

        // 求相机2的两个切点
        xc = map_texture::coor_c2(0, 0);
        yc = map_texture::coor_c2(0, 1);
        k3_temp =  -(2*sqrt((4*f*c*c - 2*c*d*e + b*d*d + e*e - 4*b*f)*(xc*xc + 2*c*xc*yc + d*xc + b*yc*yc + e*yc + f)) \
               + 4*c*f - d*e - 2*e*xc + 4*c*c*xc*yc - 2*b*d*yc + 2*c*d*xc + 2*c*e*yc - 4*b*xc*yc)/ \
               (- 4*c*c*xc*xc - 4*c*e*xc - e*e + 4*b*xc*xc + 4*b*d*xc + 4*b*f);
        x3 = -(d + e*k3_temp + 2*c*(yc - k3_temp*xc) + 2*b*k3_temp*(yc - k3_temp*xc))/(2*b*k3_temp*k3_temp + 4*c*k3_temp + 2);
        y3 = k3_temp * (x3 - xc) + yc;

        k4_temp = (2*sqrt((4*f*c*c - 2*c*d*e + b*d*d + e*e - 4*b*f)*(xc*xc + 2*c*xc*yc + d*xc + b*yc*yc + e*yc + f)) \
              - 4*c*f + d*e + 2*e*xc - 4*c*c*xc*yc + 2*b*d*yc - 2*c*d*xc - 2*c*e*yc + 4*b*xc*yc)/ \
              (- 4*c*c*xc*xc - 4*c*e*xc - e*e + 4*b*xc*xc + 4*b*d*xc + 4*b*f);
        x4 = -(d + e*k4_temp + 2*c*(yc - k4_temp*xc) + 2*b*k4_temp*(yc - k4_temp*xc))/(2*b*k4_temp*k4_temp + 4*c*k4_temp + 2);
        y4 = k4_temp * (x4 - xc) + yc;

        // 求相机3的两个切点
        xc = map_texture::coor_c3(0, 0);
        yc = map_texture::coor_c3(0, 1);
        k5_temp =  -(2*sqrt((4*f*c*c - 2*c*d*e + b*d*d + e*e - 4*b*f)*(xc*xc + 2*c*xc*yc + d*xc + b*yc*yc + e*yc + f)) \
               + 4*c*f - d*e - 2*e*xc + 4*c*c*xc*yc - 2*b*d*yc + 2*c*d*xc + 2*c*e*yc - 4*b*xc*yc)/ \
               (- 4*c*c*xc*xc - 4*c*e*xc - e*e + 4*b*xc*xc + 4*b*d*xc + 4*b*f);
        x5 = -(d + e*k5_temp + 2*c*(yc - k5_temp*xc) + 2*b*k5_temp*(yc - k5_temp*xc))/(2*b*k5_temp*k5_temp + 4*c*k5_temp + 2);
        y5 = k5_temp * (x5 - xc) + yc;

        k6_temp = (2*sqrt((4*f*c*c - 2*c*d*e + b*d*d + e*e - 4*b*f)*(xc*xc + 2*c*xc*yc + d*xc + b*yc*yc + e*yc + f)) \
              - 4*c*f + d*e + 2*e*xc - 4*c*c*xc*yc + 2*b*d*yc - 2*c*d*xc - 2*c*e*yc + 4*b*xc*yc)/ \
              (- 4*c*c*xc*xc - 4*c*e*xc - e*e + 4*b*xc*xc + 4*b*d*xc + 4*b*f);
        x6 = -(d + e*k6_temp + 2*c*(yc - k6_temp*xc) + 2*b*k6_temp*(yc - k6_temp*xc))/(2*b*k6_temp*k6_temp + 4*c*k6_temp + 2);
        y6 = k6_temp * (x6 - xc) + yc;

        // 计算旋转+平移变换参数T，转为标准型
        double x0, y0;
        x0 = (b*d - c*e) / (-c*c + b);
        y0 = (e - c*d) / (-c*c + b);
        x0 = x0 / 2; y0 = y0 / 2;
        double alpha = atan(2 * c / (b - 1)) / 2;
        if(alpha > M_PI / 4)
        {
            alpha = M_PI/2 - alpha;
        }
        else {
            if(alpha < -M_PI/4)
            {
                alpha = -M_PI / 2 - alpha;
            }
        }

        // 求出长短轴
        double aa, bb, ff, rx, ry;
        aa = cos(alpha)*(cos(alpha) - c*sin(alpha)) - sin(alpha)*(c*cos(alpha) - b*sin(alpha));
        bb = cos(alpha)*(b*cos(alpha) + c*sin(alpha)) + sin(alpha)*(sin(alpha) + c*cos(alpha));
        ff = f + y0*(b*y0 - e + c*x0) + x0*(x0 - d + c*y0) - y0*(c*cos(alpha) - b*sin(alpha)) - x0*(cos(alpha) - c*sin(alpha)) - y0*(b*cos(alpha) + c*sin(alpha)) - x0*(sin(alpha) + c*cos(alpha));
        rx = sqrt(-ff/aa); // 长轴
        ry = sqrt(-ff/bb); // 短轴

        Matrix3f T, inv_T;
        T << cos(alpha), sin(alpha), -x0,
            -sin(alpha), cos(alpha), -y0,
             0, 0, 1;
        inv_T = T.inverse();

        // 标准化椭圆上的切点
        Vector3f p_contact_1, p_contact_2, p_contact_3, p_contact_4, p_contact_5, p_contact_6;
        p_contact_1 << x1, y1, 1;
        p_contact_2 << x2, y2, 1;
        p_contact_3 << x3, y3, 1;
        p_contact_4 << x4, y4, 1;
        p_contact_5 << x5, y5, 1;
        p_contact_6 << x6, y6, 1;

        Vector3f norm_p_contact_1, norm_p_contact_2, norm_p_contact_3, norm_p_contact_4, norm_p_contact_5, norm_p_contact_6;
        norm_p_contact_1 = inv_T * p_contact_1;
        norm_p_contact_2 = inv_T * p_contact_2;
        norm_p_contact_3 = inv_T * p_contact_3;
        norm_p_contact_4 = inv_T * p_contact_4;
        norm_p_contact_5 = inv_T * p_contact_5;
        norm_p_contact_6 = inv_T * p_contact_6;

        double step = 2 * pi / (n_ellipse);

        // 计算切点对应的索引
        int edge_idx_1 = compute_edge_index(rx, ry, norm_p_contact_1, step);
        int edge_idx_2 = compute_edge_index(rx, ry, norm_p_contact_2, step);
        int edge_idx_3 = compute_edge_index(rx, ry, norm_p_contact_3, step);
        int edge_idx_4 = compute_edge_index(rx, ry, norm_p_contact_4, step);
        int edge_idx_5 = compute_edge_index(rx, ry, norm_p_contact_5, step);
        int edge_idx_6 = compute_edge_index(rx, ry, norm_p_contact_6, step);

        int BIAS = 12;
        edge_idx_2 = (edge_idx_2 + BIAS) % n_ellipse;
        edge_idx_1 = (edge_idx_1 - BIAS) % n_ellipse;
        if(edge_idx_1 < 0)
            edge_idx_1 = edge_idx_1 + n_ellipse;
        edge_idx_4 = (edge_idx_4 + BIAS) % n_ellipse;
        edge_idx_3 = (edge_idx_3 - BIAS) % n_ellipse;
        if(edge_idx_3 < 0)
            edge_idx_3 = edge_idx_3 + n_ellipse;
        edge_idx_6 = (edge_idx_6 + BIAS) % n_ellipse;
        edge_idx_5 = (edge_idx_5 - BIAS) % n_ellipse;
        if(edge_idx_5 < 0)
            edge_idx_5 = edge_idx_5 + n_ellipse;

        // 映射一个椭圆上的纹理
        vector<int> edge_idx_list(6);
        edge_idx_list[0] = edge_idx_1;
        edge_idx_list[1] = edge_idx_2;
        edge_idx_list[2] = edge_idx_3;
        edge_idx_list[3] = edge_idx_4;
        edge_idx_list[4] = edge_idx_5;
        edge_idx_list[5] = edge_idx_6;

        // [vein_texture] = one_ellipse_texture(edge_idx, img_1, img_2, img_3, edge_arr, step, rx, ry, T, z_arr, pts_ellipse)
//        [vein_texture(:, i)] = one_ellipse_texture(edge_idx, img_1, img_2, img_3, edge_arr, step, rx, ry, T, z_arr(i), pts_ellipse);
        vector<uchar> ellipse_texture = one_ellipse_texture(edge_idx_list, img_list, edge_arr, step, rx, ry, T, z_arr[i], P_list);

        texture_3D.push_back(ellipse_texture);
        for(int j=0; j<n_ellipse; j++)
        {
            texture_img.at<uchar>(j, i) = ellipse_texture[j];
        }

        double x_normal, y_normal;
        vector<double> real_x_list;
        vector<double> real_y_list;
        vector<double> real_z_list;
        for(double t=0; t<2*pi-step; t+=step)
        {
            x_normal = rx * cos(t);
            y_normal = ry * sin(t);
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;

            double real_x = coor_real_c(0);
            double real_y = coor_real_c(1);
            double real_z = coor_real_c(2);

            real_x_list.push_back(real_x);
            real_y_list.push_back(real_y);
            real_z_list.push_back(z_arr[i]);
        }

        x_3D.push_back(real_x_list);
        y_3D.push_back(real_y_list);
        z_3D.push_back(real_z_list);
    }



}

int map_texture::compute_edge_index(double rx, double ry, Vector3f norm_p_contact, double step)
{
    double cos_t = norm_p_contact(0) / rx;
//    double sin_t = norm_p_contack(1) / ry;

    double angle = atan( (rx / ry) * (norm_p_contact(1) / norm_p_contact(0)) );
    if(cos_t > 0)  // - pi/2 - pi/2
    {
        // 如果是第一象限，不做改变
        if(angle < 0)   // 第四象限
        {
            angle = 2 * pi + angle;
        }
    }
    else    // 第2\3象限
    {
        angle = pi + angle;
    }

    int edge_idx = (int)round(angle / step);

    return edge_idx;
}

vector<uchar> map_texture::one_ellipse_texture(vector<int> edge_idx_list, vector<cv::Mat> img_list, vector<vector<int>> edge_arr, double step,
        double rx, double ry, Matrix3f T, double z_arr, vector<MatrixXf> P_list)
{
    int edge_idx_1_1 = edge_idx_list[0];      // edge_idx{1, 1}
    int edge_idx_1_2 = edge_idx_list[1];      // edge_idx{1, 2}
    int edge_idx_2_1 = edge_idx_list[2];      // edge_idx{2, 1}
    int edge_idx_2_2 = edge_idx_list[3];      // edge_idx{2, 2}
    int edge_idx_3_1 = edge_idx_list[4];      // edge_idx{3, 1}
    int edge_idx_3_2 = edge_idx_list[5];      // edge_idx{3, 2}

    // 3 * 4
    MatrixXf P1 = P_list[0];
    MatrixXf P2 = P_list[1];
    MatrixXf P3 = P_list[2];

    cv::Mat img_1 = img_list[0];
    cv::Mat img_2 = img_list[1];
    cv::Mat img_3 = img_list[2];

    vector<int> u_1 = edge_arr[0];
    vector<int> b_1 = edge_arr[1];
    vector<int> u_2 = edge_arr[2];
    vector<int> b_2 = edge_arr[3];
    vector<int> u_3 = edge_arr[4];
    vector<int> b_3 = edge_arr[5];

    int min_y = 91;
    int max_y = 490;

    // -----------1号摄像头-----------
    vector<uchar> vein_texture_1(n_ellipse);
    vector<double> t_list;
    double x_normal, y_normal;
    int reflect_x, reflect_y;
    vector<int> reflect_x_list, reflect_y_list;
    double t;
    if(edge_idx_2_2 > edge_idx_3_1)
    {
        for(int i=edge_idx_2_2; i>=edge_idx_3_1; i--) {
            t = i * step;
            t_list.push_back(t);
            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;
            // P1 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P1 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_x_list.push_back(reflect_x);
            reflect_y_list.push_back(reflect_y);
        }
    }
    else
    {
        for(int i=edge_idx_2_2; i>=1; i--)
        {
            t = i * step;
            t_list.push_back(t);
            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;
            // P1 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P1 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_x_list.push_back(reflect_x);
            reflect_y_list.push_back(reflect_y);
        }
        for(int i=n_ellipse; i>=edge_idx_3_1; i--)
        {
            t = i * step;
            t_list.push_back(t);
            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;
            // P1 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P1 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_x_list.push_back(reflect_x);
            reflect_y_list.push_back(reflect_y);
        }

    }
    int n_pixels = t_list.size();

    for(int pt=0; pt<n_pixels; pt++)
    {
//        idx = rem(edge_idx{3,1} + pt, pts_ellipse) + 1;
        int idx = (edge_idx_3_1 + pt) % n_ellipse;
//        vein_texture1(idx) = uint8( img_1(reflect_img_x(n_pixs - pt), reflect_img_y(n_pixs - pt)) );
        uchar img_pixel = img_1.at<uchar>(reflect_x_list[n_pixels - pt - 1], reflect_y_list[n_pixels - pt - 1]);
        int row = reflect_x_list[n_pixels - pt - 1];
        int col = reflect_y_list[n_pixels - pt - 1];
        vein_texture_1[idx] = img_1.at<uchar>(reflect_x_list[n_pixels - pt - 1], reflect_y_list[n_pixels - pt - 1]);
    }



    // -----------2号摄像头-----------
    vector<uchar> vein_texture_2(n_ellipse);
    t_list.clear();
    reflect_x_list.clear();
    reflect_y_list.clear();
    if(edge_idx_3_2 > edge_idx_1_1)
    {
        for(int i=edge_idx_3_2; i>=edge_idx_1_1; i--) {
            t = i * step;
            t_list.push_back(t);
            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;
            // P2 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P2 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_x_list.push_back(reflect_x);
            reflect_y_list.push_back(reflect_y);
        }
    }
    else
    {
        for(int i=edge_idx_3_2; i>=1; i--)
        {
            t = i * step;
            t_list.push_back(t);
            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;
            // P2 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P2 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_x_list.push_back(reflect_x);
            reflect_y_list.push_back(reflect_y);
        }
        for(int i=n_ellipse; i>=edge_idx_1_1; i--)
        {
            t = i * step;
            t_list.push_back(t);
            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;
            // P2 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P2 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_x_list.push_back(reflect_x);
            reflect_y_list.push_back(reflect_y);
        }

    }
    n_pixels = t_list.size();

    for(int pt=0; pt<n_pixels; pt++)
    {
//        idx = rem(edge_idx{1,1} + pt, pts_ellipse) + 1;
        int idx = (edge_idx_1_1 + pt) % n_ellipse;
//        vein_texture2(idx) = uint8( img_2(reflect_img_x(n_pixs - pt), reflect_img_y(n_pixs - pt)) );
        vein_texture_2[idx] = img_2.at<uchar>(reflect_x_list[n_pixels - pt - 1], reflect_y_list[n_pixels - pt - 1]);
    }


    // -----------3号摄像头-----------
    vector<uchar> vein_texture_3(n_ellipse);
    t_list.clear();
    reflect_x_list.clear();
    reflect_y_list.clear();
    if(edge_idx_1_2 > edge_idx_2_1)
    {
        for(int i=edge_idx_1_2; i>=edge_idx_2_1; i--) {
            t = i * step;
            t_list.push_back(t);
            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;
            // P3 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P3 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_x_list.push_back(reflect_x);
            reflect_y_list.push_back(reflect_y);
        }
    }
    else
    {
        for(int i=edge_idx_1_2; i>=1; i--)
        {
            t = i * step;
            t_list.push_back(t);
            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;
            // P3 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P3 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_x_list.push_back(reflect_x);
            reflect_y_list.push_back(reflect_y);
        }
        for(int i=n_ellipse; i>=edge_idx_2_1; i--)
        {
            t = i * step;
            t_list.push_back(t);
            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;
            // P3 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P3 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_x_list.push_back(reflect_x);
            reflect_y_list.push_back(reflect_y);
        }

    }
    n_pixels = t_list.size();

    for(int pt=0; pt<n_pixels; pt++)
    {
//        idx = rem(edge_idx{2,1} + pt, pts_ellipse) + 1;
        int idx = (edge_idx_2_1 + pt) % n_ellipse;
//        vein_texture3(idx) = uint8( img_3(reflect_img_x(n_pixs - pt), reflect_img_y(n_pixs - pt)) );
        vein_texture_3[idx] = img_3.at<uchar>(reflect_x_list[n_pixels - pt - 1], reflect_y_list[n_pixels - pt - 1]);
    }


    // -----------1、2号摄像头-----------
    vector<uchar> vein_texture_1_2(n_ellipse);
    t_list.clear();
    vector<int> reflect_img1_x, reflect_img1_y;
    vector<int> reflect_img2_x, reflect_img2_y;
    if(edge_idx_1_1 > edge_idx_2_2)
    {
        for(int i=edge_idx_1_1-1; i>=edge_idx_2_2+1; i--) {
            t = i * step;
            t_list.push_back(t);

            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;

            // 相机1
            // P1 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P1 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img1_x.push_back(reflect_x);
            reflect_img1_y.push_back(reflect_y);

            // 相机2
            // P2 3*4矩阵
            reflect_x_y = P2 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img2_x.push_back(reflect_x);
            reflect_img2_y.push_back(reflect_y);
        }
    }
    else
    {
        for(int i=edge_idx_1_1-1; i>=1; i--)
        {
            t = i * step;
            t_list.push_back(t);

            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;

            // 相机1
            // P1 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P1 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img1_x.push_back(reflect_x);
            reflect_img1_y.push_back(reflect_y);

            // 相机2
            // P2 3*4矩阵
            reflect_x_y = P2 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img2_x.push_back(reflect_x);
            reflect_img2_y.push_back(reflect_y);
        }
        for(int i=n_ellipse; i>=edge_idx_2_2+1; i--)
        {
            t = i * step;
            t_list.push_back(t);

            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;

            // 相机1
            // P1 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P1 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img1_x.push_back(reflect_x);
            reflect_img1_y.push_back(reflect_y);

            // 相机2
            // P2 3*4矩阵
            reflect_x_y = P2 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img2_x.push_back(reflect_x);
            reflect_img2_y.push_back(reflect_y);
        }

    }
    n_pixels = t_list.size();
    vector<double> weight2;
    weight2.clear();
    vector<double> weight1;
    weight1.clear();
    for(int i=1; i<=n_pixels; i++)
    {
        double w2 = (double)1.0 / n_pixels * i - (double)1.0 / (2 * n_pixels);
        double w1 = 1 - w2;

        weight2.push_back(w2);
        weight1.push_back(w1);
    }

    for(int pt=0; pt<n_pixels; pt++)
    {
//        idx = rem((edge_idx{2,2} + 1) + pt, pts_ellipse) + 1;
        int idx = (edge_idx_2_2 + pt + 1) % n_ellipse;
        int img1_col = reflect_img1_y[n_pixels - pt - 1] - min_y + 1;
        int img2_col = reflect_img2_y[n_pixels - pt - 1] - min_y + 1;
        // 检查两个摄像头下的点有没有超出左右边界，不超则执行if里面程序
        if(img1_col > 0 && img1_col < u_1.size() && img2_col > 0 && img2_col < b_2.size())
        {
            // 检查有没超出上下边界，如果没有，执行if 里面的程序
            if(u_1[img1_col-1] <= reflect_img1_x[n_pixels - pt-1] && b_2[img2_col-1] >= reflect_img2_x[n_pixels - pt-1])
            {
//                vein_texture1_2(idx) = uint8( weight1(pt+1) * img_1(reflect_img1_x(n_pixs - pt), reflect_img1_y(n_pixs - pt)) +
//                weight2(pt+1) * img_2(reflect_img2_x(n_pixs - pt), reflect_img2_y(n_pixs - pt)))
                vein_texture_1_2[idx] = (uchar)round(weight1[pt] * img_1.at<uchar>(reflect_img1_x[n_pixels - pt - 1], reflect_img1_y[n_pixels - pt - 1]) +
                                                     weight2[pt] * img_2.at<uchar>(reflect_img2_x[n_pixels - pt - 1], reflect_img2_y[n_pixels - pt - 1]));
            }
            else if(u_1[img1_col-1] > reflect_img1_x[n_pixels - pt-1])
            {
//                p_st = [ux_1( img1_col ) + 1, reflect_img1_y(n_pixs - pt) - 1];
                vector<int> p_st(2);
                p_st[0] = u_1[img1_col-1] + 1;
                p_st[1] = reflect_img1_y[n_pixels - pt-1] - 1;
//                small_patch = img_1(p_st(1) : p_st(1) + 1, p_st(2) : p_st(2) + 1);  % 5*3大小，如果修改，注意后面也要改
//                        value1 = sum(sum(small_patch)) / 4;
                vector<uchar> small_patch;
                small_patch.push_back(img_1.at<uchar>(p_st[0], p_st[1]));
                small_patch.push_back(img_1.at<uchar>(p_st[0]+1, p_st[1]));
                small_patch.push_back(img_1.at<uchar>(p_st[0], p_st[1]+1));
                small_patch.push_back(img_1.at<uchar>(p_st[0]+1, p_st[1]+1));
                int sum = accumulate(small_patch.begin(), small_patch.end(), 0);
                int value1 = sum / 4;
//                value2 = img_2(reflect_img2_x(n_pixs - pt), reflect_img2_y(n_pixs - pt));
                int value2 = img_2.at<uchar>(reflect_img2_x[n_pixels - pt-1], reflect_img2_y[n_pixels - pt-1]);
//                vein_texture1_2(idx) = uint8( weight1(pt+1) * value1 + weight2(pt+1) * value2);
                vein_texture_1_2[idx] = (uchar)round(weight1[pt] * value1 + weight2[pt] * value2);
            }
            else if(b_2[img2_col-1] < reflect_img2_x[n_pixels - pt-1])
            {
//                value1 = img_1(reflect_img1_x(n_pixs - pt), reflect_img1_y(n_pixs - pt));
                int value1 = img_1.at<uchar>(reflect_img1_x[n_pixels - pt-1], reflect_img1_y[n_pixels - pt-1]);
//                p_st = [bx_2( img2_col) - 1, reflect_img2_y(n_pixs - pt) - 1];
                vector<int> p_st(2);
                p_st[0] = b_2[img2_col-1] - 1;
                p_st[1] = reflect_img2_y[n_pixels - pt-1] - 1;
//                small_patch = img_2(p_st(1) - 1 : p_st(1), p_st(2) : p_st(2) + 1);
                vector<uchar> small_patch;
                small_patch.push_back(img_2.at<uchar>(p_st[0]-1, p_st[1]));
                small_patch.push_back(img_2.at<uchar>(p_st[0], p_st[1]));
                small_patch.push_back(img_2.at<uchar>(p_st[0]-1, p_st[1]+1));
                small_patch.push_back(img_2.at<uchar>(p_st[0], p_st[1]+1));
//                value2 = sum(sum(small_patch)) / 4;
                int sum = accumulate(small_patch.begin(), small_patch.end(), 0);
                int value2 = sum / 4;
//                vein_texture1_2(idx) = uint8( weight1(pt+1) * value1 + weight2(pt+1) * value2);
                vein_texture_1_2[idx] = (uchar)round(weight1[pt] * value1 + weight2[pt] * value2);
            } else{  // 两个都超出了怎么处理
//                p_st = [ux_1( img1_col ) + 1, reflect_img1_y(n_pixs - pt) - 1];
                vector<int> p_st(2);
                p_st[0] = u_1[img1_col-1] + 1;
                p_st[1] = reflect_img1_y[n_pixels - pt-1] - 1;
//                small_patch = img_1(p_st(1) : p_st(1) + 1, p_st(2) : p_st(2) + 1);  % 5*3大小，如果修改，注意后面也要改
//                        value1 = sum(sum(small_patch)) / 4;
                vector<uchar> small_patch;
                small_patch.push_back(img_1.at<uchar>(p_st[0], p_st[1]));
                small_patch.push_back(img_1.at<uchar>(p_st[0]+1, p_st[1]));
                small_patch.push_back(img_1.at<uchar>(p_st[0], p_st[1]+1));
                small_patch.push_back(img_1.at<uchar>(p_st[0]+1, p_st[1]+1));
                int sum = accumulate(small_patch.begin(), small_patch.end(), 0);
                int value1 = sum / 4;

//                p_st = [bx_2( img2_col) - 1, reflect_img2_y(n_pixs - pt) - 1];
                p_st[0] = b_2[img2_col-1] - 1;
                p_st[1] = reflect_img2_y[n_pixels - pt-1] - 1;
//                small_patch = img_2(p_st(1) - 1 : p_st(1), p_st(2) : p_st(2) + 1);
                small_patch.clear();
                small_patch.push_back(img_2.at<uchar>(p_st[0]-1, p_st[1]));
                small_patch.push_back(img_2.at<uchar>(p_st[0], p_st[1]));
                small_patch.push_back(img_2.at<uchar>(p_st[0]-1, p_st[1]+1));
                small_patch.push_back(img_2.at<uchar>(p_st[0], p_st[1]+1));
//                value2 = sum(sum(small_patch)) / 4;
                sum = accumulate(small_patch.begin(), small_patch.end(), 0);
                int value2 = sum / 4;
                vein_texture_1_2[idx] = (uchar)round(weight1[pt] * value1 + weight2[pt] * value2);
            }
        } else{
//            vein_texture1_2(idx) = uint8( weight1(pt+1) * img_1(reflect_img1_x(n_pixs - pt), reflect_img1_y(n_pixs - pt)) + weight2(pt+1) * img_2(reflect_img2_x(n_pixs - pt), reflect_img2_y(n_pixs - pt)));
            int value1 = img_1.at<uchar>(reflect_img1_x[n_pixels - pt-1], reflect_img1_y[n_pixels - pt-1]);
            int value2 = img_2.at<uchar>(reflect_img2_x[n_pixels - pt-1], reflect_img2_y[n_pixels - pt-1]);
            vein_texture_1_2[idx] = (uchar)round(weight1[pt] * value1 + weight2[pt] * value2);
        }
    }

    // -----------2、3号摄像头-----------
    vector<uchar> vein_texture_2_3(n_ellipse);

    t_list.clear();
    vector<int> reflect_img3_x, reflect_img3_y;
//    vector<int> reflect_img2_x, reflect_img2_y;
    reflect_img2_x.clear();
    reflect_img2_y.clear();
    if(edge_idx_2_1 > edge_idx_3_2)
    {
        for(int i=edge_idx_2_1-1; i>=edge_idx_3_2+1; i--) {
            t = i * step;
            t_list.push_back(t);

            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;

            // 相机2
            // P2 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P2 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img2_x.push_back(reflect_x);
            reflect_img2_y.push_back(reflect_y);

            // 相机3
            // P3 3*4矩阵
            reflect_x_y = P3 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img3_x.push_back(reflect_x);
            reflect_img3_y.push_back(reflect_y);
        }
    }
    else
    {
        for(int i=edge_idx_2_1-1; i>=1; i--)
        {
            t = i * step;
            t_list.push_back(t);

            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;

            // 相机2
            // P2 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P2 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img2_x.push_back(reflect_x);
            reflect_img2_y.push_back(reflect_y);

            // 相机3
            // P3 3*4矩阵
            reflect_x_y = P3 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img3_x.push_back(reflect_x);
            reflect_img3_y.push_back(reflect_y);
        }
        for(int i=n_ellipse; i>=edge_idx_3_2+1; i--)
        {
            t = i * step;
            t_list.push_back(t);

            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;

            // 相机2
            // P2 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P2 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img2_x.push_back(reflect_x);
            reflect_img2_y.push_back(reflect_y);

            // 相机3
            // P3 3*4矩阵
            reflect_x_y = P3 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img3_x.push_back(reflect_x);
            reflect_img3_y.push_back(reflect_y);
        }

    }
    n_pixels = t_list.size();
//    vector<double> weight2;
    weight2.clear();
    vector<double> weight3;
    weight3.clear();
    for(int i=1; i<=n_pixels; i++)
    {
        double w3 = (double)1.0 / n_pixels * i - (double)1.0 / (2 * n_pixels);
        double w2 = 1 - w3;

        weight3.push_back(w3);
        weight2.push_back(w2);
    }

    for(int pt=0; pt<n_pixels; pt++)
    {
//        idx = rem(edge_idx{3,2} + 1 + pt, pts_ellipse) + 1;
        int idx = (edge_idx_3_2 + pt + 1) % n_ellipse;
        int img2_col = reflect_img2_y[n_pixels - pt - 1] - min_y + 1;
        int img3_col = reflect_img3_y[n_pixels - pt - 1] - min_y + 1;
        // 检查两个摄像头下的点有没有超出左右边界，不超则执行if里面程序
        if(img2_col > 0 && img2_col < u_2.size() && img3_col > 0 && img3_col < b_3.size())
        {
            // 检查有没超出上下边界，如果没有，执行if 里面的程序
            if(u_2[img2_col-1] <= reflect_img2_x[n_pixels - pt-1] && b_3[img3_col-1] >= reflect_img3_x[n_pixels - pt-1])
            {
//                vein_texture2_3(idx) = uint8( weight2(pt+1) * img_2(reflect_img2_x(n_pixs - pt), reflect_img2_y(n_pixs - pt)) +
//                weight3(pt+1) * img_3(reflect_img3_x(n_pixs - pt), reflect_img3_y(n_pixs - pt)));
                vein_texture_2_3[idx] = (uchar)round(weight2[pt] * img_2.at<uchar>(reflect_img2_x[n_pixels - pt - 1], reflect_img2_y[n_pixels - pt - 1]) +
                                                     weight3[pt] * img_3.at<uchar>(reflect_img3_x[n_pixels - pt - 1], reflect_img3_y[n_pixels - pt - 1]));
            }
            else if(u_2[img2_col-1] > reflect_img2_x[n_pixels - pt-1])
            {
//                p_st = [ux_2( img2_col ) + 1, reflect_img2_y(n_pixs - pt) - 1];
                vector<int> p_st(2);
                p_st[0] = u_2[img2_col-1] + 1;
                p_st[1] = reflect_img2_y[n_pixels - pt-1] - 1;
//                small_patch = img_2(p_st(1) : p_st(1) + 1, p_st(2) : p_st(2) + 1);  % 5*3大小，如果修改，注意后面也要改
//                        value2 = sum(sum(small_patch)) / 4;
                vector<uchar> small_patch;
                small_patch.push_back(img_2.at<uchar>(p_st[0], p_st[1]));
                small_patch.push_back(img_2.at<uchar>(p_st[0]+1, p_st[1]));
                small_patch.push_back(img_2.at<uchar>(p_st[0], p_st[1]+1));
                small_patch.push_back(img_2.at<uchar>(p_st[0]+1, p_st[1]+1));
                int sum = accumulate(small_patch.begin(), small_patch.end(), 0);
                int value2 = sum / 4;
//                value3 = img_3(reflect_img3_x(n_pixs - pt), reflect_img3_y(n_pixs - pt));
                int value3 = img_3.at<uchar>(reflect_img3_x[n_pixels - pt-1], reflect_img3_y[n_pixels - pt-1]);
//                vein_texture2_3(idx) = uint8( weight2(pt+1) * value2 + weight3(pt+1) * value3);
                vein_texture_2_3[idx] = (uchar)round(weight2[pt] * value2 + weight3[pt] * value3);
            }
            else if(b_3[img3_col-1] < reflect_img3_x[n_pixels - pt-1])
            {
//                value2 = img_2(reflect_img2_x(n_pixs - pt), reflect_img2_y(n_pixs - pt));
                int value2 = img_2.at<uchar>(reflect_img2_x[n_pixels - pt-1], reflect_img2_y[n_pixels - pt-1]);
//                p_st = [bx_3( img3_col) - 1, reflect_img3_y(n_pixs - pt) - 1];
                vector<int> p_st(2);
                p_st[0] = b_3[img3_col-1] - 1;
                p_st[1] = reflect_img3_y[n_pixels - pt-1] - 1;
//                small_patch = img_3(p_st(1) - 1 : p_st(1), p_st(2) : p_st(2) + 1);
                vector<uchar> small_patch;
                small_patch.push_back(img_3.at<uchar>(p_st[0]-1, p_st[1]));
                small_patch.push_back(img_3.at<uchar>(p_st[0], p_st[1]));
                small_patch.push_back(img_3.at<uchar>(p_st[0]-1, p_st[1]+1));
                small_patch.push_back(img_3.at<uchar>(p_st[0], p_st[1]+1));
//                value3 = sum(sum(small_patch)) / 4;
                int sum = accumulate(small_patch.begin(), small_patch.end(), 0);
                int value3 = sum / 4;
//                vein_texture2_3(idx) = uint8( weight2(pt+1) * value2 + weight3(pt+1) * value3);
                vein_texture_2_3[idx] = (uchar)round(weight2[pt] * value2 + weight3[pt] * value3);
            } else{  // 两个都超出了怎么处理
//                p_st = [ux_2( img2_col ) + 1, reflect_img2_y(n_pixs - pt) - 1];
                vector<int> p_st(2);
                p_st[0] = u_2[img2_col-1] + 1;
                p_st[1] = reflect_img2_y[n_pixels - pt-1] - 1;
//                small_patch = img_2(p_st(1) : p_st(1) + 1, p_st(2) : p_st(2) + 1);  % 5*3大小，如果修改，注意后面也要改
//                        value2 = sum(sum(small_patch)) / 4;
                vector<uchar> small_patch;
                small_patch.push_back(img_2.at<uchar>(p_st[0], p_st[1]));
                small_patch.push_back(img_2.at<uchar>(p_st[0]+1, p_st[1]));
                small_patch.push_back(img_2.at<uchar>(p_st[0], p_st[1]+1));
                small_patch.push_back(img_2.at<uchar>(p_st[0]+1, p_st[1]+1));
                int sum = accumulate(small_patch.begin(), small_patch.end(), 0);
                int value2 = sum / 4;

//                p_st = [bx_3( img3_col) - 1, reflect_img3_y(n_pixs - pt) - 1];
                p_st[0] = b_3[img3_col-1] - 1;
                p_st[1] = reflect_img3_y[n_pixels - pt-1] - 1;
//                small_patch = img_3(p_st(1) - 1 : p_st(1), p_st(2) : p_st(2) + 1);
                small_patch.clear();
                small_patch.push_back(img_3.at<uchar>(p_st[0]-1, p_st[1]));
                small_patch.push_back(img_3.at<uchar>(p_st[0], p_st[1]));
                small_patch.push_back(img_3.at<uchar>(p_st[0]-1, p_st[1]+1));
                small_patch.push_back(img_3.at<uchar>(p_st[0], p_st[1]+1));
//                value3 = sum(sum(small_patch)) / 4;
                sum = accumulate(small_patch.begin(), small_patch.end(), 0);
                int value3 = sum / 4;
                vein_texture_2_3[idx] = (uchar)round(weight2[pt] * value2 + weight3[pt] * value3);
            }
        } else{
//            vein_texture2_3(idx) = uint8( weight2(pt+1) * img_2(reflect_img2_x(n_pixs - pt), reflect_img2_y(n_pixs - pt)) + weight3(pt+1) * img_3(reflect_img3_x(n_pixs - pt), reflect_img3_y(n_pixs - pt)));
            int value2 = img_2.at<uchar>(reflect_img2_x[n_pixels - pt-1], reflect_img2_y[n_pixels - pt-1]);
            int value3 = img_3.at<uchar>(reflect_img3_x[n_pixels - pt-1], reflect_img3_y[n_pixels - pt-1]);
            vein_texture_2_3[idx] = (uchar)round(weight2[pt] * value2 + weight3[pt] * value3);
        }
    }


    // -----------1、3号摄像头-----------
    vector<uchar> vein_texture_1_3(n_ellipse);

    t_list.clear();
//    vector<int> reflect_img3_x, reflect_img3_y;
//    vector<int> reflect_img2_x, reflect_img2_y;
    reflect_img1_x.clear();
    reflect_img1_y.clear();
    reflect_img3_x.clear();
    reflect_img3_y.clear();
    if(edge_idx_3_1 > edge_idx_1_2)
    {
        for(int i=edge_idx_3_1-1; i>=edge_idx_1_2+1; i--) {
            t = i * step;
            t_list.push_back(t);

            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;

            // 相机1
            // P1 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P1 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img1_x.push_back(reflect_x);
            reflect_img1_y.push_back(reflect_y);

            // 相机3
            // P3 3*4矩阵
            reflect_x_y = P3 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img3_x.push_back(reflect_x);
            reflect_img3_y.push_back(reflect_y);
        }
    }
    else
    {
        for(int i=edge_idx_3_1-1; i>=1; i--)
        {
            t = i * step;
            t_list.push_back(t);

            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;

            // 相机1
            // P1 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P1 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img1_x.push_back(reflect_x);
            reflect_img1_y.push_back(reflect_y);

            // 相机3
            // P3 3*4矩阵
            reflect_x_y = P3 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img3_x.push_back(reflect_x);
            reflect_img3_y.push_back(reflect_y);
        }
        for(int i=n_ellipse; i>=edge_idx_1_2+1; i--)
        {
            t = i * step;
            t_list.push_back(t);

            x_normal = rx * cos(t);
            y_normal = ry * sin(t);

            // 计算映射到椭圆上的xy坐标索引
            Vector3f x_y_normal;
            x_y_normal << x_normal, y_normal, 1;
            Vector3f coor_real_c;
            coor_real_c = T * x_y_normal;
            Vector4f reflect_matrix;
            reflect_matrix << coor_real_c(0), coor_real_c(1), z_arr, 1;

            // 相机1
            // P1 3*4矩阵
            Vector3f reflect_x_y;
            reflect_x_y = P1 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img1_x.push_back(reflect_x);
            reflect_img1_y.push_back(reflect_y);

            // 相机3
            // P3 3*4矩阵
            reflect_x_y = P3 * reflect_matrix;
            reflect_x = (int)round(reflect_x_y(1) / reflect_x_y(2));
            reflect_y = (int)round(reflect_x_y(0) / reflect_x_y(2));
            reflect_img3_x.push_back(reflect_x);
            reflect_img3_y.push_back(reflect_y);
        }

    }
    n_pixels = t_list.size();
//    vector<double> weight1;
    weight1.clear();
//    vector<double> weight3;
    weight3.clear();
    for(int i=1; i<=n_pixels; i++)
    {
        double w1 = (double)1.0 / n_pixels * i - (double)1.0 / (2 * n_pixels);
        double w3 = 1 - w1;

        weight1.push_back(w1);
        weight3.push_back(w3);
    }

    for(int pt=0; pt<n_pixels; pt++)
    {
//        idx = rem(edge_idx{1,2} + 1 + pt, pts_ellipse) + 1;
        int idx = (edge_idx_1_2 + pt + 1) % n_ellipse;
        int img3_col = reflect_img3_y[n_pixels - pt - 1] - min_y + 1;
        int img1_col = reflect_img1_y[n_pixels - pt - 1] - min_y + 1;
        // 检查两个摄像头下的点有没有超出左右边界，不超则执行if里面程序
        if(img3_col > 0 && img3_col < u_3.size() && img1_col > 0 && img1_col < b_1.size())
        {
            // 检查有没超出上下边界，如果没有，执行if 里面的程序
            if(u_3[img3_col-1] <= reflect_img3_x[n_pixels - pt-1] && b_1[img1_col-1] >= reflect_img1_x[n_pixels - pt-1])
            {
//                vein_texture1_3(idx) = uint8( weight3(pt+1) * img_3(reflect_img3_x(n_pixs - pt), reflect_img3_y(n_pixs - pt)) +
//                  weight1(pt+1) * img_1(reflect_img1_x(n_pixs - pt), reflect_img1_y(n_pixs - pt)));
                vein_texture_1_3[idx] = (uchar)round(weight3[pt] * img_3.at<uchar>(reflect_img3_x[n_pixels - pt - 1], reflect_img3_y[n_pixels - pt - 1]) +
                                                     weight1[pt] * img_1.at<uchar>(reflect_img1_x[n_pixels - pt - 1], reflect_img1_y[n_pixels - pt - 1]));
            }
            else if(u_3[img3_col-1] > reflect_img3_x[n_pixels - pt-1])
            {
//                p_st = [ux_3( img3_col ) + 1, reflect_img3_y(n_pixs - pt) - 1];
                vector<int> p_st(2);
                p_st[0] = u_3[img3_col-1] + 1;
                p_st[1] = reflect_img3_y[n_pixels - pt-1] - 1;
//                small_patch = img_3(p_st(1) : p_st(1) + 1, p_st(2) : p_st(2) + 1);  % 5*3大小，如果修改，注意后面也要改
//                        value3 = sum(sum(small_patch)) / 4;
                vector<uchar> small_patch;
                small_patch.push_back(img_3.at<uchar>(p_st[0], p_st[1]));
                small_patch.push_back(img_3.at<uchar>(p_st[0]+1, p_st[1]));
                small_patch.push_back(img_3.at<uchar>(p_st[0], p_st[1]+1));
                small_patch.push_back(img_3.at<uchar>(p_st[0]+1, p_st[1]+1));
                int sum = accumulate(small_patch.begin(), small_patch.end(), 0);
                int value3 = sum / 4;
//                value1 = img_1(reflect_img1_x(n_pixs - pt), reflect_img1_y(n_pixs - pt));
                int value1 = img_1.at<uchar>(reflect_img1_x[n_pixels - pt-1], reflect_img1_y[n_pixels - pt-1]);
//                vein_texture1_3(idx) = uint8( weight3(pt+1) * value3 + weight1(pt+1) * value1);
                vein_texture_1_3[idx] = (uchar)round(weight3[pt] * value3 + weight1[pt] * value1);
            }
            else if(b_1[img1_col-1] < reflect_img1_x[n_pixels - pt-1])
            {
//                value3 = img_3(reflect_img3_x(n_pixs - pt), reflect_img3_y(n_pixs - pt));
                int value3 = img_3.at<uchar>(reflect_img3_x[n_pixels - pt-1], reflect_img3_y[n_pixels - pt-1]);
//                p_st = [bx_1( img1_col) - 1, reflect_img1_y(n_pixs - pt) - 1];
                vector<int> p_st(2);
                p_st[0] = b_1[img1_col-1] - 1;
                p_st[1] = reflect_img1_y[n_pixels - pt-1] - 1;
//                small_patch = img_1(p_st(1) - 1 : p_st(1), p_st(2) : p_st(2) + 1);
                vector<uchar> small_patch;
                small_patch.push_back(img_1.at<uchar>(p_st[0]-1, p_st[1]));
                small_patch.push_back(img_1.at<uchar>(p_st[0], p_st[1]));
                small_patch.push_back(img_1.at<uchar>(p_st[0]-1, p_st[1]+1));
                small_patch.push_back(img_1.at<uchar>(p_st[0], p_st[1]+1));
//                value1 = sum(sum(small_patch)) / 4;
                int sum = accumulate(small_patch.begin(), small_patch.end(), 0);
                int value1 = sum / 4;
//                vein_texture1_3(idx) = uint8( weight3(pt+1) * value3 + weight1(pt+1) * value1);
                vein_texture_1_3[idx] = (uchar)round(weight3[pt] * value3 + weight1[pt] * value1);
            } else{  // 两个都超出了怎么处理
//                p_st = [ux_3( img3_col ) + 1, reflect_img3_y(n_pixs - pt) - 1];
                vector<int> p_st(2);
                p_st[0] = u_3[img3_col-1] + 1;
                p_st[1] = reflect_img3_y[n_pixels - pt-1] - 1;
//                small_patch = img_3(p_st(1) : p_st(1) + 1, p_st(2) : p_st(2) + 1);  % 5*3大小，如果修改，注意后面也要改
//                        value3 = sum(sum(small_patch)) / 4;
                vector<uchar> small_patch;
                small_patch.push_back(img_3.at<uchar>(p_st[0], p_st[1]));
                small_patch.push_back(img_3.at<uchar>(p_st[0]+1, p_st[1]));
                small_patch.push_back(img_3.at<uchar>(p_st[0], p_st[1]+1));
                small_patch.push_back(img_3.at<uchar>(p_st[0]+1, p_st[1]+1));
                int sum = accumulate(small_patch.begin(), small_patch.end(), 0);
                int value3 = sum / 4;

//                p_st = [bx_1( img1_col) - 1, reflect_img1_y(n_pixs - pt) - 1];
                p_st[0] = b_1[img1_col-1] - 1;
                p_st[1] = reflect_img1_y[n_pixels - pt-1] - 1;
//                small_patch = img_1(p_st(1) - 1 : p_st(1), p_st(2) : p_st(2) + 1);
                small_patch.clear();
                small_patch.push_back(img_1.at<uchar>(p_st[0]-1, p_st[1]));
                small_patch.push_back(img_1.at<uchar>(p_st[0], p_st[1]));
                small_patch.push_back(img_1.at<uchar>(p_st[0]-1, p_st[1]+1));
                small_patch.push_back(img_1.at<uchar>(p_st[0], p_st[1]+1));
//                value1 = sum(sum(small_patch)) / 4;
                sum = accumulate(small_patch.begin(), small_patch.end(), 0);
                int value1 = sum / 4;
                vein_texture_1_3[idx] = (uchar)round(weight3[pt] * value3 + weight1[pt] * value1);
            }
        } else{
//            vein_texture1_3(idx) = uint8( weight3(pt+1) * img_3(reflect_img3_x(n_pixs - pt), reflect_img3_y(n_pixs - pt)) + weight1(pt+1) * img_1(reflect_img1_x(n_pixs - pt), reflect_img1_y(n_pixs - pt)));
            int value3 = img_3.at<uchar>(reflect_img3_x[n_pixels - pt-1], reflect_img3_y[n_pixels - pt-1]);
            int value1 = img_1.at<uchar>(reflect_img1_x[n_pixels - pt-1], reflect_img1_y[n_pixels - pt-1]);
            vein_texture_2_3[idx] = (uchar)round(weight3[pt] * value3 + weight1[pt] * value1);
        }
    }


    // vein_texture = vein_texture1 + vein_texture2 + vein_texture3 + vein_texture1_2 + vein_texture2_3 + vein_texture1_3;
    vector<uchar> vein_texture(n_ellipse);
    for(int i=0; i<n_ellipse; i++)
    {
        vein_texture[i] = vein_texture_1[i] + vein_texture_2[i] + vein_texture_3[i] + vein_texture_1_2[i] + vein_texture_2_3[i] + vein_texture_1_3[i];
//        vein_texture[i] = vein_texture_1[i];
    }

    return vein_texture;
}

