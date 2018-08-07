#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <glog/logging.h>
#include <ctime>
#include <nlopt.h>
#include <unistd.h>

// 自定义函数
#include "preprocess/preprocess.h"
#include "utils/utils.h"
#include "reconstruction/finger_reconstruct.h"
#include "edge_detection/edge_detection.h"
#include "map_texture/map_texture.h"


void main_reconstruct()
{
//    test_interp();
    bool en_show = true;
    string dataset_path = "/home/xhb/xhb/DataSet/data/session1/src/";
    int sub = 203;
    int t = 13;
    string img_name = gen_img_name(sub, t);
    //cout << img_name << endl;
    clock_t start, finish;

    // 导入一组图片，对应sub-t.bmp
    start = clock();
    vector<cv::Mat> triple_imgs = load_triple_imgs(dataset_path, img_name);
    finish = clock();
    LOG(INFO) << "读入图片:" << (double)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms";
    //cout << triple_imgs.size() << endl;
    cv::Mat img_1 = triple_imgs[0];
    cv::Mat img_2 = triple_imgs[1];
    cv::Mat img_3 = triple_imgs[2];


    if(en_show)
    {
//        cv::imshow("img_1", img_1);
//        cv::imshow("img_2", img_2);
//        cv::imshow("img_3", img_3);
    }

    // 高斯滤波
    start = clock();
    cv::GaussianBlur(img_1, img_1, cv::Size(5, 5), 2, 2);
    cv::GaussianBlur(img_2, img_2, cv::Size(5, 5), 2, 2);
    cv::GaussianBlur(img_3, img_3, cv::Size(5, 5), 2, 2);
    finish = clock();
    LOG(INFO) << "高斯滤波:" << (double)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms";
    if(en_show)
    {
//        cv::imshow("img_1_blur", img_1);
//        cv::imshow("img_2_blur", img_2);
//        cv::imshow("img_3_blur", img_3);
    }

    // 设置相机默认参数
    start = clock();
    Finger_Reconstruct fc = Finger_Reconstruct();
    fc.set_cam_param();
//    fc.print_cam_param();
    finish = clock();
    LOG(INFO) << "设置相机参数:" << (double)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms";

    // 图像校正
    start = clock();
    img_1 = fc.img_rectify(img_1, fc.H1);
    img_2 = fc.img_rectify(img_2, fc.H2);
    img_3 = fc.img_rectify(img_3, fc.H3);
    finish = clock();
    LOG(INFO) << "图像校正:" << (double)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms";
    if(en_show)
    {
//        cv::imshow("img_1_rectified", img_1);
//        cv::imshow("img_2_rectified", img_2);
//        cv::imshow("img_3_rectified", img_3);
    }

    // 手指边缘提取
    start = clock();
    vector<vector<int>> edge_arr = find_all_edges(img_1, img_2, img_3);
    finish = clock();
    LOG(INFO) << "手指边缘提取" << (double)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms";

    // 显示提取到的手指边缘
//    disp_detected_edges(edge_arr, img_1, img_2, img_3);

//    cv::waitKey();

    // 灰度归一化
    start = clock();
    vector<cv::Mat> img_list = adjust_brightness(img_1, img_2, img_3, edge_arr, 128);
    img_1 = img_list[0];
    img_2 = img_list[1];
    img_3 = img_list[2];
    finish = clock();
    LOG(INFO) << "灰度归一化" << (double)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms";
    if(en_show)
    {
//        cv::imshow("img_1_adjust_brightness", img_1);
//        cv::imshow("img_2_adjust_brightness", img_2);
//        cv::imshow("img_3_adjust_brightness", img_3);
    }

    // 手指三维重建
    start = clock();
    fc.reconstruct(edge_arr);
    finish = clock();
    LOG(INFO) << "手指三维重建" << (double)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms";

//    vector<vector<double>> temp_X_arr = fc.X_arr;

    // 绘制三维手指模型
//    start = clock();
//    fc.convert_3d_coor(50);
//    coor_plot_3d::plot_3d_model(fc.X_3D, fc.Y_3D, fc.Z_3D);
//    finish = clock();
//    LOG(INFO) << "绘制三维指型" << (double)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms";

    // 纹理映射
//    fc.convert_3d_coor(400);
    start = clock();
    map_texture::set_params(fc.coor_c, 400);
    vector<MatrixXf> P_list(3);
    P_list[0] = fc.P1;
    P_list[1] = fc.P2;
    P_list[2] = fc.P3;
    vector<cv::Mat> img_list_src(3);
    img_list_src[0] = img_1;
    img_list_src[1] = img_2;
    img_list_src[2] = img_3;
    map_texture::map_texture(fc.X_arr, fc.z_arr, img_list_src, edge_arr, P_list);
    finish = clock();
    LOG(INFO) << "纹理映射" << (double)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms";

    // 绘制2维纹理图
//    cv::Mat texture = map_texture::texture_img;
//    cout << texture.size() << endl;
//    cv::imshow("map texture", texture);

    // 绘制3维指静脉模型
    vector<vector<unsigned char>> texture_temp = map_texture::texture_3D;
    coor_plot_3d::plot_3d_finger_vein_model(map_texture::x_3D, map_texture::y_3D, map_texture::z_3D, map_texture::texture_3D);


    cv::waitKey();

    return;
}

int main(int argc,char* argv[]) {
    // std::cout << "Hello, World!" << std::endl;

    // 初始化日志打印配置
    init_log(argv);

    // 初始化glut库
    glutInit(&argc, argv);

    main_reconstruct();

    return 0;
}