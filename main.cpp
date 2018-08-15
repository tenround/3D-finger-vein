#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <glog/logging.h>
#include <ctime>
#include <nlopt.h>
#include <unistd.h>
#include <sys/time.h>

// 自定义函数
#include "preprocess/preprocess.h"
#include "utils/utils.h"
#include "reconstruction/finger_reconstruct.h"
#include "edge_detection/edge_detection.h"
#include "map_texture/map_texture.h"

// 错误解
//if((sub == 44 && t == 14) || (sub == 60 && t == 14) || (sub == 153 && t == 4)
//|| (sub == 153 && t == 6) || (sub == 153 && t == 7) || (sub == 153 && t == 8)
//|| (sub == 153 && t == 9) || (sub == 153 && t == 11) || (sub == 153 && t == 12)
//|| (sub == 164 && t == 11) || (sub == 204 && t == 8) || (sub == 16 && t == 8)
//|| (sub == 155 && t == 7))
//continue;

void main_reconstruct()
{
//    test_interp();
    bool en_show = true;
    string dataset_path = "/home/xhb/xhb/DataSet/data/session1/src/";
    int sub = 1;
    int t = 10;
    string img_name = gen_img_name(sub, t);
    //cout << img_name << endl;
    clock_t start, finish;
    long t1, t2;

    // 导入一组图片，对应sub-t.bmp
    start = clock();
    t1 = getCurrentTime();
    vector<cv::Mat> triple_imgs = load_triple_imgs(dataset_path, img_name);
    finish = clock();
    t2 = getCurrentTime();
    LOG(INFO) << "读入图片:" << (float)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms" << "/" << (t2 - t1) << "ms";
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
    t1 = getCurrentTime();
    cv::GaussianBlur(img_1, img_1, cv::Size(5, 5), 2, 2);
    cv::GaussianBlur(img_2, img_2, cv::Size(5, 5), 2, 2);
    cv::GaussianBlur(img_3, img_3, cv::Size(5, 5), 2, 2);
    finish = clock();
    t2 = getCurrentTime();
    LOG(INFO) << "高斯滤波:" << (float)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms" << "/" << (t2 - t1) << "ms";
    if(en_show)
    {
//        cv::imshow("img_1_blur", img_1);
//        cv::imshow("img_2_blur", img_2);
//        cv::imshow("img_3_blur", img_3);
    }

    // 设置相机默认参数
    start = clock();
    t1 = getCurrentTime();
    Finger_Reconstruct fc = Finger_Reconstruct();
    fc.set_cam_param();
//    fc.print_cam_param();
    finish = clock();
    t2 = getCurrentTime();
    LOG(INFO) << "设置相机参数:" << (float)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms" << "/" << (t2 - t1) << "ms";

    // 图像校正
    start = clock();
    t1 = getCurrentTime();
    img_1 = fc.img_rectify(img_1, fc.H1);
    img_2 = fc.img_rectify(img_2, fc.H2);
    img_3 = fc.img_rectify(img_3, fc.H3);
    finish = clock();
    t2 = getCurrentTime();
    LOG(INFO) << "图像校正:" << (float)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms" << "/" << (t2 - t1) << "ms";
    if(en_show)
    {
//        cv::imshow("img_1_rectified", img_1);
//        cv::imshow("img_2_rectified", img_2);
//        cv::imshow("img_3_rectified", img_3);
    }

    // 图像增强（在对手指作边缘检测前，先进行图像增强，加强边缘，但同时也会降低图像静脉纹理的质量，故仅在边缘检测时使用）
    cv::Mat img_edge_1, img_edge_2, img_edge_3;
    cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, -1, 0, 0, 7, 0, 0, -1, 0);
    cv::filter2D(img_1, img_edge_1, CV_8UC1, kernel);
    cv::filter2D(img_2, img_edge_2, CV_8UC1, kernel);
    cv::filter2D(img_3, img_edge_3, CV_8UC1, kernel);
    if(en_show)
    {
//        cv::imshow("img_edge_1", img_edge_1);
//        cv::imshow("img_edge_2", img_edge_2);
//        cv::imshow("img_edge_3", img_edge_3);
    }

    // 手指边缘提取
    start = clock();
    t1 = getCurrentTime();
    vector<vector<int>> edge_arr = find_all_edges(img_1, img_2, img_3);
//    vector<vector<int>> edge_arr = find_all_edges(img_edge_1, img_edge_2, img_edge_3);
    finish = clock();
    t2 = getCurrentTime();
    LOG(INFO) << "手指边缘提取" << (float)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms" << "/" << (t2 - t1) << "ms";

    // 显示提取到的手指边缘
//    disp_detected_edges(edge_arr, img_1, img_2, img_3);
    cv::waitKey();

    // 灰度归一化
    start = clock();
    t1 = getCurrentTime();
    vector<cv::Mat> img_list = adjust_brightness(img_1, img_2, img_3, edge_arr, 128);
    img_1 = img_list[0];
    img_2 = img_list[1];
    img_3 = img_list[2];
    finish = clock();
    t2 = getCurrentTime();
    LOG(INFO) << "灰度归一化:" << (float)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms" << "/" << (t2 - t1) << "ms";
    if(en_show)
    {
//        cv::imshow("img_1_adjust_brightness", img_1);
//        cv::imshow("img_2_adjust_brightness", img_2);
//        cv::imshow("img_3_adjust_brightness", img_3);
    }

    // 手指三维重建
    start = clock();
    t1 = getCurrentTime();
    fc.reconstruct(edge_arr, 400);
    finish = clock();
    t2 = getCurrentTime();
    LOG(INFO) << "手指三维重建:" << (float)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms" << "/" << (t2 - t1) << "ms";

    // 重建出799个截面
//    cout << fc.X_arr.size() << endl;
//    cout << fc.z_arr.size() << endl;

//    vector<vector<float>> temp_X_arr = fc.X_arr;

    // 绘制三维手指模型
//    fc.convert_3d_coor(50);
//    coor_plot_3d::plot_3d_model(fc.X_3D, fc.Y_3D, fc.Z_3D);

    // 纹理映射
//    fc.convert_3d_coor(400);
    start = clock();
    t1 = getCurrentTime();
    map_texture::set_params(fc.coor_c, 400);
    vector<MatrixXf> P_list(3);
    P_list[0] = fc.P1;
    P_list[1] = fc.P2;
    P_list[2] = fc.P3;
//    vector<cv::Mat> img_list_src(3);
//    img_list_src[0] = img_1;
//    img_list_src[1] = img_2;
//    img_list_src[2] = img_3;
    map_texture::map_texture(fc.X_arr, fc.z_arr, img_list, edge_arr, P_list);
    finish = clock();
    t2 = getCurrentTime();
    LOG(INFO) << "纹理映射:" << (float)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms" << "/" << (t2 - t1) << "ms";

    // 绘制2维纹理图
//    cv::Mat texture = map_texture::texture_img;
//    cout << texture.size() << endl;
//    cout << map_texture::x_3D.size() << ", " << map_texture::x_3D[0].size() << endl;
//    cv::imshow("map texture", texture);
//    cv::waitKey();

    // 绘制3维指静脉模型
//    vector<vector<unsigned char>> texture_temp = map_texture::texture_3D;
//    coor_plot_3d::plot_3d_finger_vein_model(map_texture::x_3D, map_texture::y_3D, map_texture::z_3D, map_texture::texture_3D);

    start = clock();
    t1 = getCurrentTime();
//    map_texture::projection(180, 180);// 2度采样一个点
    map_texture::projection_2(180, 180);// 2度采样一个点
    finish = clock();
    t2 = getCurrentTime();
    LOG(INFO) << "生成纹理图/深度图:" << (float)(finish - start) / CLOCKS_PER_SEC * 1000 << "ms" << "/" << (t2 - t1) << "ms";

    cv::Mat texture_prj = map_texture::texture_prj;
    cv::Mat depth_prj_img = map_texture::depth_prj_img;
    depth_prj_img.convertTo(depth_prj_img, CV_8UC1, 255.0);
    cv::normalize(depth_prj_img, depth_prj_img, 0, 255, cv::NORM_MINMAX);
//    cout << texture_prj.size() << endl;
    cout << "texture_prj: " << texture_prj.size() << endl;
    cout << "depth_prj_img: " << depth_prj_img.size() << endl;

//    cv::resize(texture_prj, texture_prj, cv::Size(360, 360));
//    cv::resize(depth_prj_img, depth_prj_img, cv::Size(360, 360));
    cv::imshow("texture", texture_prj);
    cv::imshow("depth", depth_prj_img);

    cv::waitKey();

    return;
}

void generate_one_map()
{
    string dataset_path = "/home/xhb/xhb/DataSet/data/session1/src/";

    int sub = 54;
    int t = 13;
    string img_name = gen_img_name(sub, t);

    cout << img_name << endl;

    // 导入一组图片，对应sub-t.bmp
    vector<cv::Mat> triple_imgs = load_triple_imgs(dataset_path, img_name);
    cv::Mat img_1 = triple_imgs[0];
    cv::Mat img_2 = triple_imgs[1];
    cv::Mat img_3 = triple_imgs[2];

    // 高斯滤波
    cv::GaussianBlur(img_1, img_1, cv::Size(5, 5), 2, 2);
    cv::GaussianBlur(img_2, img_2, cv::Size(5, 5), 2, 2);
    cv::GaussianBlur(img_3, img_3, cv::Size(5, 5), 2, 2);

    // 设置相机默认参数
    Finger_Reconstruct fc = Finger_Reconstruct();
    fc.set_cam_param();

    // 图像校正
    img_1 = fc.img_rectify(img_1, fc.H1);
    img_2 = fc.img_rectify(img_2, fc.H2);
    img_3 = fc.img_rectify(img_3, fc.H3);

    // 手指边缘提取
    vector<vector<int>> edge_arr = find_all_edges(img_1, img_2, img_3);

    // 灰度归一化
    vector<cv::Mat> img_list = adjust_brightness(img_1, img_2, img_3, edge_arr, 128);
    img_1 = img_list[0];
    img_2 = img_list[1];
    img_3 = img_list[2];

    // 手指三维重建
    fc.reconstruct(edge_arr, 460);

    // 纹理映射
    map_texture::set_params(fc.coor_c, 520);
    vector<MatrixXf> P_list(3);
    P_list[0] = fc.P1;
    P_list[1] = fc.P2;
    P_list[2] = fc.P3;
    map_texture::map_texture(fc.X_arr, fc.z_arr, img_list, edge_arr, P_list);

    // 生成纹理图/深度图
    map_texture::projection(180, 180);// 2度采样一个点

    cv::Mat texture_prj = map_texture::texture_prj;
    cv::Mat depth_prj_img = map_texture::depth_prj_img;
    cv::resize(texture_prj, texture_prj, cv::Size(360, 360));
    cv::resize(depth_prj_img, depth_prj_img, cv::Size(360, 360));
    depth_prj_img.convertTo(depth_prj_img, CV_8UC1, 255.0);
    cv::normalize(depth_prj_img, depth_prj_img, 0, 255, cv::NORM_MINMAX);
//    cv::imshow("texture", texture_prj);
//    cv::imshow("depth", depth_prj_img);

    cv::imwrite(strcat("./fv_img/", img_name), texture_prj );
    cv::imwrite(strcat("./fv_dep/", img_name), depth_prj_img );
    cout << "生成纹理图:" << strcat("./fv_img/", img_name) << endl;
    cout << "生成深度图:" << strcat("./fv_dep/", img_name) << endl;

    cv::waitKey();

    return;
}

void generate_maps()
{
    string dataset_path = "/home/xhb/xhb/DataSet/data/session2/src/";
    for(int sub=1; sub<=203; sub++)
    {
        for(int t=1; t<=14; t++)
        {
            // 错误解
//            if((sub == 60 && t == 6) || (sub == 123 && t == 1) || (sub == 203 && t == 5))
//                continue;

//            if((sub == 33 && t == 11) || (sub == 139 && t == 2) || (sub == 187 && t == 2))
//                continue;

//            if((sub == 153 && t == 9))
//                continue;

            string img_name = gen_img_name(sub, t);

            cout << img_name << endl;

            // 导入一组图片，对应sub-t.bmp
            vector<cv::Mat> triple_imgs = load_triple_imgs(dataset_path, img_name);
            cv::Mat img_1 = triple_imgs[0];
            cv::Mat img_2 = triple_imgs[1];
            cv::Mat img_3 = triple_imgs[2];


            // 高斯滤波
            cv::GaussianBlur(img_1, img_1, cv::Size(5, 5), 2, 2);
            cv::GaussianBlur(img_2, img_2, cv::Size(5, 5), 2, 2);
            cv::GaussianBlur(img_3, img_3, cv::Size(5, 5), 2, 2);

            // 设置相机默认参数
            Finger_Reconstruct fc = Finger_Reconstruct();
            fc.set_cam_param();

            // 图像校正
            img_1 = fc.img_rectify(img_1, fc.H1);
            img_2 = fc.img_rectify(img_2, fc.H2);
            img_3 = fc.img_rectify(img_3, fc.H3);

            // 手指边缘提取
            vector<vector<int>> edge_arr = find_all_edges(img_1, img_2, img_3);

            // 灰度归一化
            vector<cv::Mat> img_list = adjust_brightness(img_1, img_2, img_3, edge_arr, 128);
            img_1 = img_list[0];
            img_2 = img_list[1];
            img_3 = img_list[2];

            // 手指三维重建
            fc.reconstruct(edge_arr, 400);
            cout << "重建成功" << endl;

            // 纹理映射
//            map_texture::set_params(fc.coor_c, 520);
            map_texture::set_params(fc.coor_c, 400);
            vector<MatrixXf> P_list(3);
            P_list[0] = fc.P1;
            P_list[1] = fc.P2;
            P_list[2] = fc.P3;
            map_texture::map_texture(fc.X_arr, fc.z_arr, img_list, edge_arr, P_list);
            cout << "纹理映射成功" << endl;

            cv::Mat texture = map_texture::texture_img;
            cv::imwrite(strcat("./map_ori/", img_name), texture );
            cout << "生成原始纹理映射图:" << strcat("./map_ori/", img_name) << endl;

            // 生成纹理图/深度图
//            map_texture::projection (180, 180);// 2度采样一个点
            map_texture::projection_2(180, 180);

            cv::Mat texture_prj = map_texture::texture_prj;
            cv::Mat depth_prj_img = map_texture::depth_prj_img;
            cv::resize(texture_prj, texture_prj, cv::Size(360, 360));
            cv::resize(depth_prj_img, depth_prj_img, cv::Size(360, 360));
            depth_prj_img.convertTo(depth_prj_img, CV_8UC1, 255.0);
            cv::normalize(depth_prj_img, depth_prj_img, 0, 255, cv::NORM_MINMAX);
//            cv::imshow("texture", texture_prj);
//            cv::imshow("depth", depth_prj_img);

            cv::imwrite(strcat("./fv_img/", img_name), texture_prj );
            cv::imwrite(strcat("./fv_dep/", img_name), depth_prj_img );
            cout << "生成纹理图:" << strcat("./fv_img/", img_name) << endl;
            cout << "生成深度图:" << strcat("./fv_dep/", img_name) << endl;
        }
    }
}

int main(int argc,char* argv[]) {
    // std::cout << "Hello, World!" << std::endl;

    // 初始化日志打印配置
    init_log(argv);

    // 初始化glut库
    glutInit(&argc, argv);

//    main_reconstruct();

//    generate_one_map();

    generate_maps();

    return 0;
}