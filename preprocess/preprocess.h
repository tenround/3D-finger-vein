//
// Created by xhb on 18-7-27.
//

#ifndef INC_3D_FINGER_VEIN_PREPROCESS_H
#define INC_3D_FINGER_VEIN_PREPROCESS_H

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

vector<cv::Mat> load_triple_imgs(string folder, string img_name);
vector<cv::Mat> adjust_brightness(cv::Mat img_1, cv::Mat img_2, cv::Mat img_3, vector<vector<int>> edge_arr, int setVal);
float avgCols(cv::Mat src, vector<int> upper, vector<int> bottom);

#endif //INC_3D_FINGER_VEIN_PREPROCESS_H
