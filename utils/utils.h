//
// Created by xhb on 18-7-27.
//

#ifndef INC_3D_FINGER_VEIN_UTILS_H
#define INC_3D_FINGER_VEIN_UTILS_H

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <time.h>
#include <sys/syscall.h>
#include <glog/logging.h>
#include <sys/time.h>

using namespace std;
using namespace cv;


void int2str(const int & int_temp, string & string_temp);
void str2int(const string & string_temp, int & int_temp);
string gen_img_name(int sub, int t);
void init_log(char* argv[]);
double poly_num(int iu, int il, int i, double lx, double ux);
vector<double> interp1_linear(vector<double> x1, vector<double> x2, vector<double> x2_dst);
void test_interp();
long getCurrentTime();

#endif //INC_3D_FINGER_VEIN_UTILS_H
