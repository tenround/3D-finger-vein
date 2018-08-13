//
// Created by xhb on 18-7-27.
//

#include "preprocess.h"
#include <vector>

// 导入图片
vector<cv::Mat> load_triple_imgs(string folder, string img_name)
{
    stringstream filename_1, filename_2, filename_3;
    filename_1 << folder << "1/" << img_name;
    filename_2 << folder << "2/" << img_name;
    filename_3 << folder << "3/" << img_name;

    cv::Mat img_1, img_2, img_3;
    img_1 = cv::imread(filename_1.str());
    img_2 = cv::imread(filename_2.str());
    img_3 = cv::imread(filename_3.str());

    cv::cvtColor(img_1, img_1, CV_BGR2GRAY);
    cv::cvtColor(img_2, img_2, CV_BGR2GRAY);
    cv::cvtColor(img_3, img_3, CV_BGR2GRAY);

    vector<cv::Mat> triple_img_list;
    triple_img_list.push_back(img_1);
    triple_img_list.push_back(img_2);
    triple_img_list.push_back(img_3);

    return triple_img_list;
}

vector<cv::Mat> adjust_brightness(cv::Mat img_1, cv::Mat img_2, cv::Mat img_3, vector<vector<int>> edge_arr, int setVal)
{
    vector<int> u_1 = edge_arr[0];
    vector<int> b_1 = edge_arr[1];
    vector<int> u_2 = edge_arr[2];
    vector<int> b_2 = edge_arr[3];
    vector<int> u_3 = edge_arr[4];
    vector<int> b_3 = edge_arr[5];

    float avgVal_1 = avgCols(img_1, u_1, b_1);
    float avgVal_2 = avgCols(img_2, u_2, b_2);
    float avgVal_3 = avgCols(img_3, u_3, b_3);

    cv::Mat img_1_ad;
    img_1_ad.create(img_1.size(), img_1.type());
    cv::Mat img_2_ad;
    img_2_ad.create(img_2.size(), img_2.type());
    cv::Mat img_3_ad;
    img_3_ad.create(img_3.size(), img_3.type());

    // 遍历三张图像调整灰度
    int N_rows = img_1.rows;
    int N_cols = img_1.cols;
    // img_1
    for(int i=0; i<N_rows; i++)
    {
        const uchar* img_ptr = img_1.ptr<uchar>(i);
        uchar* dst_ptr = img_1_ad.ptr<uchar>(i);
        for(int j=0; j<N_cols; j++)
        {
            int pixVal = (int)*img_ptr++;
            int dstVal = pixVal + setVal - (int)avgVal_1;
            if(dstVal < 0)
                dstVal = 0;
            else if(dstVal > 255)
                dstVal = 255;
            *dst_ptr++ = (uchar)dstVal;
        }
    }

    // img_2
    for(int i=0; i<N_rows; i++)
    {
        const uchar* img_ptr = img_2.ptr<uchar>(i);
        uchar* dst_ptr = img_2_ad.ptr<uchar>(i);
        for(int j=0; j<N_cols; j++)
        {
            int pixVal = (int)*img_ptr++;
            int dstVal = pixVal + setVal - (int)avgVal_2;
            if(dstVal < 0)
                dstVal = 0;
            else if(dstVal > 255)
                dstVal = 255;
            *dst_ptr++ = (uchar)dstVal;
        }
    }

    // img_3
    for(int i=0; i<N_rows; i++)
    {
        const uchar* img_ptr = img_3.ptr<uchar>(i);
        uchar* dst_ptr = img_3_ad.ptr<uchar>(i);
        for(int j=0; j<N_cols; j++)
        {
            int pixVal = (int)*img_ptr++;
            int dstVal = pixVal + setVal - (int)avgVal_3;
            if(dstVal < 0)
                dstVal = 0;
            else if(dstVal > 255)
                dstVal = 255;
            *dst_ptr++ = (uchar)dstVal;
        }
    }

    vector<cv::Mat> img_ad_list;
    img_ad_list.push_back(img_1_ad);
    img_ad_list.push_back(img_2_ad);
    img_ad_list.push_back(img_3_ad);

    return img_ad_list;
}

float avgCols(cv::Mat src, vector<int> upper, vector<int> bottom)
{
    int min_y = 91;
    int max_y = 490;

    int N_cols = max_y - min_y + 1;

    float sumVal = 0;
    float sumPix = 0;
    for(int i=0; i<N_cols; i++)
    {
        int upper_y = upper[i];
        int bottom_y = bottom[i];
        for(int j=upper_y; j<=bottom_y; j++)
        {
            sumVal = sumVal + src.at<uchar>(j, i+min_y);
        }
        sumPix = sumPix + bottom_y - upper_y + 1;
    }

    float avgVal = sumVal / sumPix;

    return avgVal;
}