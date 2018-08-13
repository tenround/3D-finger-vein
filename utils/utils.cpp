//
// Created by xhb on 18-7-27.
//

#include "utils.h"



// int转string
void int2str(const int & int_temp, string & string_temp)
{
    stringstream stream;
    stream << int_temp;
    string_temp = stream.str();
}

// string转int
void str2int(const string & string_temp, int & int_temp)
{
    stringstream stream(string_temp);
    stream >> int_temp;
}

// 生成img_name
string gen_img_name(int sub, int t)
{
    string sub_str, t_str;
    stringstream img_name_stream;
    int2str(sub, sub_str);
    int2str(t, t_str);
    img_name_stream << sub << "-" << t << ".bmp";
    return img_name_stream.str();
}

// 初始化glog
void init_log(char* argv[])
{
    google::InitGoogleLogging(argv[0]); // 初始化 glog
    google::SetLogDestination(google::GLOG_INFO, "./log/log");  // 设置log地址
    LOG(INFO) << "调试日志（许鸿斌）";
}


// 在两个值之间做插值
float poly_num(int iu, int il, int i, float lx, float ux)
{
    return lx * (float)(iu - i) / (float)(iu - il) + ux * (float)(i - il) / (float)(iu - il);
}

// 分段线性插值法
vector<float> interp1_linear(vector<float> x1, vector<float> x2, vector<float> x2_dst)
{
    // x1与x2等长
    // x2_dst是x2的目标数组，输入
    // x1_dst是x1的目标数组，返回
    int x1_len = x1.size();
    int dst_len = x2_dst.size();
    float interval;
    vector<float> x1_dst(dst_len);
    vector<int> idx_list;
    int i, j=0;
    int iu, il, idx;
    float lx, ux;
    for(i=0; i<dst_len;i++)
    {
        if(x2_dst[i] >= x2[j])
        {
            idx_list.push_back(i);
            j++;
        }
    }

    // lx, ux, iu, il, i
    for(idx=0; idx<idx_list.size()-1;idx++)
    {
        il = idx_list[idx];
        iu = idx_list[idx+1];
        for(int k=il;k<=iu;k++)
        {
            lx = x1[idx];
            ux = x1[idx+1];
            x1_dst[k] = poly_num(iu, il, k, lx, ux);
//            cout << x1_dst[k] << endl;
        }
    }

    return x1_dst;
}

void test_interp()
{
    vector<float> z;
    vector<float> x;
    vector<float> z_dst;
    vector<float> x_dst;
    for(int i=1;i<=400;i++)
    {
        z_dst.push_back((float)i);
    }
    for(int i=1;i<=400;i=i+20)
    {
        z.push_back(i);
        x.push_back(i-1);
    }
    z.push_back(400);
    x.push_back(399);
    x_dst = interp1_linear(x, z, z_dst);

}

// 测量时间
long getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

string strcat(string s1, string s2)
{
    stringstream ss;
    ss << s1 << s2;
    return ss.str();
}

string strcat(string s1, string s2, string s3)
{
    stringstream ss;
    ss << s1 << s2 << s3;
    return ss.str();
}