//
// Created by xhb on 18-8-1.
//

#ifndef INC_3D_FINGER_VEIN_COOR_PLOT_H
#define INC_3D_FINGER_VEIN_COOR_PLOT_H

#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <GL/glut.h>
#include <numeric>
#include "matplotlibcpp.h"

#define pi M_PI

namespace plt = matplotlibcpp;
using namespace std;

class coor_plot {
public:
    void create_figure();
    void show();
    void plot_constraint_line(float k, float bias);
    void plot_ellipse(vector<float> x);
//    void plot_3d_model(vector<vector<float>> X_arr, vector<float> z_arr);
    void save(string filename);
    void close();

private:


};

namespace coor_plot_3d {
    // 函数
    void set_point_cloud(vector<vector<float>> X_3D, vector<vector<float>> Y_3D, vector<vector<float>> Z_3D);
    void render_screen();
    void setupRenderingState();
    void changeSize(GLint w, GLint h);
    void keyFunc(int key, int x, int y);
    void mouseFunc(int buttion, int state, int x, int y);
    void mouseMotion(int x, int y);
    void plot_3d_model(vector<vector<float>> X, vector<vector<float>> Y, vector<vector<float>> Z);

    void render_screen_finger_vein();
    void set_finger_vein_point_cloud(vector<vector<float>> X, vector<vector<float>> Y, vector<vector<float>> Z, vector<vector<unsigned char>> texture);
    void plot_3d_finger_vein_model(vector<vector<float>> X, vector<vector<float>> Y, vector<vector<float>> Z, vector<vector<unsigned char>> texture);
};


#endif //INC_3D_FINGER_VEIN_COOR_PLOT_H
