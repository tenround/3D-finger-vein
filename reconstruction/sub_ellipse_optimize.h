//
// Created by xhb on 18-8-1.
//

#ifndef INC_3D_FINGER_VEIN_SUB_ELLIPSE_OPTIMIZE_H
#define INC_3D_FINGER_VEIN_SUB_ELLIPSE_OPTIMIZE_H

#include <iostream>
#include <vector>
#include <nlopt.hpp>
#include <Eigen/Dense>
#include <math.h>
#include <glog/logging.h>

using namespace std;
using namespace Eigen;
// using namespace nlopt;

namespace sub_ellipse_opt {
    extern double minf_0, minf_1;

    void set_params(vector<MatrixXf> coor, vector<double> params, vector<double> b_c);
    vector<double> solve(vector<double> x0);
    vector<double> solve_2(vector<double> x0);
    vector<double> solve_3(vector<double> x0, vector<double> x1);
    double fun_min(const vector<double> & x, vector<double> & grad, void * fun_data);
    double fun_neq_constraint_all(const vector<double> & x, vector<double> & grad, void * fun_data);
    double fun_neq_constraint_12(const vector<double> & x, vector<double> & grad, void * fun_data);
    double fun_neq_constraint_34(const vector<double> & x, vector<double> & grad, void * fun_data);
    double fun_neq_constraint_56(const vector<double> & x, vector<double> & grad, void * fun_data);
    double fun_neq_constraint_1(const vector<double> & x, vector<double> & grad, void * fun_data);
    double fun_neq_constraint_2(const vector<double> & x, vector<double> & grad, void * fun_data);
    double fun_neq_constraint_3(const vector<double> & x, vector<double> & grad, void * fun_data);
    double fun_neq_constraint_4(const vector<double> & x, vector<double> & grad, void * fun_data);
    double fun_neq_constraint_5(const vector<double> & x, vector<double> & grad, void * fun_data);
    double fun_neq_constraint_6(const vector<double> & x, vector<double> & grad, void * fun_data);
}


#endif //INC_3D_FINGER_VEIN_SUB_ELLIPSE_OPTIMIZE_H
