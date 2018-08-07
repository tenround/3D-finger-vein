//
// Created by xhb on 18-8-1.
//

#include "sub_ellipse_optimize.h"

namespace sub_ellipse_opt{
    Eigen::MatrixXf coor_c1, coor_c2, coor_c3;
    double k1, k2, k3, k4, k5, k6;
    double bias1, bias2, bias3, bias4, bias5, bias6;
    double init_center_x, init_center_y, init_radius;
    double b, c;
}

void sub_ellipse_opt::set_params(vector<MatrixXf> coor, vector<double> params, vector<double> b_c)
{
    sub_ellipse_opt::coor_c1 = coor[0];
    sub_ellipse_opt::coor_c2 = coor[1];
    sub_ellipse_opt::coor_c3 = coor[2];

    sub_ellipse_opt::k1 = params[0];
    sub_ellipse_opt::k2 = params[1];
    sub_ellipse_opt::k3 = params[2];
    sub_ellipse_opt::k4 = params[3];
    sub_ellipse_opt::k5 = params[4];
    sub_ellipse_opt::k6 = params[5];

    sub_ellipse_opt::bias1 = params[6];
    sub_ellipse_opt::bias2 = params[7];
    sub_ellipse_opt::bias3 = params[8];
    sub_ellipse_opt::bias4 = params[9];
    sub_ellipse_opt::bias5 = params[10];
    sub_ellipse_opt::bias6 = params[11];

    sub_ellipse_opt::init_center_x  = params[12];
    sub_ellipse_opt::init_center_y  = params[13];
    sub_ellipse_opt::init_radius = params[14];

    sub_ellipse_opt::b = b_c[0];
    sub_ellipse_opt::c = b_c[1];

//    cout << "k1: " << sub_ellipse_opt::k1 << endl;
//    cout << "k2: " << sub_ellipse_opt::k2 << endl;
//    cout << "k3: " << sub_ellipse_opt::k3 << endl;
//    cout << "k4: " << sub_ellipse_opt::k4 << endl;
//    cout << "k5: " << sub_ellipse_opt::k5 << endl;
//    cout << "k6: " << sub_ellipse_opt::k6 << endl;
}

vector<double> sub_ellipse_opt::solve(vector<double> x0)
{
    nlopt::opt opt(nlopt::LN_COBYLA, 3);
    opt.set_min_objective(fun_min, NULL);
    opt.add_inequality_constraint(fun_neq_constraint_all, NULL, 1e-4);
    opt.set_xtol_rel(1e-8);
    opt.set_maxeval(1000);

    double minf_0;
    vector<double> x(3);

    try{
        nlopt::result result_0 = opt.optimize(x0, minf_0);
        cout << "minf: " << minf_0 << endl;
        x = x0;
    }
    catch(exception &e) {
        cout << "first ellipse nopt failed: " << e.what() << endl;
        LOG(INFO) << "first ellipse nopt failed: " << e.what();
    }

    return x;
}

vector<double> sub_ellipse_opt::solve_2(vector<double> x0)
{
    nlopt::opt opt(nlopt::LN_COBYLA, 3);
    opt.set_min_objective(fun_min, NULL);
//    opt.add_inequality_constraint(fun_neq_constraint_all, NULL, 1e-4);
    opt.add_inequality_constraint(fun_neq_constraint_12, NULL, 1e-3);
    opt.add_inequality_constraint(fun_neq_constraint_34, NULL, 1e-3);
    opt.add_inequality_constraint(fun_neq_constraint_56, NULL, 1e-3);
    opt.set_xtol_rel(1e-6);
    opt.set_maxeval(1000);

    double minf_0;
    vector<double> x(3);

    try{
        nlopt::result result_0 = opt.optimize(x0, minf_0);
        cout << "minf: " << minf_0 << endl;
        x = x0;
    }
    catch(exception &e) {
        cout << "first ellipse nopt failed: " << e.what() << endl;
        LOG(INFO) << "first ellipse nopt failed: " << e.what();
    }

    return x;
}

vector<double> sub_ellipse_opt::solve_3(vector<double> x0, vector<double> x1)
{
    nlopt::opt opt(nlopt::LN_COBYLA, 3);
    opt.set_min_objective(fun_min, NULL);
//    opt.add_inequality_constraint(fun_neq_constraint_all, NULL, 1e-4);
    opt.add_inequality_constraint(fun_neq_constraint_1, NULL, 1e-2);
    opt.add_inequality_constraint(fun_neq_constraint_3, NULL, 1e-2);
    opt.add_inequality_constraint(fun_neq_constraint_5, NULL, 1e-2);
    opt.add_inequality_constraint(fun_neq_constraint_2, NULL, 1e-2);
    opt.add_inequality_constraint(fun_neq_constraint_4, NULL, 1e-2);
    opt.add_inequality_constraint(fun_neq_constraint_6, NULL, 1e-2);
    opt.set_xtol_rel(1e-8);
    opt.set_maxeval(1000);

    double minf_0, minf_1;
    vector<double> x(3);

    try{
        nlopt::result result_0 = opt.optimize(x0, minf_0);
        nlopt::result result_1 = opt.optimize(x1, minf_1);
        cout << "minf_0: " << minf_0 << endl;
        cout << "minf_1: " << minf_1 << endl;

        if(minf_0 <= minf_1)
        {
            x = x0;
        }
        else
        {
            x = x1;
        }

        if(isnan(minf_0))
        {
            x = x1;
        }
        if(isnan(minf_1))
        {
            x = x0;
        }
    }
    catch(exception &e) {
        cout << "first ellipse nopt failed: " << e.what() << endl;
        LOG(INFO) << "first ellipse nopt failed: " << e.what();
    }

    return x;
}

double sub_ellipse_opt::fun_min(const vector<double> & x, vector<double> & grad, void * fun_data)
{
    double b, c, d, e, f;
    b = sub_ellipse_opt::b;  c = sub_ellipse_opt::c;
    d = x[0];  e = x[1];  f = x[2];

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

    // 相机1
    double xc = sub_ellipse_opt::coor_c1(0, 0);
    double yc = sub_ellipse_opt::coor_c1(0, 1);
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

    // 相机2
    xc = sub_ellipse_opt::coor_c2(0, 0);
    yc = sub_ellipse_opt::coor_c2(0, 1);
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

    // 相机3
    xc = sub_ellipse_opt::coor_c3(0, 0);
    yc = sub_ellipse_opt::coor_c3(0, 1);
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

    // 计算切点到约束线的距离
    double d1, d2, d3, d4, d5, d6;
    d1 = ((sub_ellipse_opt::k1 * x1 - y1 + sub_ellipse_opt::bias1) / sqrt(sub_ellipse_opt::k1 * sub_ellipse_opt::k1 + 1));
    d2 = ((sub_ellipse_opt::k2 * x2 - y2 + sub_ellipse_opt::bias2) / sqrt(sub_ellipse_opt::k2 * sub_ellipse_opt::k2 + 1));
    d3 = ((sub_ellipse_opt::k3 * x3 - y3 + sub_ellipse_opt::bias3) / sqrt(sub_ellipse_opt::k3 * sub_ellipse_opt::k3 + 1));
    d4 = ((sub_ellipse_opt::k4 * x4 - y4 + sub_ellipse_opt::bias4) / sqrt(sub_ellipse_opt::k4 * sub_ellipse_opt::k4 + 1));
    d5 = ((sub_ellipse_opt::k5 * x5 - y5 + sub_ellipse_opt::bias5) / sqrt(sub_ellipse_opt::k5 * sub_ellipse_opt::k5 + 1));
    d6 = ((sub_ellipse_opt::k6 * x6 - y6 + sub_ellipse_opt::bias6) / sqrt(sub_ellipse_opt::k6 * sub_ellipse_opt::k6 + 1));

    d1 = abs(d1);
    d2 = abs(d2);
    d3 = abs(d3);
    d4 = abs(d4);
    d5 = abs(d5);
    d6 = abs(d6);

//    double distance = d1 * d1 + d2 * d2 + d3 * d3 + d4 * d4 + d5 * d5 + d6 * d6;

    double distance = d1 + d2 + d3 + d4 + d5 + d6;

    return distance;
}

double sub_ellipse_opt::fun_neq_constraint_all(const vector<double> & x, vector<double> & grad, void * fun_data)
{
    double b, c, d, e, f;
    b = sub_ellipse_opt::b;  c = sub_ellipse_opt::c;
    d = x[0];  e = x[1];  f = x[2];
    double k, bias;
    double A, B, C;

    k = sub_ellipse_opt::k1; bias = sub_ellipse_opt::bias1;
    A = 1 + b * k * k;
    B = 2 * k * b * bias + c * bias + d + e * k;
    C = b * bias * bias + e * bias + f;
    double result1 = (B * B - 4 * A * C);

    k = sub_ellipse_opt::k2; bias = sub_ellipse_opt::bias2;
    A = 1 + b * k * k;
    B = 2 * k * b * bias + c * bias + d + e * k;
    C = b * bias * bias + e * bias + f;
    double result2 = (B * B - 4 * A * C);

    k = sub_ellipse_opt::k3; bias = sub_ellipse_opt::bias3;
    A = 1 + b * k * k;
    B = 2 * k * b * bias + c * bias + d + e * k;
    C = b * bias * bias + e * bias + f;
    double result3 = (B * B - 4 * A * C);

    k = sub_ellipse_opt::k4; bias = sub_ellipse_opt::bias4;
    A = 1 + b * k * k;
    B = 2 * k * b * bias + c * bias + d + e * k;
    C = b * bias * bias + e * bias + f;
    double result4 = (B * B - 4 * A * C);

    k = sub_ellipse_opt::k5; bias = sub_ellipse_opt::bias5;
    A = 1 + b * k * k;
    B = 2 * k * b * bias + c * bias + d + e * k;
    C = b * bias * bias + e * bias + f;
    double result5 = (B * B - 4 * A * C);

    k = sub_ellipse_opt::k6; bias = sub_ellipse_opt::bias6;
    A = 1 + b * k * k;
    B = 2 * k * b * bias + c * bias + d + e * k;
    C = b * bias * bias + e * bias + f;
    double result6 = (B * B - 4 * A * C);

    double result = result1 + result2 + result3 + result4 + result5 + result6;

    return result;
}


double sub_ellipse_opt::fun_neq_constraint_12(const vector<double> & x, vector<double> & grad, void * fun_data)
{
    double b, c, d, e, f;
    b = sub_ellipse_opt::b;  c = sub_ellipse_opt::c;
    d = x[0];  e = x[1];  f = x[2];
    double k, bias;
    double A, B, C;

    k = sub_ellipse_opt::k1; bias = sub_ellipse_opt::bias1;
    A = 1 + b * k * k;
    B = 2 * k * b * bias + c * bias + d + e * k;
    C = b * bias * bias + e * bias + f;
    double result1 = (B * B - 4 * A * C);

    k = sub_ellipse_opt::k2; bias = sub_ellipse_opt::bias2;
    A = 1 + b * k * k;
    B = 2 * k * b * bias + c * bias + d + e * k;
    C = b * bias * bias + e * bias + f;
    double result2 = (B * B - 4 * A * C);

    double result = result1 + result2;

    return result;
}

double sub_ellipse_opt::fun_neq_constraint_34(const vector<double> & x, vector<double> & grad, void * fun_data)
{
    double b, c, d, e, f;
    b = sub_ellipse_opt::b;  c = sub_ellipse_opt::c;
    d = x[0];  e = x[1];  f = x[2];
    double k, bias;
    double A, B, C;

    k = sub_ellipse_opt::k3; bias = sub_ellipse_opt::bias3;
    A = 1 + b * k * k;
    B = 2 * k * b * bias + c * bias + d + e * k;
    C = b * bias * bias + e * bias + f;
    double result3 = (B * B - 4 * A * C);

    k = sub_ellipse_opt::k4; bias = sub_ellipse_opt::bias4;
    A = 1 + b * k * k;
    B = 2 * k * b * bias + c * bias + d + e * k;
    C = b * bias * bias + e * bias + f;
    double result4 = (B * B - 4 * A * C);

    double result = result3 + result4;

    return result;
}

double sub_ellipse_opt::fun_neq_constraint_56(const vector<double> & x, vector<double> & grad, void * fun_data)
{
    double b, c, d, e, f;
    b = sub_ellipse_opt::b;  c = sub_ellipse_opt::c;
    d = x[0];  e = x[1];  f = x[2];
    double k, bias;
    double A, B, C;

    k = sub_ellipse_opt::k5; bias = sub_ellipse_opt::bias5;
    A = 1 + b * k * k;
    B = 2 * k * b * bias + c * bias + d + e * k;
    C = b * bias * bias + e * bias + f;
    double result5 = (B * B - 4 * A * C);

    k = sub_ellipse_opt::k6; bias = sub_ellipse_opt::bias6;
    A = 1 + b * k * k;
    B = 2 * k * b * bias + c * bias + d + e * k;
    C = b * bias * bias + e * bias + f;
    double result6 = (B * B - 4 * A * C);

    double result = result5 + result6;

    return result;
}


double sub_ellipse_opt::fun_neq_constraint_1(const vector<double> & x, vector<double> & grad, void * fun_data)
{
    double b, c, d, e, f;
    b = sub_ellipse_opt::b;  c = sub_ellipse_opt::c;
    d = x[0];  e = x[1];  f = x[2];
    double k, bias;
    k = sub_ellipse_opt::k1; bias = sub_ellipse_opt::bias1;
    double A = 1 + b * k * k;
    double B = 2 * k * b * bias + c * bias + d + e * k;
    double C = b * bias * bias + e * bias + f;
    double result = (B * B - 4 * A * C);
    return result;
}

double sub_ellipse_opt::fun_neq_constraint_2(const vector<double> & x, vector<double> & grad, void * fun_data)
{
    double b, c, d, e, f;
    b = sub_ellipse_opt::b;  c = sub_ellipse_opt::c;
    d = x[0];  e = x[1];  f = x[2];
    double k, bias;
    k = sub_ellipse_opt::k2; bias = sub_ellipse_opt::bias2;
    double A = 1 + b * k * k;
    double B = 2 * k * b * bias + c * bias + d + e * k;
    double C = b * bias * bias + e * bias + f;
    double result = (B * B - 4 * A * C);
    return result;
}


double sub_ellipse_opt::fun_neq_constraint_3(const vector<double> & x, vector<double> & grad, void * fun_data)
{
    double b, c, d, e, f;
    b = sub_ellipse_opt::b;  c = sub_ellipse_opt::c;
    d = x[0];  e = x[1];  f = x[2];
    double k, bias;
    k = sub_ellipse_opt::k3; bias = sub_ellipse_opt::bias3;
    double A = 1 + b * k * k;
    double B = 2 * k * b * bias + c * bias + d + e * k;
    double C = b * bias * bias + e * bias + f;
    double result = (B * B - 4 * A * C);
    return result;
}

double sub_ellipse_opt::fun_neq_constraint_4(const vector<double> & x, vector<double> & grad, void * fun_data)
{
    double b, c, d, e, f;
    b = sub_ellipse_opt::b;  c = sub_ellipse_opt::c;
    d = x[0];  e = x[1];  f = x[2];
    double k, bias;
    k = sub_ellipse_opt::k4; bias = sub_ellipse_opt::bias4;
    double A = 1 + b * k * k;
    double B = 2 * k * b * bias + c * bias + d + e * k;
    double C = b * bias * bias + e * bias + f;
    double result = (B * B - 4 * A * C);
    return result;
}

double sub_ellipse_opt::fun_neq_constraint_5(const vector<double> & x, vector<double> & grad, void * fun_data)
{
    double b, c, d, e, f;
    b = sub_ellipse_opt::b;  c = sub_ellipse_opt::c;
    d = x[0];  e = x[1];  f = x[2];
    double k, bias;
    k = sub_ellipse_opt::k5; bias = sub_ellipse_opt::bias5;
    double A = 1 + b * k * k;
    double B = 2 * k * b * bias + c * bias + d + e * k;
    double C = b * bias * bias + e * bias + f;
    double result = (B * B - 4 * A * C);
    return result;
}

double sub_ellipse_opt::fun_neq_constraint_6(const vector<double> & x, vector<double> & grad, void * fun_data)
{
    double b, c, d, e, f;
    b = sub_ellipse_opt::b;  c = sub_ellipse_opt::c;
    d = x[0];  e = x[1];  f = x[2];
    double k, bias;
    k = sub_ellipse_opt::k6; bias = sub_ellipse_opt::bias6;
    double A = 1 + b * k * k;
    double B = 2 * k * b * bias + c * bias + d + e * k;
    double C = b * bias * bias + e * bias + f;
    double result = (B * B - 4 * A * C);
    return result;
}