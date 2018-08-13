//
// Created by xhb on 18-8-1.
//

#include "coor_plot.h"

void coor_plot::create_figure()
{
//    plt::figure();
//    plt::clf();
    plt::figure_size(1200, 780);
}

void coor_plot::show()
{
    plt::xlim(-20, 20);
    plt::ylim(-20, 20);
    plt::show();
//    plt::clf();
}

void coor_plot::close()
{
//    plt::clf();
    plt::close();
}

void coor_plot::plot_constraint_line(float k, float bias)
{
    short n = 500;       // num of points
    float x_max = 20.;
    float x_min = -20.;
    float step = (x_max - x_min) / (n - 1);
    vector<float> x, y;
    // int idx = 0;
    for(float i=x_min;i<=x_max;i=i+step)
    {
        x.push_back(i);
        y.push_back(k * i + bias);
    }
    // plot the constraint line, color is selected automatically
    plt::plot(x, y);
}

void coor_plot::plot_ellipse(vector<float> x)
{
    float a, b, c, d, e, f;
    a = 1;  b = x[0]; c = x[1];
    d = x[2];  e = x[3];  f = x[4];

    // 椭圆方程： a*x*x + b*y*y + c*x*y + d*x + e*y + f = 0
    float x0, y0;
    x0 = (b*d - c*e) / (-c*c + b);
    y0 = (e - c*d) / (-c*c + b);
    x0 = x0 / 2; y0 = y0 / 2;
    float alpha = atan(2 * c / (b - 1)) / 2;
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
    float aa, bb, ff, rx, ry;
    aa = cos(alpha)*(cos(alpha) - c*sin(alpha)) - sin(alpha)*(c*cos(alpha) - b*sin(alpha));
    bb = cos(alpha)*(b*cos(alpha) + c*sin(alpha)) + sin(alpha)*(sin(alpha) + c*cos(alpha));
    ff = f + y0*(b*y0 - e + c*x0) + x0*(x0 - d + c*y0) - y0*(c*cos(alpha) - b*sin(alpha)) - x0*(cos(alpha) - c*sin(alpha)) - y0*(b*cos(alpha) + c*sin(alpha)) - x0*(sin(alpha) + c*cos(alpha));
    rx = sqrt(-ff/aa); // 长轴
    ry = sqrt(-ff/bb); // 短轴

    Eigen::Matrix2f R;
    R << cos(alpha), sin(alpha),
        -sin(alpha), cos(alpha);

    float theta;
    float n = 500;
    float step = (2*M_PI) / (n - 1);
    float ellipse_x, ellipse_y;
    vector<float> ellipse_x_list, ellipse_y_list;
    Eigen::Vector2f ellipse_xy;
    Eigen::Vector2f Rotation_ellipse;
    for(theta=0; theta<=2*M_PI; theta+=step)
    {
        ellipse_x = -x0 + rx * cos(theta);
        ellipse_y = -y0 + ry * sin(theta);
        ellipse_xy << ellipse_x, ellipse_y;
        Rotation_ellipse = R * ellipse_xy;

        // 旋转后得到椭圆的实际坐标
        ellipse_x = Rotation_ellipse(0);
        ellipse_y = Rotation_ellipse(1);

        ellipse_x_list.push_back(ellipse_x);
        ellipse_y_list.push_back(ellipse_y);
    }
    // plot the ellipse, color is selected automatically
    plt::plot(ellipse_x_list, ellipse_y_list);


}

void coor_plot::save(string filename)
{
    plt::xlim(-20, 20);
    plt::ylim(-20, 20);
    stringstream fullname;
    fullname << "./images/" << filename << ".jpg";
    plt::save(fullname.str());
//    plt::clf();
}

namespace coor_plot_3d {
    // 获取屏幕的宽度
    GLint SCREEN_WIDTH = 0;
    GLint SCREEN_HEIGHT = 0;
    // 设置程序的窗口大小
    GLint windowWidth = 800;
    GLint windowHeight = 600;

    // 受支持的点大小范围
    GLfloat sizes[2];
    // 受支持的点大小增量
    GLfloat step;
    // 鼠标左键是否按下
    bool mouseLeftDown;
    // 鼠标右键是否按下
    bool mouseRightDown;
    // 鼠标位置，X、Y坐标
    float mouseX, mouseY;
    // 相机距离
    float cameraDistance;
    // 绕x轴旋转角度
    GLfloat xRotAngle;
    // 绕y轴旋转角度
    GLfloat zRotAngle;
    // 缩放程度
    GLfloat times=1;

    // 待绘制点云的XYZ坐标
    vector<vector<float>> X_3D;
    vector<vector<float>> Y_3D;
    vector<vector<float>> Z_3D;
    vector<vector<unsigned char>> texture_3D;
};

// 将三维点云的XYZ坐标放入类中
void coor_plot_3d::set_point_cloud(vector<vector<float>> X, vector<vector<float>> Y, vector<vector<float>> Z)
{
    X_3D = X;
    Y_3D = Y;
    Z_3D = Z;
}

// 屏幕响应函数
void coor_plot_3d::render_screen()
{
    GLfloat x, y, z, angle;
    // 当前点的大小
    GLfloat curSize = 0.0f;
    // 把整个窗口清理为当前清理颜色：黑色
    glClear(GL_COLOR_BUFFER_BIT);
    // 将当前Matrix状态入栈
    glPushMatrix();

    // 键盘控制，鼠标控制
    // 缩放
    glScalef(times, times, times);
    // 平移一定距离
    glTranslated(0, 0, cameraDistance);
    // 坐标系绕x轴旋转xRotAngle
    glRotatef(xRotAngle, 1.0f, 0.0f, 0.0f);
    // 坐标系绕y轴旋转zRotAngle
    glRotatef(zRotAngle, 0.0f, 0.0f, 1.0f);

    // 绘制坐标系
    // x轴
    //平滑处理
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH, GL_NICEST);
    GLfloat x_start = -60.0f;
    GLfloat x_end = 60.0f;
    GLfloat array_len = 10.0f;
    glColor3f(1.0f, 0.0f, 0.0f);
    glLineWidth(3);
    glBegin(GL_LINES);
    glVertex3f(x_start, 0.0f, 0.0f);
    glVertex3f(x_end, 0.0f, 0.0f);

    glVertex3f(x_end, 0.0f, 0.0f);
    glVertex3f(x_end-array_len, array_len, 0.0f);

    glVertex3f(x_end, 0.0f, 0.0f);
    glVertex3f(x_end-array_len, -array_len, 0.0f);
    glEnd();

    // y轴
    GLfloat y_start = -60.0f;
    GLfloat y_end = 60.0f;
    glColor3f(0.0f, 1.0f, 0.0f);
    glLineWidth(3);
    glBegin(GL_LINES);
    glVertex3f(0.0f, y_start, 0.0f);
    glVertex3f(0.0f, y_end, 0.0f);

    glVertex3f(0.0f, y_end, 0.0f);
    glVertex3f(array_len, y_end-array_len, 0.0f);

    glVertex3f(0.0f, y_end, 0.0f);
    glVertex3f(-array_len, y_end-array_len, 0.0f);
    glEnd();

    // z轴
    GLfloat z_start = -60.0f;
    GLfloat z_end = 60.0f;
    glColor3f(0.0f, 0.0f, 1.0f);
    glLineWidth(3);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, z_start);
    glVertex3f(0.0f, 0.0f, z_end);

    glVertex3f(0.0f, 0.0f, z_end);
    glVertex3f(array_len, 0.0f, z_end-array_len);

    glVertex3f(0.0f, 0.0f, z_end);
    glVertex3f(-array_len, 0.0f, z_end-array_len);
    glEnd();

    // 当前点的大小，默认设为最小
    curSize = 3;
    int finger_len = (int)X_3D.size();
    int ellipse_len = (int)X_3D[0].size();
    //设置点的大小
    glPointSize(curSize);
    //平滑处理
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH, GL_NICEST);
    glColor3f(0.0f, 1.0f, 1.0f);
    glBegin(GL_POINTS);
    for(int i=0; i<finger_len; i++)
    {
        for(int j=0; j<ellipse_len; j++)
        {
            x = (GLfloat)X_3D[i][j];
            x = x * 4;
            y = (GLfloat)Y_3D[i][j];
            y = y * 4;

            z = (GLfloat)Z_3D[i][0];
            z = (z - 290) / 4;

            glVertex3f(x, y, z);
        }

    }
    glEnd();

    // 恢复压入栈的矩阵
    glPopMatrix();
    // 交换两个缓冲区的指针
    glutSwapBuffers();
}

//设置Rendering State
void coor_plot_3d::setupRenderingState()
{
    // 设置背景颜色为白色
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    // 点的颜色为黄色
    glColor3f(1.0f, 1.0f, 1.0f);
    // 获取所支持的点的大小的取值范围
    glGetFloatv(GL_POINT_SIZE_RANGE, sizes);
    // 获取所支持的点的大小增量
    // glGetFloatv(GL_POINT_SIZE_GRANULARITY, &step);

}

void coor_plot_3d::changeSize(GLint w, GLint h)
{
    // 长宽比
    GLfloat ratio;
    //设置坐标系为x(-100.0f,100.0f)、y(-100.0f,100.0f)、z(-100.0f,100.0f)
    GLfloat coordinates_size = 100.0f;
    // 窗口宽高为0，直接返回
    if((w == 0) || (h == 0))
        return;
    // 设置视口与窗口大小一致
    glViewport(0, 0, w, h);
    // 对投影矩阵应用随后的矩阵操作
    glMatrixMode(GL_PROJECTION);
    // 重置当前指定的矩阵为单位矩阵
    glLoadIdentity();
    ratio = (GLfloat)w / (GLfloat)h;
    // 正交投影
//    glOrtho(-200, 200, -1000, 1000, -1000, 6000);
    if(w < h)
        glOrtho(-coordinates_size, coordinates_size, -coordinates_size/ratio, coordinates_size/ratio, -coordinates_size, coordinates_size);
    else
        glOrtho(-coordinates_size*ratio, coordinates_size*ratio, -coordinates_size, coordinates_size, -coordinates_size, coordinates_size);
    // 对模型视图矩阵堆栈应用随后的矩阵操作
    glMatrixMode(GL_MODELVIEW);
    // 重置当前指定的矩阵为单位矩阵
    glLoadIdentity();

}

// 键盘输入处理函数
void coor_plot_3d::keyFunc(int key, int x, int y)
{
    if(key == GLUT_KEY_UP)
    {
        xRotAngle -= 5.0f;
    }
    else if(key == GLUT_KEY_DOWN)
    {
        xRotAngle += 5.0f;
    }
    else if(key == GLUT_KEY_LEFT)
    {
        zRotAngle -= 5.0f;
    }
    else if(key == GLUT_KEY_RIGHT)
    {
        zRotAngle += 5.0f;
    }
    else if(key == GLUT_KEY_F1)
    {
        exit(0);
    }
    // 重新绘制
    glutPostRedisplay();
}

// 鼠标事件处理函数
void coor_plot_3d::mouseFunc(int button, int state, int x, int y)
{
    mouseX = x;
    mouseY = y;

    if(button == GLUT_LEFT_BUTTON)
    {
        if(state == GLUT_DOWN)
        {
            mouseLeftDown = true;
        }
        else if(state == GLUT_UP)
        {
            mouseLeftDown = false;
        }
    }
    else if(button == GLUT_RIGHT_BUTTON)
    {
        if(state == GLUT_DOWN)
        {
            mouseRightDown = true;
        }
        else if(state == GLUT_UP)
        {
            mouseRightDown = false;
        }
    }
}

// 鼠标事件响应函数
void coor_plot_3d::mouseMotion(int x, int y)
{
    if(mouseLeftDown)
    {
        zRotAngle += (x - mouseX);
        xRotAngle += (y - mouseY);
        mouseX = x;
        mouseY = y;
    }

    if(mouseRightDown)
    {
//        cameraDistance += (y - mouseY) * 0.2f;
        times += (y - mouseY) * 0.008f;
        mouseY = y;
    }

    // 重新绘制
    glutPostRedisplay();
}

void coor_plot_3d::plot_3d_model(vector<vector<float>> X, vector<vector<float>> Y, vector<vector<float>> Z)
{
    set_point_cloud(X, Y, Z);

    xRotAngle = 0.0f;
    zRotAngle = 0.0f;

//    glutInit(&argc, argv);
    // 使用双缓冲区模式
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    //获取系统的宽像素
    SCREEN_WIDTH =  glutGet(GLUT_SCREEN_WIDTH);
    //获取系统的高像素
    SCREEN_HEIGHT = glutGet(GLUT_SCREEN_HEIGHT);
    //创建窗口，窗口名字为3DFM
    glutCreateWindow("3DFM");
    //设置窗口大小
    glutReshapeWindow(windowWidth, windowHeight);
    //窗口居中显示
    glutPositionWindow((SCREEN_WIDTH-windowWidth)/2, (SCREEN_HEIGHT-windowHeight)/2);
    //设置窗口大小变化时的回调函数
    glutReshapeFunc(changeSize);
    //设置显示回调函数
    glutDisplayFunc(render_screen);
    //设置按键输入处理回调函数
    glutSpecialFunc(keyFunc);
    glutMouseFunc(mouseFunc);
    glutMotionFunc(mouseMotion);
    //设置全局渲染参数
    setupRenderingState();
    glutMainLoop();

}

void coor_plot_3d::render_screen_finger_vein()
{
    GLfloat x, y, z, angle;
    // 当前点的大小
    GLfloat curSize = 0.0f;
    // 把整个窗口清理为当前清理颜色：黑色
    glClear(GL_COLOR_BUFFER_BIT);
    // 将当前Matrix状态入栈
    glPushMatrix();

    // 键盘控制，鼠标控制
    // 缩放
    glScalef(times, times, times);
    // 平移一定距离
    glTranslated(0, 0, cameraDistance);
//    cout << "cameraDistance:" << cameraDistance << endl;
    // 坐标系绕x轴旋转xRotAngle
    glRotatef(xRotAngle, 1.0f, 0.0f, 0.0f);
//    cout << "xRotAngle: " << xRotAngle << endl;
    // 坐标系绕y轴旋转zRotAngle
    glRotatef(zRotAngle, 0.0f, 0.0f, 1.0f);
//    cout << "zRotAngle: " << zRotAngle << endl;

    // 绘制坐标系
    // x轴
    //平滑处理
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH, GL_NICEST);
    GLfloat x_start = -80.0f;
    GLfloat x_end = 80.0f;
    GLfloat array_len = 10.0f;
    glColor3f(1.0f, 0.0f, 0.0f);
    glLineWidth(3);
    glBegin(GL_LINES);
    glVertex3f(x_start, 0.0f, 0.0f);
    glVertex3f(x_end, 0.0f, 0.0f);

    glVertex3f(x_end, 0.0f, 0.0f);
    glVertex3f(x_end-array_len, array_len, 0.0f);

    glVertex3f(x_end, 0.0f, 0.0f);
    glVertex3f(x_end-array_len, -array_len, 0.0f);
    glEnd();

    // y轴
    GLfloat y_start = -80.0f;
    GLfloat y_end = 80.0f;
    glColor3f(0.0f, 1.0f, 0.0f);
    glLineWidth(3);
    glBegin(GL_LINES);
    glVertex3f(0.0f, y_start, 0.0f);
    glVertex3f(0.0f, y_end, 0.0f);

    glVertex3f(0.0f, y_end, 0.0f);
    glVertex3f(array_len, y_end-array_len, 0.0f);

    glVertex3f(0.0f, y_end, 0.0f);
    glVertex3f(-array_len, y_end-array_len, 0.0f);
    glEnd();

    // z轴
    GLfloat z_start = -80.0f;
    GLfloat z_end = 80.0f;
    glColor3f(0.0f, 0.0f, 1.0f);
    glLineWidth(3);
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, z_start);
    glVertex3f(0.0f, 0.0f, z_end);

    glVertex3f(0.0f, 0.0f, z_end);
    glVertex3f(array_len, 0.0f, z_end-array_len);

    glVertex3f(0.0f, 0.0f, z_end);
    glVertex3f(-array_len, 0.0f, z_end-array_len);
    glEnd();

    // 当前点的大小，默认设为最小
    curSize = 3;
    int finger_len = (int)X_3D.size();
    int ellipse_len = (int)X_3D[0].size();
    //设置点的大小
    glPointSize(curSize);
    //平滑处理
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH, GL_NICEST);
//    glColor3f(1.0f, 1.0f, 0.0f);
    glBegin(GL_POINTS);
    for(int i=0; i<finger_len; i++)
    {
        for(int j=0; j<ellipse_len; j++)
        {
//            vector<vector<unsigned char>> texture_temp = texture_3D;
            unsigned char texture = texture_3D[i][j];
            float gray_val_texture = (float)texture / 255;
            glColor3f((GLfloat)gray_val_texture, (GLfloat)gray_val_texture, (GLfloat)gray_val_texture);
//            glColor3f(1.0f, 1.0f, 0.0f);

            x = (GLfloat)X_3D[i][j];
            x = x * 4;
            y = (GLfloat)Y_3D[i][j];
            y = y * 4;
            z = (GLfloat)Z_3D[i][0];
            z = z * 3;
//            z = (z - 290) / 4;

            glVertex3f(x, y, z);
        }

    }
    glEnd();

    // 恢复压入栈的矩阵
    glPopMatrix();
    // 交换两个缓冲区的指针
    glutSwapBuffers();
}

void coor_plot_3d::set_finger_vein_point_cloud(vector<vector<float>> X, vector<vector<float>> Y, vector<vector<float>> Z, vector<vector<unsigned char>> texture)
{
    X_3D = X;
    Y_3D = Y;
    Z_3D = Z;
    texture_3D = texture;
}

void coor_plot_3d::plot_3d_finger_vein_model(vector<vector<float>> X, vector<vector<float>> Y, vector<vector<float>> Z, vector<vector<unsigned char>> texture)
{
    set_finger_vein_point_cloud(X, Y, Z, texture);

    xRotAngle = 0.0f;
    zRotAngle = 0.0f;

//    glutInit(&argc, argv);
    // 使用双缓冲区模式
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    //获取系统的宽像素
    SCREEN_WIDTH =  glutGet(GLUT_SCREEN_WIDTH);
    //获取系统的高像素
    SCREEN_HEIGHT = glutGet(GLUT_SCREEN_HEIGHT);
    //创建窗口，窗口名字为3DFM
    glutCreateWindow("3DFM");
    //设置窗口大小
    glutReshapeWindow(windowWidth, windowHeight);
    //窗口居中显示
    glutPositionWindow((SCREEN_WIDTH-windowWidth)/2, (SCREEN_HEIGHT-windowHeight)/2);
    //设置窗口大小变化时的回调函数
    glutReshapeFunc(changeSize);
    //设置显示回调函数
//    glutDisplayFunc(render_screen);
    glutDisplayFunc(render_screen_finger_vein);
    //设置按键输入处理回调函数
    glutSpecialFunc(keyFunc);
    glutMouseFunc(mouseFunc);
    glutMotionFunc(mouseMotion);
    //设置全局渲染参数
    setupRenderingState();
    glutMainLoop();
}

