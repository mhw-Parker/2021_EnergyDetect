#ifndef ENERGY_H
#define ENERGY_H

#include <iostream>
#include <cmath>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <utility.hpp>

#include "mydefine.h"
#include "log.h"

#include"struct_define.h"

using namespace std;
using namespace cv;

#define PI 3.14
#define SHOOT_TIME 1
#define FPS 60
#define OMEGA 1.884
#define MAX_ANGLE 75

typedef struct {
    int radius;
    float angle;
    double time_stamp;
} polarLocal;

typedef struct Blade_{
    int armor_index;
    int flow_strip_fan_index;
    Blade_(int i_, int j_)
    {
        flow_strip_fan_index = i_;
        armor_index = j_;
    }

    Blade_()
    {
        armor_index = 0;
        flow_strip_fan_index = 0;
    }
}Blade;

class EnergyDetector {
public:
    EnergyDetector();//构造函数
    ~EnergyDetector();//析构函数

    void EnergyTask(Mat src, bool mode, int64 time_stamp);//接口
    Point getPredict();
    Point getOffset();
    vector<Point2f> pts;
    cv::Point2f target_point;//目标装甲板中心坐标
    cv::Point2f predict_point;//预测的击打点坐标
    cv::Point2f circle_center_point;//风车圆心坐标
    std::vector<cv::Point2f> target_armor_centers;//get R
private:
    polarLocal polar_t;
    bool show_armors; //是否显示所有装甲
    bool show_target_armor; //是否显示目标装甲
    bool show_strip_fan;//是否显示有流动条的扇叶
    bool show_center_R;//是否显示中心点R
    bool show_target_point;//是否显示目标点
    bool show_predict_point;//是否显示预测点
    bool BIG_MODE = true;//是否为大符模式
    bool inter_flag = false;//是否contour有交集
    bool start_flag = false;//是否开始预测

    WindmillParamFlow _flow;
    McuData mcu_data;

    void clearAll();//清空所有容器vector
    void initEnergy();//能量机关初始化
    void initEnergyPartParam();//能量机关参数初始化
    Mat preprocess(Mat& src);

    bool detectArmor(Mat &src);
    bool detectFlowStripFan(Mat &src);
    bool detectR(Mat &src, Mat &show);
    void calR();
    bool getTargetPoint(Mat &src);

    bool isValidArmorContour(const vector<cv::Point>& armor_contour) const;//装甲板矩形尺寸要求
    bool isValidCenterRContour(const vector<cv::Point>& center_R_contour);//风车中心选区尺寸要求
    bool isValidFlowStripFanContour(cv::Mat& src, const vector<cv::Point>& flow_strip_fan_contour) const;//流动条扇叶矩形尺寸要求

    double pointDistance(cv::Point point_1, cv::Point point_2);//计算两点距离
    polarLocal toPolar(Point cart, double time_stamp);
    Point2f toCartesian(polarLocal pol);
    Point rotate(cv::Point target_point) const;
    float calPreAngle(float start_time,float end_time);
    void getPredictPoint(Mat src);
    void getPts(RotatedRect armor);
	void getPredictRect();

    std::vector<Blade> target_blades;//可能的目标装甲板
    std::vector<cv::RotatedRect> armors;//图像中所有可能装甲板
    std::vector<cv::RotatedRect> flow_strip_fans;//可能的流动扇叶
    std::vector<cv::RotatedRect> centerRs;//可能的中心
    std::vector<cv::Point2f> armor_centers;//用来做最小二乘拟合
    RMTools::MedianFilter<double> median_filter;//处理中位数

    cv::Rect center_r_area;
    cv::RotatedRect centerR;//风车中心字母R
    cv::RotatedRect pre_centerR;//风车中心字母R

    Blade target_blade;//目标扇叶
    RotatedRect target_armor;
    cv::RotatedRect lst_target_armor;//目标装甲板

    int energy_rotation_direction;//风车旋转方向
    int clockwise_rotation_init_cnt;//装甲板顺时针旋转次数
    int anticlockwise_rotation_init_cnt;//装甲板逆时针旋转次数
    bool energy_rotation_init;//若仍在判断风车旋转方向，则为true
    void initRotation();//对能量机关旋转方向进行初始化

    float predict_rad;//预测提前角
    float predict_rad_norm;//预测提前角的绝对值
    float target_polar_angle;//待击打装甲板的极坐标角度
    float last_target_polar_angle_judge_rotation;//上一帧待击打装甲板的极坐标角度（用于判断旋向）

    list<polarLocal> history_target_armor_polar;
    polarLocal predict_polar;
    polarLocal target_polar;
    //todo predict task

    int64 frame_time;

    //polarLocal target_polar;
    int small_rate = 60;    //小符60°/s = 1.05rad/s
    vector<int> target_angle_1second;//1s内每帧的目标点角度
    float v0_rate;
    float v1_rate;
    float t0;
    float t1;
    int64 t0_real;
    int64 t1_real;
    int tiao_armor_f = 0;
    float shoot_Debug = 0.2;


};

#endif //ENERGY_H

