/*********************************************************************************
  *Copyright(C),2019-2021,西南交通大学HELIOS战队All Rights Reserved
  *FileName:  EnergyDetector.cpp
  *Author:  黄冬婷
  *Version: 1.2
  *Date:  2021.01.21
  *Description: 能量机关识别及预测
  *Function List:
     1.EnergyTask   能量机关任务执行接口
     2.getTargetPoint   获得目标点
     3.getPredictPoint    寻找攻击点
**********************************************************************************/

#include <fstream>
#include "EnergyDetector.h"

using namespace std;
extern McuData mcu_data;

Mat outline, debug_wave;
int debug_cur_count = 0;

static Mat polyfit(list<float> &in_point, int n) {
    int size = in_point.size();
    //所求未知数个数
    int x_num = n + 1;
    //构造矩阵U和Y
    Mat mat_u(size, x_num, CV_64F);
    Mat mat_y(size, 1, CV_64F);

    for (int i = 0; i < mat_u.rows; ++i)
        for (int j = 0; j < mat_u.cols; ++j) {
            mat_u.at<double>(i, j) = pow(i * 5, j);
        }

    int i = 0;
    list<float>::iterator it; //声明一个迭代器

    for (it = in_point.begin(); it != in_point.end(); it++) {
        mat_y.at<double>(i, 0) = static_cast<double>(*it);
        i++;
    }

    //矩阵运算，获得系数矩阵K
    Mat mat_k(x_num, 1, CV_64F);
    mat_k = (mat_u.t() * mat_u).inv() * mat_u.t() * mat_y;
    return mat_k;
}

/**
 * @brief EnergyDetector::EnergyDetector
 * @param null
 * @return null
 * @remark Energy类构造函数，初始化有关参数
 */
EnergyDetector::EnergyDetector() {
    initEnergy();
    initEnergyPartParam();//对能量机关参数进行初始化
}

/**
 * @brief EnergyDetector::EnergyDetectorDetector
 * @param null
 * @return null
 * @remark Energy类析构函数
 */
EnergyDetector::~EnergyDetector() = default;

/**
 * @brief EnergyDetector::initEnergy
 * @param null
 * @return null
 * @remark 初始化成员变量
 */
void EnergyDetector::initEnergy() {
    show_armors = true;//是否显示图像
    show_target_armor = false;//是否显示调试过程
    show_strip_fan = false;//是否显示灯扇
    show_center_R = true;//是否显示数据
    show_target_point = true;//是否显示目标点
    show_predict_point = true;//是否显示预测点

    last_target_polar_angle_judge_rotation = -1000;//上一帧待击打装甲板的极坐标角度（用于判断旋向）
    clockwise_rotation_init_cnt = 0;//装甲板顺时针旋转次数
    anticlockwise_rotation_init_cnt = 0;//装甲板逆时针旋转s次数
    energy_rotation_init = true;//若仍在判断风车旋转方向，则为true
    predict_rad = 0;//预测提前角
    predict_rad_norm = 25;// 预测提前角的绝对值
    predict_point = Point(0, 0);//预测打击点初始化
    pts.resize(4);
}

/**
 * @brief EnergyDetector::initEnergyPartParam
 * @param null
 * @return null
 * @remark 初始化参数
 */
void EnergyDetector::initEnergyPartParam() {
    _flow.BLUE_GRAY_THRESH = 100;//敌方红色时的阈值
    _flow.RED_GRAY_THRESH = 180;//敌方蓝色时的阈值

    _flow.armor_contour_area_max = 3000; //装甲板的相关筛选参数
    _flow.armor_contour_area_min = 1200;
    _flow.armor_contour_length_max = 100;
    _flow.armor_contour_length_min = 40;
    _flow.armor_contour_width_max = 60;
    _flow.armor_contour_width_min = 20;
    _flow.armor_contour_hw_ratio_max = 3;
    _flow.armor_contour_hw_ratio_min = 1;

    _flow.flow_strip_fan_contour_area_max = 6200;//流动条所在扇叶的相关筛选参数
    _flow.flow_strip_fan_contour_area_min = 4400;
    _flow.flow_strip_fan_contour_length_max = 200;
    _flow.flow_strip_fan_contour_length_min = 120;
    _flow.flow_strip_fan_contour_width_max = 110;
    _flow.flow_strip_fan_contour_width_min = 40;
    _flow.flow_strip_fan_contour_hw_ratio_max = 2.8;
    _flow.flow_strip_fan_contour_hw_ratio_min = 1.2;
    _flow.flow_strip_fan_contour_area_ratio_max = 0.55;
    _flow.flow_strip_fan_contour_area_ratio_min = 0.35;

    _flow.Strip_Fan_Distance_max = 56;//流动条到装甲板距离参数
    _flow.Strip_Fan_Distance_min = 28;

    _flow.flow_strip_contour_area_max = 700;//流动条相关参数筛选
    _flow.flow_strip_contour_area_min = 50;
    _flow.flow_strip_contour_length_max = 55;
    _flow.flow_strip_contour_length_min = 40;//32
    _flow.flow_strip_contour_width_max = 20;
    _flow.flow_strip_contour_width_min = 4;
    _flow.flow_strip_contour_hw_ratio_min = 3;
    _flow.flow_strip_contour_hw_ratio_max = 7;
    _flow.flow_strip_contour_area_ratio_min = 0.6;
    _flow.flow_strip_contour_intersection_area_min = 100;

    _flow.target_intersection_contour_area_min = 40;//重合面积

    _flow.twin_point_max = 20;

    _flow.Center_R_Control_area_max = 80;//中心R标筛选相关参数
    _flow.Center_R_Control_area_min = 30;
    _flow.Center_R_Control_length_max = 20;
    _flow.Center_R_Control_length_min = 6;
    _flow.Center_R_Control_width_max = 20;
    _flow.Center_R_Control_width_min = 6;
    _flow.Center_R_Control_radio_max = 1.2;
    _flow.Center_R_Control_radio_min = 1;
    _flow.Center_R_Control_area_radio_min = 0.6;
    _flow.Center_R_Control_area_intersection_area_min = 10;

    _flow.flow_area_max = 5000;//扇叶筛选相关参数
    _flow.flow_area_min = 1500;
    _flow.flow_length_max = 100;
    _flow.flow_length_min = 45;
    _flow.flow_width_max = 52;
    _flow.flow_width_min = 10;
    _flow.flow_aim_max = 3.5;
    _flow.flow_aim_min = 1.2;
    _flow.flow_area_ratio_min = 0.6;
}

/**
 * @brief EnergyDetector::clearAll
 * @param null
 * @return null
 * @remark 在每帧任务开始前清空容器
 */
void EnergyDetector::clearAll() {
    armors.clear();
    flow_strip_fans.clear();
    target_blades.clear();
    centerRs.clear();
    armor_centers.clear();
}

/**
 * @brief EnergyDetector::EnergyTask
 * @param Mat& src
 * @return null
 * @remark 能量机关任务执行接口
 */
void EnergyDetector::EnergyTask(Mat src, bool mode, int64 time_stamp) {
    clearAll();

    Mat binary;
    BIG_MODE = mode;

    frame_time = time_stamp;

    Mat img = src.clone();
    binary = preprocess(img);

    if (detectArmor(binary) && detectFlowStripFan(binary) && getTargetPoint(binary)) {
        getPredictPoint(img);

        getPts(target_armor);
    }


    //detectR(binary,img);

//    if(target_armor_centers.size() == 3)
//    {
//        calR();
//        target_armor_centers.clear();
//    }

    imshow("outline", outline);
	imshow("debug_wave", debug_wave);
}

/**
 * @brief EnergyDetector::preprocess
 * @param Mat& src
 * @return Mat& binary
 * @remark 图像预处理，完成二值化
 */

Mat EnergyDetector::preprocess(Mat &src) {
    Mat dst, binary;
    cvtColor(src, dst, COLOR_BGR2GRAY);

    Mat single;
    vector<Mat> channels;

    split(src, channels);

    if (blueTarget) {
        single = channels.at(0);
    } else {
        single = channels.at(2);
    }

    threshold(single, binary, 90, 255, THRESH_BINARY);

    Mat element_dilate_1 = getStructuringElement(MORPH_RECT, Size(7, 7));
    dilate(binary, binary, element_dilate_1);
    morphologyEx(binary, binary, MORPH_CLOSE, element_dilate_1);
    threshold(binary, binary, 0, 255, THRESH_BINARY);

    return binary;
}

/**
 * @brief EnergyDetector::detectBuff
 * @param Mat& src
 * @return null
 * @remark 检测所有可能的装甲
 */
bool EnergyDetector::detectArmor(Mat &src) {
    //armor dilate
    Mat armor_dilate = src.clone();

    imshow("armor_dilate", armor_dilate);
    //寻找所有装甲
    std::vector<vector<Point> > armor_contours;
	std::vector<RotatedRect> valid_armor_contours;
    std::vector<vector<Point> > armor_contours_external;//用总轮廓减去外轮廓，只保留内轮廓，除去流动条的影响。
    findContours(armor_dilate, armor_contours, RETR_LIST, CHAIN_APPROX_NONE);
    findContours(armor_dilate, armor_contours_external, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    //绘制轮廓
    outline = Mat(src.size(), CV_8UC3, Scalar(0, 0, 0));
	debug_wave = imread("../debug_wave.jpg");

    /*
       //绘制内轮廓
       for(auto &contours : armor_contours)
       {
           for (int i = 0; i < contours.size(); i++)//去除外轮廓
           {
               line(outline, contours[i], contours[(i + 1)%(contours.size())],
                    Scalar(0, 255, 255), 2, LINE_8);
           }
       }

   //绘制外轮廓
       for(auto &contours : armor_contours_external)
       {
           for (int i = 0; i < contours.size(); i++)//去除外轮廓
           {
               line(outline, contours[i], contours[(i + 1)%(contours.size())],
                    Scalar(0, 0, 255), 2, LINE_8);
           }
       }
   */

    for (auto &i : armor_contours_external)//去除外轮廓
    {
        auto external_contour_size = i.size();

        for (auto &j : armor_contours) {

            auto all_size = j.size();

            if (external_contour_size == all_size) {
                swap(j, armor_contours[armor_contours.size() - 1]);
                armor_contours.pop_back();//清除掉流动条
                break;
            }

        }

    }

    for (auto &armor_contour : armor_contours) {

        if (!isValidArmorContour(armor_contour)) {
            continue;
        }
		RotatedRect flow_rect = cv::minAreaRect(armor_contour);
		valid_armor_contours.push_back(flow_rect);
		Point2f flow_pts[4];
        flow_rect.points(flow_pts);

        for (int i = 0; i < 4; i++) {
            line(outline, flow_pts[i], flow_pts[(i + 1) % (4)],
                 Scalar(255, 255, 255), 2, LINE_8);
        }

        armors.emplace_back(cv::minAreaRect(armor_contour));//回传所有装甲板到armors容器中
        armor_centers.emplace_back(cv::minAreaRect(armor_contour).center);//回传所有装甲板center到armor_center容器中
    }
	
	//Debugging: 7.28 from XRZ
	//1. When there are 5 valid armor, calculate center_of_fan.
	//2. Calculate R.
	//3. Calculate distances between adjacent armors, that is, find for armor_i, the nearest armor of it and count the distance in for the answer.(adjacent_distance)
	//Result(1024*820): center_of_fan, R = (double)168.5, adjacent_distance = (double)196.5, all of which are distances in camera coordinates.
	if (valid_armor_contours.size() == 5) {
		double tx = 0, ty = 0, R = 0;
		for (int i = 0; i < valid_armor_contours.size(); i++) {
			putText(outline, to_string(i), valid_armor_contours[i].center, cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 1, 8);
			tx += valid_armor_contours[i].center.x / 5.0; ty += valid_armor_contours[i].center.y / 5.0;
		}
		Point2f center_of_fan(tx, ty);
		circle(outline, center_of_fan, 2, Scalar(0, 255, 0), -1);
		for (int i = 0; i < valid_armor_contours.size(); i++) {
			R += pointDistance(center_of_fan, valid_armor_contours[i].center) / 5;
		}
		//LOGE("Radius: %.2lf", R);
		double adjacent_distance = 0;
		for (int i = 0; i < valid_armor_contours.size(); i++) {
			double temp_distance = 7777;
			for (int j = 0; j < valid_armor_contours.size(); j++) {
				if (i == j) continue;
				temp_distance = min(temp_distance, pointDistance(valid_armor_contours[i].center, valid_armor_contours[j].center));
			}
			adjacent_distance += temp_distance / 5;
 		}
		//LOGE("Adjacent distance: %.2lf", adjacent_distance);
	}

    if (armors.empty()) return false;

    return true;

}

/**
 * @brief EnergyDetector::detectFlowStripFan
 * @param Mat& src
 * @return null
 * @remark 检测所有可能的流动条所在的装甲板
 */
bool EnergyDetector::detectFlowStripFan(Mat &src) {
    //flow_strip_fan dilate

    Mat flow_fan_dilate = src.clone();

    //寻找所有流动条所在扇叶
    vector<vector<Point> > flow_strip_fan_contours;
    findContours(flow_fan_dilate, flow_strip_fan_contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    vector<cv::RotatedRect> candidate_flow_strip_fans;

    debug_cur_count = 0;
    for (auto &flow_strip_fan_contour : flow_strip_fan_contours) {

        if (!isValidFlowStripFanContour(flow_fan_dilate, flow_strip_fan_contour)) {
            continue;
        }
        RotatedRect flow_rect = cv::minAreaRect(flow_strip_fan_contour);
        Point2f flow_pts[4];
        flow_rect.points(flow_pts);

        for (int i = 0; i < 4; i++) {
            line(outline, flow_pts[i], flow_pts[(i + 1) % (4)],
                 Scalar(255, 255, 255), 2, LINE_8);
        }

        flow_strip_fans.emplace_back(cv::minAreaRect(flow_strip_fan_contour));
    }

    if (flow_strip_fans.empty())
        return false;

    return true;

#if DEBUG_MSG == 1
    if (flow_strip_fans.empty()) {
        LOGM("flow strip fan false!\n");
    }
    if (!flow_strip_fans.empty()) {
        LOGM("flow strip fan success!\n");
    }
#endif

}

/**
 * @brief EnergyDetector::detectR
 * @param Mat& src
 * @return null
 * @remark 检测所有可能的中心R，并筛选出唯一中心点centerR，确定circle_center_point
 */
bool EnergyDetector::detectR(Mat &src, Mat &show) {
    //R dilate
    Mat R_dilate = src.clone();
    Mat gray_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
    Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    Mat hsv;
    Mat mask;
    cvtColor(show, hsv, COLOR_RGB2HSV);
    inRange(hsv, Scalar(0, 10, 46), Scalar(180, 60, 255), mask);
    dilate(mask, mask, gray_element);

    //imshow("R_dilate1",R_dilate);
    //imshow("mask",mask);
    R_dilate = R_dilate - mask;
    dilate(R_dilate, R_dilate, gray_element);
    erode(R_dilate, R_dilate, element);

    //imshow("R_dilate2",R_dilate);
    //todo find R center
    vector<vector<Point> > center_R_contours;
    findContours(R_dilate, center_R_contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    for (auto &center_R_contour : center_R_contours) {
        if (!isValidCenterRContour(center_R_contour)) {
            continue;
        }

        RotatedRect tmp_R = cv::minAreaRect(center_R_contour);

        for (auto &flow_strip_fan : flow_strip_fans) {
            std::vector<cv::Point2f> intersection;
            if (rotatedRectangleIntersection(flow_strip_fan, tmp_R, intersection) != 0) {
                inter_flag = true;
                break;
            }
        }

        if (!inter_flag) {
            centerRs.emplace_back(cv::minAreaRect(center_R_contour));
        }
    }
    //给出唯一centerR
    if (!centerRs.empty()) {
        if (centerRs.size() == 1) {
            centerR = centerRs[0];
            pre_centerR = centerR;
        } else {
            //计算前后两帧中心点距离
            int *dis = (int *) malloc(centerRs.size() * sizeof(int));

            memset(dis, 0, centerRs.size() * sizeof(int));

            for (int i = 0; i < centerRs.size(); i++) {
                *(dis + i) += abs(pre_centerR.center.x - centerRs[i].center.x);
                *(dis + i) += abs(pre_centerR.center.y - centerRs[i].center.y);
            }

            int min_dis = *dis;
            int min_index = 0;

            for (int t = 1; t < centerRs.size(); t++) {
                if (*(dis + t) < min_dis) {
                    min_dis = *(dis + t);
                    min_index = t;
                }
            }
            centerR = centerRs[min_index];
            float center_ratio = centerRs[min_index].size.height / centerRs[min_index].size.width;
            if ((center_ratio > 1.2) || (center_ratio < 0.75))
                centerR = pre_centerR;
            free(dis);
            pre_centerR = centerR;//update pre
        }
    } else
        centerR = pre_centerR;

    circle_center_point = centerR.center;
}

void EnergyDetector::calR() {
    Point2f one = target_armor_centers[0];
    Point2f two = target_armor_centers[1];
    Point2f three = target_armor_centers[2];
    float a1 = 2 * (two.x - one.x);
    float b1 = 2 * (two.y - one.y);
    float c1 = pow(two.x, 2) + pow(two.y, 2) - pow(one.x, 2) - pow(one.y, 2);
    float a2 = 2 * (three.x - two.x);
    float b2 = 2 * (three.y - two.y);
    float c2 = pow(three.x, 2) + pow(three.y, 2) - pow(two.x, 2) - pow(two.y, 2);

    circle_center_point.x = ((c1 * b2) - (c2 * b1)) / ((a1 * b2) - (a2 * b1));
    circle_center_point.y = ((a1 * c2) - (a2 * c1)) / ((a1 * b2) - (a2 * b1));

}

/**
 * @brief EnergyDetector::getTargetPoint
 * @param Mat& src
 * @return null
 * @remark 根据armor与flowStripFan情况判断给出唯一目标装甲板target_armor以及其中心点target_point
 */
bool EnergyDetector::getTargetPoint(Mat &src) {
    //todo find armor in best_fan 目标armor
    for (int i = 0; i < flow_strip_fans.size(); i++) {
        //为target_armors打分
        for (int j = 0; j < armors.size(); j++) {
            std::vector<cv::Point2f> intersection;
            if (rotatedRectangleIntersection(armors[j], flow_strip_fans[i], intersection) == 0)
                continue;
            double cur_contour_area = contourArea(intersection);
            if (cur_contour_area > _flow.target_intersection_contour_area_min) {
                target_blades.emplace_back(Blade_(i, j));
            }//返回目标装甲板参数
        }
    }

    if (!target_blades.empty()) {
        if (target_blades.size() == 1) {
            target_blade = target_blades[0];
        } else {
            //为target_armors打分
            auto *grade = (double *) malloc(target_blades.size() * sizeof(double));
            memset(grade, 0, target_blades.size() * sizeof(double));

            for (int i = 0; i < target_blades.size(); i++) {
                *(grade + i) += pointDistance(armors[target_blades[i].armor_index].center,
                                              lst_target_armor.center);      //距离远的为新的待打击装甲
            }

            double max_grade = *grade;
            int max_index = 0;

            for (int t = 0; t < target_blades.size(); t++) {
                if (*(grade + t) < max_grade) {
                    max_grade = *(grade + t);
                    max_index = t;
                }
            }

            free(grade);

            target_blade = target_blades[max_index];
        }

        target_armor = armors[target_blade.armor_index];
        lst_target_armor = target_armor;        //update
    } else {
        return false;
    }

    target_point = target_armor.center;

    //计算中心R标所在区域
    int center_blade_edge_index;
    int max_length = 0;
    Point2f blade_points[4];
    flow_strip_fans[target_blade.flow_strip_fan_index].points(blade_points);

    for (int i = 0; i < 4; i++) {
        float cur_getTargetPoint_length = pointDistance((blade_points[i] + blade_points[(i + 1) % 4]) / 2.0,
                                                        target_armor.center);
        if (cur_getTargetPoint_length > max_length) {
            max_length = cur_getTargetPoint_length;
            center_blade_edge_index = i;
        }
    }

    Point2f fan_center_diff_armor_center =
            flow_strip_fans[target_blade.flow_strip_fan_index].center - target_armor.center;
    Point2f origin_center_blade_edge =
            blade_points[(center_blade_edge_index + 1) % 4] - blade_points[center_blade_edge_index];
    Point2f vertical_center_blade_edge;

    vertical_center_blade_edge.x = -origin_center_blade_edge.y;
    vertical_center_blade_edge.y = origin_center_blade_edge.x;

    if (fan_center_diff_armor_center.dot(vertical_center_blade_edge) < 0) {
        vertical_center_blade_edge *= -1.0;
    }

    Point2f center_point = (blade_points[(center_blade_edge_index + 1) % 4] + blade_points[center_blade_edge_index]) / 2
                           + vertical_center_blade_edge * 0.5;

    center_r_area = Rect(center_point - Point2f(max_length / 2, max_length / 2), Size(max_length, max_length));

    //just for detectR not working
    circle_center_point = center_point;

    rectangle(outline, center_r_area, Scalar(255, 255, 255), 2);
    circle(outline, center_point, 3, Scalar(0, 255, 255), 3);
    circle(outline, target_point, 2, Scalar(0, 255, 0), 3);
    line(outline, blade_points[(center_blade_edge_index + 1) % 4], blade_points[center_blade_edge_index],
         Scalar(0, 255, 255), 3);

    //虽然我明白target_armor_centers是存储历史中心点的，但是这个条件没看懂
    if (!target_armor_centers.empty() && target_armor_centers.size() < 3 &&
        pointDistance(target_point, target_armor_centers[target_armor_centers.size() - 1]) > 60) {
        target_armor_centers.push_back(target_point);
    } else if (target_armor_centers.empty())
        target_armor_centers.push_back(target_point);

    return true;
}


/**
 * @brief EnergyDetector::getPredictPoint
 * @param Mat src
 * @return null
 * @remark 根据运动曲线规律给出预测射击点位置predict_point
 */

void EnergyDetector::getPredictPoint(Mat src) {
    target_polar = toPolar(target_point, frame_time);

    LOGE("Radius : %d", target_polar.radius);

    history_target_armor_polar.push_back(target_polar);

    if (history_target_armor_polar.size() == 7) {
        history_target_armor_polar.pop_front();
    }

    if (history_target_armor_polar.size() < 6) {
        predict_polar = target_polar;
    } else {
        int list_traverse_count = 0;
        float last_avg_angle = 0;
        double last_avg_time = 0;

        //选取前节点

        for (auto it = history_target_armor_polar.begin(); list_traverse_count < 3; it++, list_traverse_count++) {
            last_avg_angle += (*it).angle / 3;
            last_avg_time += (*it).time_stamp / 3;
        }

        list_traverse_count = 0;
        float cur_avg_angle = 0;
        double cur_avg_time = 0;

        //选取后节点
        for (auto it = history_target_armor_polar.rbegin(); list_traverse_count < 3; it++, list_traverse_count++) {
            cur_avg_angle += (*it).angle / 3;
            cur_avg_time += (*it).time_stamp / 3;
        }

        //计算中间节点角速度及时间
        double angular_velocity = 0;
        double cur_middle_time, time_middle_last;
        cur_middle_time = (cur_avg_time / 2 + last_avg_time / 2);
        angular_velocity = (cur_avg_angle - last_avg_angle) / ((cur_avg_time - last_avg_time) / getTickFrequency());

        median_filter.insert(angular_velocity);
        angular_velocity = median_filter.get_middle_value();

        time_middle_last = (history_target_armor_polar.back().time_stamp - cur_middle_time) / getTickFrequency();

        LOGE("getTickFrequency : %lf", getTickFrequency());

        //计算当前旋转角加速度方向
        auto it = history_target_armor_polar.rbegin();
        vector<float> history_angular_velocity;

        double angular_acceleration_symbol = 0;
        double angular_direction_symbol = 0;

        auto last_polar_point = it;
        it++;

        for (; it != history_target_armor_polar.rend(); it++) {
            double cur_ = last_polar_point->angle - it->angle;
            angular_direction_symbol += cur_;

            history_angular_velocity.push_back(fabs(cur_));
            last_polar_point = it;
        }

        for (int i = 0; i < history_angular_velocity.size() - 1; i++) {
            angular_acceleration_symbol += history_angular_velocity[i] - history_angular_velocity[i + 1];
        }

        if ((angular_velocity - 1.305) / 0.785 > 1)
            angular_velocity = 1.305 + 0.785;
        else if ((angular_velocity - 1.305) / 0.785 < -1)
            angular_velocity = 1.305 - 0.78;

        double spd_time_1 = asin(((angular_velocity - 1.305) / 0.785));

        if (angular_acceleration_symbol > 0 && angular_velocity > 1.305)
            spd_time_1 = 6.283 - spd_time_1;
        else if (angular_acceleration_symbol < 0 && angular_velocity < 1.305)
            spd_time_1 = -6.283 - spd_time_1;

        spd_time_1 /= 1.884;

        double spd_integral_1 =
                1.305 * (spd_time_1 + time_middle_last) - 0.416667 * cos(1.884 * (spd_time_1 + time_middle_last));
        double spd_integral_2 = 1.305 * (spd_time_1 + time_middle_last + 0.5) - 0.416667 *
                                                                                cos(1.884 *
                                                                                    (spd_time_1 + time_middle_last +
                                                                                     0.5));

        double angle_diff = spd_integral_2 - spd_integral_1;

        predict_polar.angle = target_polar.angle + 1 * angle_diff;

        predict_polar.radius = target_polar.radius;

        Point2f predict_center = toCartesian(predict_polar);
        LOGE("cur_avg_angle : %f, last_avg_angle  : %f, Time : %f", cur_avg_angle * 57.3, last_avg_angle * 57.3,
             time_middle_last + 0.005);
        LOGE("Velocity : %lf, Direction Symbol : %lf, Acceleration Symbol : %lf", angular_velocity,
             angular_direction_symbol, angular_acceleration_symbol);
        LOGE("angle_diff : %lf", angle_diff * 57.3);

        line(outline, predict_center, circle_center_point, Scalar(0, 0, 255), 2);
        circle(outline, predict_center, 5, Scalar(255, 255, 255), -1);
        line(debug_wave, Point2f(50 * (spd_time_1 + time_middle_last), 0),
             Point2f(50 * (spd_time_1 + time_middle_last), 799), Scalar(255, 255, 255), 2);
    }
}
/**
 * @remark: 在预测位置画出待打击装甲板
 */
void EnergyDetector::getPredictRect() {
    Point2f rectPoints[4];//定义矩形的4个顶点
    target_armor.points(rectPoints); //计算矩形的4个顶点

    //judge long side
    if (sqrt(pow((rectPoints[0].x - rectPoints[1].x), 2) + pow((rectPoints[0].y - rectPoints[1].y), 2))
        > sqrt(pow((rectPoints[2].x - rectPoints[1].x), 2) + pow((rectPoints[2].y - rectPoints[1].y), 2))) {
        //pts[0]-pts[1] is long side
        pts[0] = rectPoints[0];
        pts[1] = rectPoints[1];
        pts[2] = rectPoints[2];
        pts[3] = rectPoints[3];
    } else {
        //pts[1]-pts[2] is long side
        pts[0] = rectPoints[1];
        pts[1] = rectPoints[2];
        pts[2] = rectPoints[3];
        pts[3] = rectPoints[0];
    }

    polarLocal cur_polar_point;

    for (int i = 0; i < 4; i++) {
        cur_polar_point = toPolar(pts[i], 0.0);
        cur_polar_point.angle += (predict_polar.angle - target_polar.angle);
        pts[i] = toCartesian(cur_polar_point);
    }

    for (int i = 0; i < 4; i++) {
        line(outline, pts[i], pts[(i + 1) % (4)],
             Scalar(203, 192, 255), 2, LINE_8);
    }

}

/**
 * @brief EnergyDetector::initRotation
 * @param null
 * @return null
 * @remark 用于判断能量机关旋转方向
 */
void EnergyDetector::initRotation() {
    if (target_polar_angle >= -180 && last_target_polar_angle_judge_rotation >= -180
        && fabs(target_polar_angle - last_target_polar_angle_judge_rotation) < 30) {
        //target_polar_angle和last_target_polar_angle_judge_rotation的初值均为1000，大于-180表示刚开始几帧不要
        //若两者比较接近，则说明没有切换目标，因此可以用于顺逆时针的判断
        if (target_polar_angle < last_target_polar_angle_judge_rotation) clockwise_rotation_init_cnt++;
        else if (target_polar_angle > last_target_polar_angle_judge_rotation) anticlockwise_rotation_init_cnt++;
    }
    //由于刚开始圆心判断不准，角度变化可能计算有误，因此需要在角度正向或逆向变化足够大时才可确定是否为顺逆时针
    if (clockwise_rotation_init_cnt == 15) {
        energy_rotation_direction = 1;//顺时针变化30次，确定为顺时针
        //cout << "rotation: " << energy_rotation_direction << endl;
        energy_rotation_init = false;
    } else if (anticlockwise_rotation_init_cnt == 15) {
        energy_rotation_direction = -1;//逆时针变化30次，确定为顺时针
        //cout << "rotation: " << energy_rotation_direction << endl;
        energy_rotation_init = false;
    }
    last_target_polar_angle_judge_rotation = target_polar_angle;
}


/**
 * @brief EnergyDetector::calPreAngle
 * @param float start_time, float end_time
 * @return float delta_angle
 * @remark 根据正弦速度曲线以及总射击时间给出射击提前角
 */
float EnergyDetector::calPreAngle(float start_time, float end_time) {
    float delta_angle;
    delta_angle = 0.785 * (cos(OMEGA * start_time) - cos(OMEGA * end_time) + 1.305 * (end_time - start_time));
    return delta_angle;
}

/**
 * @brief EnergyDetector::rotate
 * @param Point target_point
 * @return Point trans_point
 * @remark 计算预测的击打点坐标 todo 后续用toCartesian 替代
 */
Point EnergyDetector::rotate(cv::Point target_point) const {
    int x1, x2, y1, y2;
    Point trans_point;
    //    为了减小强制转换的误差
    x1 = circle_center_point.x * 100;
    x2 = target_point.x * 100;
    y1 = circle_center_point.y * 100;
    y2 = target_point.y * 100;

    trans_point.x = static_cast<int>(
            (x1 + (x2 - x1) * cos(-predict_rad * 3.14 / 180.0) - (y1 - y2) * sin(-predict_rad * 3.14 / 180.0)) / 100);
    trans_point.y = static_cast<int>(
            (y1 - (x2 - x1) * sin(-predict_rad * 3.14 / 180.0) - (y1 - y2) * cos(-predict_rad * 3.14 / 180.0)) / 100);

    return trans_point;
}


/**
 * @brief EnergyDetector::isValidCenterRContour
 * @param vector<Point>& center_R_contour
 * @return bool
 * @remark 判断找到的中心点R尺寸是否合格
 */
bool EnergyDetector::isValidCenterRContour(const vector<cv::Point> &center_R_contour) {
    double cur_contour_area = contourArea(center_R_contour);
    if (cur_contour_area > _flow.Center_R_Control_area_max ||
        cur_contour_area < _flow.Center_R_Control_area_min) {
        return false;
    }
    return true;
}

/**
 * @brief EnergyDetector::isValidArmorContour
 * @param vector<Point>& armor_contour
 * @return bool
 * @remark 判断找到的装甲Armor尺寸是否合格
 */
bool EnergyDetector::isValidArmorContour(const vector<cv::Point> &armor_contour) const {
    double cur_contour_area = contourArea(armor_contour);
    LOGW("============Vaild Armor=================");
    LOGW("Count : %d\t Area : %lf", debug_cur_count++, cur_contour_area);
    if (cur_contour_area > _flow.armor_contour_area_max ||
        cur_contour_area < _flow.armor_contour_area_min) {
        return false;
    }

    RotatedRect cur_rect = minAreaRect(armor_contour);
    Size2f cur_size = cur_rect.size;
    float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
    float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
    LOGW("Length : %f\t Width : %f", length, width);
    if (length < _flow.armor_contour_length_min || width < _flow.armor_contour_width_min ||
        length > _flow.armor_contour_length_max || width > _flow.armor_contour_width_max) {
        return false;
    }

    float length_width_ratio = length / width;

    if (length_width_ratio > _flow.armor_contour_hw_ratio_max ||
        length_width_ratio < _flow.armor_contour_hw_ratio_min) {
        return false;
    }

    return true;
}


/**
 * @brief EnergyDetector::isValidFlowStripFanContour
 * @param vector<Point>& flow_strip_fan_contour
 * @return bool
 * @remark 判断找到的含有流动条的扇叶尺寸是否合格
 */
bool EnergyDetector::isValidFlowStripFanContour(cv::Mat &src, const vector<cv::Point> &flow_strip_fan_contour) const {
    double cur_contour_area = contourArea(flow_strip_fan_contour);
    LOGW("============Vaild Flow=================");
    LOGW("Area : %lf", cur_contour_area);
    if (cur_contour_area > _flow.flow_strip_fan_contour_area_max ||
        cur_contour_area < _flow.flow_strip_fan_contour_area_min) {
        return false;
    }

    RotatedRect cur_rect = minAreaRect(flow_strip_fan_contour);
    Size2f cur_size = cur_rect.size;

    float length = cur_size.height > cur_size.width ? cur_size.height : cur_size.width;
    float width = cur_size.height < cur_size.width ? cur_size.height : cur_size.width;
    LOGW("Length : %f\t Width : %f", length, width);
    if (length < _flow.flow_strip_fan_contour_length_min
        || width < _flow.flow_strip_fan_contour_width_min
        || length > _flow.flow_strip_fan_contour_length_max
        || width > _flow.flow_strip_fan_contour_width_max) {
        return false;
    }

    float length_width_ratio = length / width;
    LOGW("Ratio : %f", length_width_ratio);
    LOGW("Rect Size : %f\t Contour Size : %lf", cur_size.area(), cur_contour_area);
    if (length_width_ratio > _flow.flow_strip_fan_contour_hw_ratio_max ||
        length_width_ratio < _flow.flow_strip_fan_contour_hw_ratio_min) {
        return false;
    }

    if (cur_contour_area / cur_size.area() < _flow.flow_strip_fan_contour_area_ratio_min
        || cur_contour_area / cur_size.area() > _flow.flow_strip_fan_contour_area_ratio_max) {
        return false;
    }

    return true;
}


//todo 工具类函数
/**
 * @brief EnergyDetector::pointDistance
 * @param Point point_1,Point point_2
 * @return double distance
 * @remark 输入两点，返回两点间距离
 */
double EnergyDetector::pointDistance(cv::Point point_1, cv::Point point_2) {
    double distance = 0;
    distance = sqrt(
            pow(static_cast<double>(point_1.x - point_2.x), 2) + pow(static_cast<double>(point_1.y - point_2.y), 2));
    return distance;
}

/**
 * @brief EnergyDetector::toPolar
 * @param Point cart
 * @return polar trans
 * @remark 输入世界坐标系坐标，输出极坐标系坐标
 */
polarLocal EnergyDetector::toPolar(Point cart, double time_stamp) {
    polarLocal trans;
    trans.angle = atan2((-1 * (cart.y - circle_center_point.y)), (cart.x - circle_center_point.x));

    if (trans.angle < 0)
        trans.angle += 2 * 3.1416;

    trans.radius = pointDistance(cart, circle_center_point);
    trans.time_stamp = time_stamp;
    return trans;
}

/**
 * @brief EnergyDetector::toCartesian
 * @param polarLocal pol
 * @return Point trans
 * @remark 输入极坐标系坐标，输出世界坐标系坐标
 */
Point2f EnergyDetector::toCartesian(polarLocal pol) {
    LOGE("pol Angle : %lf, pol radius : %d", pol.angle, pol.radius);
    double delta_x = cos(pol.angle) * pol.radius;    //R*cos = x2-x1
    double delta_y;
    delta_y = -1.0 * sin(pol.angle) * pol.radius;   //(y2-y1)^2 = sqrt(radius^2-(x2-x1)^2)
    LOGE("Delta X %lf\tDelta Y %lf", delta_x, delta_y);
    Point2f trans;
    trans.x = circle_center_point.x + delta_x;
    trans.y = circle_center_point.y + delta_y;
    return trans;
}

void EnergyDetector::getPts(RotatedRect armor) {
    Point2f rectPoints[4];//定义矩形的4个顶点
    armor.points(rectPoints); //计算矩形的4个顶点

    //judge long side
    if (sqrt(pow((rectPoints[0].x - rectPoints[1].x), 2) + pow((rectPoints[0].y - rectPoints[1].y), 2))
        > sqrt(pow((rectPoints[2].x - rectPoints[1].x), 2) + pow((rectPoints[2].y - rectPoints[1].y), 2))) {
        //pts[0]-pts[1] is long side
        pts[0] = rectPoints[0];
        pts[1] = rectPoints[1];
        pts[2] = rectPoints[2];
        pts[3] = rectPoints[3];
    } else {
        //pts[1]-pts[2] is long side
        pts[0] = rectPoints[1];
        pts[1] = rectPoints[2];
        pts[2] = rectPoints[3];
        pts[3] = rectPoints[0];
    }
}

Point EnergyDetector::getPredict() {
    return predict_point;
}

Point EnergyDetector::getOffset() {
    return predict_point - target_point;
}

