#pragma once
#include <iostream>
#include <cmath>
#include <ctime>
#include <string>
#include <vector>
#include <graphics.h>

using namespace std;

/**************全局变量*************/

constexpr auto Swidth = 600.0;  //窗口宽度
constexpr auto Sheight = 1200.0;  //高度
constexpr auto PI = 3.1415926535;
constexpr auto ShowCircle = false;    //默认不绘制轨迹
constexpr auto DELAYTIME = 20;    //间隔时间
constexpr auto ChangeTime = 1000;   //换挡时间

class Point
{
public:
	Point(const double &p_x, const double &p_y, const double &p_theta = 0.0, const double& p_R = 0.0);   //生成点
	void PointMove(const double& x_speed, const double& y_speed);    //点的移动
	void showPoint();
	void PointTurn(Point& center, const double& turn_speed);
	double DistanceTo(const Point& p) const;
	double thetaTo(const Point& p) const;    //对象点相对于p的角度
public:
	double x;
	double y;
	double theta = 0.0,R = 0.0;   //旋转角度、半径
	int r = 5;    //绘制半径
};

inline void Delay(const int& del)
{
	clock_t now = clock();          //记录当前时间
	while (clock() - now < del)    //延时等待
	{

	}
}

class Vec2d
{
public:
	Vec2d() = default;
	Vec2d(const double& new_x, const double& new_y, const bool& flag);    //两个值构造向量，标志位区分
	Vec2d(const Point& p_start, const Point& p_end);    //两点构造向量
	Vec2d(const double& length, const double& angle);   //长度和方向构造向量
	double length();  //模
	double crossProd(const Vec2d& other) const;  //叉乘
	double innerProd(const Vec2d& other) const;   //点乘
public:
	double x;
	double y;
};

inline double normalizeAngle(const double& theta)
{
	double theta_new = fmod(theta + PI, 2.0 * PI);
	if (theta_new < 0.0)
		theta_new += 2.0 * PI;
	return theta_new - PI;
}

inline void correctAngleError(double& target_theta, const double& delta_theta)
{
	if (delta_theta > 0)
	{
		if (target_theta > 0)  //逆时针
			target_theta -= delta_theta;
		else if (target_theta < 0)
			target_theta += delta_theta;
	}
}

inline double disPoint2Line(const Point& p, const Point& p_start, const Point& p_end)
{
	Vec2d line(p_start, p_end);
	Vec2d line_p(p_start, p);
	if (line.length() == 0.0)
		return line_p.length();
	return fabs(line.crossProd(line_p)) / line.length();
}

//double normalizeAngle(const double& theta);   //角度修正到-pi-pi
//void correctAngleError(double& target_theta, const double& delta_theta);   //角度误差修正
//double disPoint2Line(const Point& p, const Point& p_start, const Point& p_end);   //求点到直线距离
