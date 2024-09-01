#pragma once
#include <iostream>
#include <cmath>
#include <ctime>
#include <string>
#include <vector>
#include <graphics.h>

using namespace std;

/**************ȫ�ֱ���*************/

constexpr auto Swidth = 600.0;  //���ڿ��
constexpr auto Sheight = 1200.0;  //�߶�
constexpr auto PI = 3.1415926535;
constexpr auto ShowCircle = false;    //Ĭ�ϲ����ƹ켣
constexpr auto DELAYTIME = 20;    //���ʱ��
constexpr auto ChangeTime = 1000;   //����ʱ��

class Point
{
public:
	Point(const double &p_x, const double &p_y, const double &p_theta = 0.0, const double& p_R = 0.0);   //���ɵ�
	void PointMove(const double& x_speed, const double& y_speed);    //����ƶ�
	void showPoint();
	void PointTurn(Point& center, const double& turn_speed);
	double DistanceTo(const Point& p) const;
	double thetaTo(const Point& p) const;    //����������p�ĽǶ�
public:
	double x;
	double y;
	double theta = 0.0,R = 0.0;   //��ת�Ƕȡ��뾶
	int r = 5;    //���ư뾶
};

inline void Delay(const int& del)
{
	clock_t now = clock();          //��¼��ǰʱ��
	while (clock() - now < del)    //��ʱ�ȴ�
	{

	}
}

class Vec2d
{
public:
	Vec2d() = default;
	Vec2d(const double& new_x, const double& new_y, const bool& flag);    //����ֵ������������־λ����
	Vec2d(const Point& p_start, const Point& p_end);    //���㹹������
	Vec2d(const double& length, const double& angle);   //���Ⱥͷ���������
	double length();  //ģ
	double crossProd(const Vec2d& other) const;  //���
	double innerProd(const Vec2d& other) const;   //���
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
		if (target_theta > 0)  //��ʱ��
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

//double normalizeAngle(const double& theta);   //�Ƕ�������-pi-pi
//void correctAngleError(double& target_theta, const double& delta_theta);   //�Ƕ��������
//double disPoint2Line(const Point& p, const Point& p_start, const Point& p_end);   //��㵽ֱ�߾���
