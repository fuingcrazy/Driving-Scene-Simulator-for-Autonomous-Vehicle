#pragma once
#include <iostream>
#include <cmath>
#include <ctime>
#include <string>
#include <vector>
#include <graphics.h>

using namespace std;

/**************Global Variable*************/

constexpr auto Swidth = 600.0;  //Window width
constexpr auto Sheight = 1200.0;  //Window height
constexpr auto PI = 3.1415926535;
constexpr auto ShowCircle = false;    
constexpr auto DELAYTIME = 20;    //delay time
constexpr auto ChangeTime = 1000;   

class Point
{
public:
	Point(const double &p_x, const double &p_y, const double &p_theta = 0.0, const double& p_R = 0.0);   //Generate point
	void PointMove(const double& x_speed, const double& y_speed);    //point movement
	void showPoint();
	void PointTurn(Point& center, const double& turn_speed);
	double DistanceTo(const Point& p) const;
	double thetaTo(const Point& p) const;    //theta to another point
public:
	double x;
	double y;
	double theta = 0.0,R = 0.0;   
	int r = 5;    //draw radius
};

inline void Delay(const int& del)
{
	clock_t now = clock();          //record present time
	while (clock() - now < del)    //delay
	{

	}
}

class Vec2d
{
public:
	Vec2d() = default;
	Vec2d(const double& new_x, const double& new_y, const bool& flag);    //by x and y values
	Vec2d(const Point& p_start, const Point& p_end);    //by two points
	Vec2d(const double& length, const double& angle);   //by length and direction
	double length();  
	double crossProd(const Vec2d& other) const;  //Cross Product
	double innerProd(const Vec2d& other) const;   //Inner Product
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
		if (target_theta > 0)  
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


