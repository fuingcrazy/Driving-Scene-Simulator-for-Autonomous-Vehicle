#include "planning_base.h"

Point::Point(const double& p_x, const double& p_y, const double& p_theta, const double& p_R) :x(p_x), y(p_y), theta(p_theta), R(p_R)
{

}
void Point::PointMove(const double& x_speed, const double& y_speed)
{
	x += x_speed;
	y += y_speed;
}
void Point::showPoint()
{
	setfillcolor(BLACK);
	solidcircle(x, y, r);
}
void Point::PointTurn(Point &center, const double& turn_speed)
{
	theta += turn_speed;
	x = R * cos(theta)+center.x;
	y = -R * sin(theta) + center.y;   //注意这里的方向，向上为正方向
}
double Point::DistanceTo(const Point& p) const
{ 
	return hypot(x - p.x, y - p.y);
}

double Point::thetaTo(const Point& p) const
{
	if (x >= p.x && y == p.y)  //x正半轴
		return 0.0;
	else if (x < p.x && y == p.y)   //x负半轴
		return PI;
	else if (x == p.x && y > p.y)
		return -PI / 2.0;    //y负半轴
	else if (x == p.x && y < p.y)
		return PI / 2.0;
	else if (x > p.x)
		return -atan((y - p.y) / (x - p.x));
	else if (x < p.x)
		return PI - atan((y - p.y) / (x - p.x));
	return 0.0;
}

Vec2d::Vec2d(const double& new_x, const double& new_y, const bool& flag) : x(new_x), y(new_y)
{

}
Vec2d::Vec2d(const Point& p_start, const Point& p_end)  //两点构造向量
{
	x = p_end.x - p_start.x;
	y = -(p_end.y - p_start.y);
}
Vec2d::Vec2d(const double& length, const double& angle)  //长度方向构建向量
{
	x = length * cos(angle);
	y = length * sin(angle);
}
double Vec2d::length()
{
	return hypot(x, y);
}

double Vec2d::crossProd(const Vec2d& other) const   //叉乘
{
	return x * other.y - y * other.x;
}

double Vec2d::innerProd(const Vec2d& other)  const
{
	return x * other.x + y * other.y;
}