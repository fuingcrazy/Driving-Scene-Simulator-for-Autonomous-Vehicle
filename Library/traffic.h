#pragma once
#include "planning_base.h"
#include "car.h"

class Cone
{
public:
	unique_ptr<Point> p_center;   //���ĵ�
	double r = 20.0;   //�뾶
public:
	~Cone() = default;
	Cone(const double& pos_x, const double& pos_y, const double& R = 20.00);
	void showCone();   //����׶Ͱ
};

class Person
{
public:
	Person() = default;
	Person(const double& pos_x, const double& pos_y);

	void PersonMove();
	void PersonDraw();
public:
	unique_ptr<Point> p_center;
	double r = 20.0;
	double speed = 0.0;
};