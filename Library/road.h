#pragma once
#include "traffic.h"

class RoadBase
{
public:
	virtual ~RoadBase() = default;
	virtual void showRoad() = 0;   //绘制道路,纯虚函数要求派生类必须定义具体实现形态
	virtual double getUpLine() { return 0.0; }  //父类也要写成虚函数
	virtual double getMidLine() { return 0.0; }
	virtual double getDownLine() { return 0.0; }
public:
	double Rwidth = 200.0;   //下边界
	double up_boundary = 0.0;   //上边界
	double down_boundary = 0.0;  //下边界
	double left_boundary = 0.0;  //左边界
	double right_boundary = 0.0;  //右边界
};

class RoadNormal :public RoadBase   //一般道路(继承基类road)
{
public:
	RoadNormal(const double& r_width = 200.0);
	void showRoad() override;
};

class RoadCross:public RoadBase   //斑马线道路
{
public:
	RoadCross(const double& r_width = 200.0);
	void showRoad() override;
	double getUpLine() { return this->up_line; }  //函数化参数
	double getMidLine() { return this->mid_line; }
	double getDownLine() { return this->down_line; }
public:
	double up_line = 0.0;
	double mid_line = 0.0;
	double down_line = 0.0;
	double disRec = 20.0;  //间距

};

class RoadDoubleLine :public RoadBase
{
public:
	RoadDoubleLine(const double& r_width = 200.0);
	void showRoad();

};