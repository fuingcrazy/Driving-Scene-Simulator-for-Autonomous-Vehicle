#pragma once
#include "traffic.h"

class RoadBase
{
public:
	virtual ~RoadBase() = default;
	virtual void showRoad() = 0;   //���Ƶ�·,���麯��Ҫ����������붨�����ʵ����̬
	virtual double getUpLine() { return 0.0; }  //����ҲҪд���麯��
	virtual double getMidLine() { return 0.0; }
	virtual double getDownLine() { return 0.0; }
public:
	double Rwidth = 200.0;   //�±߽�
	double up_boundary = 0.0;   //�ϱ߽�
	double down_boundary = 0.0;  //�±߽�
	double left_boundary = 0.0;  //��߽�
	double right_boundary = 0.0;  //�ұ߽�
};

class RoadNormal :public RoadBase   //һ���·(�̳л���road)
{
public:
	RoadNormal(const double& r_width = 200.0);
	void showRoad() override;
};

class RoadCross:public RoadBase   //�����ߵ�·
{
public:
	RoadCross(const double& r_width = 200.0);
	void showRoad() override;
	double getUpLine() { return this->up_line; }  //����������
	double getMidLine() { return this->mid_line; }
	double getDownLine() { return this->down_line; }
public:
	double up_line = 0.0;
	double mid_line = 0.0;
	double down_line = 0.0;
	double disRec = 20.0;  //���

};

class RoadDoubleLine :public RoadBase
{
public:
	RoadDoubleLine(const double& r_width = 200.0);
	void showRoad();

};