#pragma once
#include "planning_base.h"

enum Shift   
{
m_D,
m_N,
m_R,
m_P,
};

enum TurnDirection   
{
	TurnRight,
	TurnLeft,
};

class carBase
{
public:
	virtual ~carBase() = default;
	void initCar(const double& pos_x, const double& pos_y, const double& heading, const double& width, const double& length);    //Initialize the car
	void updatepmidf();
	void updatepmidr();
	void updatepmid();    
	void updateXYva();   
	void showCar(const COLORREF& color);    
	void showCurve();   
	void coutInfo();    //打印信息

	void moveStraightStep();    //单帧直行
	void TurnStep();    //单帧转向
	void TurnByAngle(Point first, Point Second);

	void updateStraightInfo();

	void updateRinRout(const double& R);   //更新四个转向半径
	void updateTurnInfo(const int& turn_state, const double& R);   //更新转向信息
public:
	double car_width = 50.0;
	double car_length = 100.0;    //车辆尺寸
	unique_ptr<Point> plf;    //左前点, 智能指针，自动析构  
	unique_ptr<Point> plr;    //右前点
	unique_ptr<Point> prf;    //左后点
	unique_ptr<Point> prr;    //左后点
	unique_ptr<Point> p_center;    //转向中心点

	unique_ptr<Point> pmidf;    //前轮中心点
	unique_ptr<Point> pmidr;   //后轴中心点
	unique_ptr<Point> pmid;   //几何中心点

	double Rmin = 100.0;    //最小转弯半径
	double Rof = 0;   //外前半径
	double Ror = 0;   //外后
	double Rif = 0;   //内前
	double Rir = 0;   //内后
	double Rin = 0;   //内部半径

	double R0;   //自转半径
	double theta0;   //角度,=atan(car_length/car_width)
	double theta1;

	double speed = 0;
	double speed_x = 0;
	double speed_y = 0;   //速度与分量

	double a = 0;   //加速度
	double a_x = 0;
	double a_y = 0;   //y向加速度

	double delta_theta = 0;  //角速度，-代表顺时针转向
	double delta_theta_rot = 0;  //自转角速度
	double heading_theta = 0;   //航向角，+为左偏航

	int Gear = m_P;    //档位

};

class carNormal :public carBase
{
public:
	carNormal(const double& pos_x, const double& pos_y, const double& heading = 0.0, const double& width = 80, const double& length = 160.0);

};