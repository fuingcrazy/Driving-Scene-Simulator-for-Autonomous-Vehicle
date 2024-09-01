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
	void coutInfo();    //��ӡ��Ϣ

	void moveStraightStep();    //��ֱ֡��
	void TurnStep();    //��֡ת��
	void TurnByAngle(Point first, Point Second);

	void updateStraightInfo();

	void updateRinRout(const double& R);   //�����ĸ�ת��뾶
	void updateTurnInfo(const int& turn_state, const double& R);   //����ת����Ϣ
public:
	double car_width = 50.0;
	double car_length = 100.0;    //�����ߴ�
	unique_ptr<Point> plf;    //��ǰ��, ����ָ�룬�Զ�����  
	unique_ptr<Point> plr;    //��ǰ��
	unique_ptr<Point> prf;    //����
	unique_ptr<Point> prr;    //����
	unique_ptr<Point> p_center;    //ת�����ĵ�

	unique_ptr<Point> pmidf;    //ǰ�����ĵ�
	unique_ptr<Point> pmidr;   //�������ĵ�
	unique_ptr<Point> pmid;   //�������ĵ�

	double Rmin = 100.0;    //��Сת��뾶
	double Rof = 0;   //��ǰ�뾶
	double Ror = 0;   //���
	double Rif = 0;   //��ǰ
	double Rir = 0;   //�ں�
	double Rin = 0;   //�ڲ��뾶

	double R0;   //��ת�뾶
	double theta0;   //�Ƕ�,=atan(car_length/car_width)
	double theta1;

	double speed = 0;
	double speed_x = 0;
	double speed_y = 0;   //�ٶ������

	double a = 0;   //���ٶ�
	double a_x = 0;
	double a_y = 0;   //y����ٶ�

	double delta_theta = 0;  //���ٶȣ�-����˳ʱ��ת��
	double delta_theta_rot = 0;  //��ת���ٶ�
	double heading_theta = 0;   //����ǣ�+Ϊ��ƫ��

	int Gear = m_P;    //��λ

};

class carNormal :public carBase
{
public:
	carNormal(const double& pos_x, const double& pos_y, const double& heading = 0.0, const double& width = 80, const double& length = 160.0);

};