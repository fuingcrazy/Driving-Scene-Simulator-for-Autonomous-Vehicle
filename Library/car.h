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
	void coutInfo();    

	void moveStraightStep();    
	void TurnStep();    
	void TurnByAngle(Point first, Point Second);

	void updateStraightInfo();

	void updateRinRout(const double& R);   //update Turn info
	void updateTurnInfo(const int& turn_state, const double& R);  
public:
	double car_width = 50.0;
	double car_length = 100.0;    
	unique_ptr<Point> plf;    //left front point
	unique_ptr<Point> plr;    //left right point
	unique_ptr<Point> prf;    //rear front point
	unique_ptr<Point> prr;  
	unique_ptr<Point> p_center;   

	unique_ptr<Point> pmidf;    //middle point of front side
	unique_ptr<Point> pmidr;   //middle point of rear side
	unique_ptr<Point> pmid;   //geometrical point

	double Rmin = 100.0;    //turn radius
	double Rof = 0;   
	double Ror = 0;   
	double Rif = 0;   
	double Rir = 0;   
	double Rin = 0;   

	double R0;   
	double theta0;   //=atan(car_length/car_width)
	double theta1;  //=atan(car_width/car_length)

	double speed = 0;
	double speed_x = 0;
	double speed_y = 0;   

	double a = 0;   
	double a_x = 0;
	double a_y = 0;   

	double delta_theta = 0;  //angular velocity
	double delta_theta_rot = 0;  
	double heading_theta = 0;   

	int Gear = m_P;    

};

class carNormal :public carBase
{
public:
	carNormal(const double& pos_x, const double& pos_y, const double& heading = 0.0, const double& width = 80, const double& length = 160.0);

};