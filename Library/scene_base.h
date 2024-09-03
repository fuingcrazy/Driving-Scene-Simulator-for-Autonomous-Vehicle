#pragma once
#include"car.h"
#include "road.h"
enum LaneChangeType
{
	singleType,    
	doubleType,
};

class sceneBase
{
public:
	virtual ~sceneBase() = default;
	virtual void showScene();    
	virtual void obsMoveStep() {};  //Step move for obstacle
	virtual bool planning_process() = 0;   //rewrite planning process for every scene

	void uniformStraight(const double& total_s);   //go straight, distance = total_s
	void uniformAccBySpeed(const double& t_spped_y);   //accelerate to speed_y
	void uniformAccByDis(const double& dis, const double& t_speed_y);   //accelerate to speed_y within dis
	void uniformAccByTime(const double& t_speed_y, const double& t_time);   //accelerate to speedY by t_time

	void carTurn(const int& turn_state, const double& R, const double& total_theta);  //turn,input:direction, turn radius
	void laneChange(const Point& target_point, const int& type, const double& s = 0.0);  
public:
	unique_ptr<RoadBase> road0;   //main road
	unique_ptr<carBase> car0;   //main car
	double speedlimit = -6.0;   
};