#pragma once
#include "scene_base.h"

class StaticObs :public sceneBase
{
public:
	StaticObs();
	void showScene();
	bool planning_process() override;  //规划过程
public:
	unique_ptr<Cone> cone;
	double start_dis = 200.0;   //开始作出反应的时间
};

class OvertakeObs : public sceneBase
{
public:
	OvertakeObs();
	void showScene();
	bool planning_process() override;
public:
	unique_ptr<carNormal> carObs;
	double start_dis = 0.0;
};

class MeetingObs : public sceneBase   //会车
{
public:
	MeetingObs();
	void showScene();
	bool planning_process() override;
public:
	unique_ptr<carNormal> carObs;
	double start_dis = 200.0;
};