#pragma once
#include "scene_base.h"

class StraightStopObs :public sceneBase  //停障
{
public:
	StraightStopObs();
	void showScene();
	bool planning_process() override;   //纯虚函数重写
public:
	unique_ptr<Cone> cone;    //放置一个锥桶
	double safedis = 50.0;
};

class StraightStation :public sceneBase   //停站
{
public:
	StraightStation();
	void showScene();
	bool planning_process() override;
public:
	unique_ptr<Point> station;
	int stop_time = 3;   //停靠时间
};

class StraightFollow :public sceneBase
{
public:
	StraightFollow();
	void showScene();
	bool planning_process() override;
public:
	unique_ptr<carNormal> carObs;   //障碍车
	double safedis = 120.0;   //安全距离
};

class StraightCrossWalk :public sceneBase
{
public:
	StraightCrossWalk();
	bool peopleInCross();   //判断是否有人
	void showScene();
	bool planning_process() override;
public:
	int people_n = 5;
	vector<unique_ptr<Person>> people;
	double cross_limit = -2.5;   //限速
};