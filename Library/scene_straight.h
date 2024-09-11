#pragma once
#include "scene_base.h"

class StraightStopObs :public sceneBase  
{
public:
	StraightStopObs();
	void showScene();
	bool planning_process() override;   //rewrite
public:
	unique_ptr<Cone> cone;    
	double safedis = 50.0;
};

class StraightStation :public sceneBase   
{
public:
	StraightStation();
	void showScene();
	bool planning_process() override;
public:
	unique_ptr<Point> station;
	int stop_time = 2;   
};

class StraightFollow :public sceneBase
{
public:
	StraightFollow();
	void showScene();
	bool planning_process() override;
public:
	unique_ptr<carNormal> carObs;   // obstacle car
	double safedis = 80.0;   //safe distance
};

class StraightCrossWalk :public sceneBase
{
public:
	StraightCrossWalk();
	bool peopleInCross();   //if there is someone on the crosswalk
	void showScene();
	bool planning_process() override;
public:
	int people_n = 5;
	vector<unique_ptr<Person>> people;
	double cross_limit = -2.5;   //speed limit when driving accorss the crossing
};