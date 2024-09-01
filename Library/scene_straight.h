#pragma once
#include "scene_base.h"

class StraightStopObs :public sceneBase  //ͣ��
{
public:
	StraightStopObs();
	void showScene();
	bool planning_process() override;   //���麯����д
public:
	unique_ptr<Cone> cone;    //����һ��׶Ͱ
	double safedis = 50.0;
};

class StraightStation :public sceneBase   //ͣվ
{
public:
	StraightStation();
	void showScene();
	bool planning_process() override;
public:
	unique_ptr<Point> station;
	int stop_time = 3;   //ͣ��ʱ��
};

class StraightFollow :public sceneBase
{
public:
	StraightFollow();
	void showScene();
	bool planning_process() override;
public:
	unique_ptr<carNormal> carObs;   //�ϰ���
	double safedis = 120.0;   //��ȫ����
};

class StraightCrossWalk :public sceneBase
{
public:
	StraightCrossWalk();
	bool peopleInCross();   //�ж��Ƿ�����
	void showScene();
	bool planning_process() override;
public:
	int people_n = 5;
	vector<unique_ptr<Person>> people;
	double cross_limit = -2.5;   //����
};