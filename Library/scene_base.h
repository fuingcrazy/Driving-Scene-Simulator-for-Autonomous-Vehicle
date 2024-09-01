#pragma once
#include"car.h"
#include "road.h"
enum LaneChangeType
{
	singleType,    //������
	doubleType,
};

class sceneBase
{
public:
	virtual ~sceneBase() = default;
	virtual void showScene();    //�Ǵ��麯��������಻��д�Ϳ���ֱ�ӵ���
	virtual void obsMoveStep() {};  //�ϰ��ﵥ֡�ƶ�
	virtual bool planning_process() = 0;   //�����滮����Ҫ�ֱ���

	void uniformStraight(const double& total_s);   //ֱ�У��ۼ���ʻ����Լ��
	void uniformAccBySpeed(const double& t_spped_y);   //������ʻ
	void uniformAccByDis(const double& dis, const double& t_speed_y);   //ָ������ʱ��һ���ٶ�
	void uniformAccByTime(const double& t_speed_y, const double& t_time);   //�涨ʱ���ڵ�һ���ٶ�

	void carTurn(const int& turn_state, const double& R, const double& total_theta);  //ת����֪�뾶�ͽǶ�
	void laneChange(const Point& target_point, const int& type, const double& s = 0.0);  //�����
public:
	unique_ptr<RoadBase> road0;   //��·����ָ��
	unique_ptr<carBase> car0;   //��������ָ��
	double speedlimit = -6.0;   //��·����
};