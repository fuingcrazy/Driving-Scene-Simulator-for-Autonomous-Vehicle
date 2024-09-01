#pragma once
#include"car.h"
#include "road.h"
enum LaneChangeType
{
	singleType,    //单移线
	doubleType,
};

class sceneBase
{
public:
	virtual ~sceneBase() = default;
	virtual void showScene();    //非纯虚函数如果子类不重写就可以直接调用
	virtual void obsMoveStep() {};  //障碍物单帧移动
	virtual bool planning_process() = 0;   //整个规划过程要分别处理

	void uniformStraight(const double& total_s);   //直行，累计行驶距离约束
	void uniformAccBySpeed(const double& t_spped_y);   //加速行驶
	void uniformAccByDis(const double& dis, const double& t_speed_y);   //指定距离时到一定速度
	void uniformAccByTime(const double& t_speed_y, const double& t_time);   //规定时间内到一定速度

	void carTurn(const int& turn_state, const double& R, const double& total_theta);  //转向，已知半径和角度
	void laneChange(const Point& target_point, const int& type, const double& s = 0.0);  //变道用
public:
	unique_ptr<RoadBase> road0;   //主路父类指针
	unique_ptr<carBase> car0;   //主车父类指针
	double speedlimit = -6.0;   //道路限速
};