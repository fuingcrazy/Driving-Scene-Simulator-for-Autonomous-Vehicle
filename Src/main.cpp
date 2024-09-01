#include "planning_base.h"
#include "main.h"


bool process(const int& type)   //根据类型选择入口函数
{
	switch (type)
	{
	case PlanType::StraightStopObsT:
	{
		unique_ptr<sceneBase> job = make_unique<StraightStopObs>();  //父类-->子类对象
		return job->planning_process();
	}
	case PlanType::StraightStationT:
	{
		unique_ptr<sceneBase> job = make_unique<StraightStation>();
		return job->planning_process();
	}
	case PlanType::StraightFollowT:
	{
		unique_ptr<sceneBase> job = make_unique<StraightFollow>();
		return job->planning_process();
	}
	case PlanType::StraightCrossWalkT:
	{
		unique_ptr<sceneBase> job = make_unique<StraightCrossWalk>();
		return job->planning_process();
	}
	case PlanType::ObsPoly:
	{
		unique_ptr<sceneBase> job = make_unique<Polynomial>();
		return job->planning_process();
	}
	default:
		break;
	}
	cout << "incorrect input!" << endl;
	return false;
}

int main()
{
	initgraph(Swidth, Sheight, EW_SHOWCONSOLE);    //创建图形窗口
	setbkcolor(WHITE);
	cleardevice();
	if (process(PlanType::ObsPoly))
		cout << "场景完成! " << endl;
	system("pause");
	closegraph();
	return 0;
}