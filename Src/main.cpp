#include "planning_base.h"
#include "main.h"


bool process(const int& type)   //��������ѡ����ں���
{
	switch (type)
	{
	case PlanType::StraightStopObsT:
	{
		unique_ptr<sceneBase> job = make_unique<StraightStopObs>();  //����-->�������
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
	initgraph(Swidth, Sheight, EW_SHOWCONSOLE);    //����ͼ�δ���
	setbkcolor(WHITE);
	cleardevice();
	if (process(PlanType::ObsPoly))
		cout << "�������! " << endl;
	system("pause");
	closegraph();
	return 0;
}