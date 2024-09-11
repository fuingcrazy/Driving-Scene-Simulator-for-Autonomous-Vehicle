#include "planning_base.h"
#include "main.h"


bool process(const int& type)   //select enter function by type
{
	switch (type)
	{
	case PlanType::StraightStopObsT:
	{
		unique_ptr<sceneBase> job = make_unique<StraightStopObs>();  //using father unique pointer to create children object
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
	case PlanType::ObsPassOvertakeT:
	{
		unique_ptr<sceneBase> job = make_unique<OvertakeObs>();
		return job->planning_process();
	}
	case PlanType::ObsPassMeetingT:
	{
		unique_ptr<sceneBase> job = make_unique<MeetingObs>();
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
	initgraph(Swidth, Sheight, EW_SHOWCONSOLE);    //create a window
	setbkcolor(WHITE);
	cleardevice();
	if (process(PlanType::StraightCrossWalkT))
		cout << "Scene Finish! " << endl;
	system("pause");
	closegraph();
	return 0;
}