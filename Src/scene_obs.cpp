#include "scene_obs.h"

StaticObs::StaticObs()
{
	road0 = make_unique<RoadNormal>(250.0);
	cone = make_unique<Cone>(Sheight / 2.0, Swidth / 2.0, 50.0);

	car0 = make_unique<carNormal>(Swidth / 2.0, Sheight - 70);
	car0->speed = -5.0;

	car0->coutInfo();
	showScene();
	system("pause");
}

void StaticObs::showScene()
{
	BeginBatchDraw();
	cleardevice();

	road0->showRoad();
	cone->showCone();
	car0->showCar(BLACK);

	if (ShowCircle && car0->p_center)
		car0->showCurve();   

	EndBatchDraw();
	Delay(DELAYTIME);
}

bool StaticObs::planning_process()
{
	double dis_l_L = cone->p_center->x - cone->r - road0->left_boundary;//left distance to boundary
	double dis_r_L = road0->right_boundary - cone->p_center->x - cone->r;//right distance to boundary
	double target_l_x = road0->left_boundary + dis_l_L / 2.0;//left target point
	double target_r_x = road0->right_boundary - dis_r_L / 2.0;//right target point
	double target_y = cone->p_center->y;
	Point target_l_point(target_l_x, target_y);//generate target points
	Point target_r_point(target_r_x, target_y);
	cout << "dis_l_L = " << dis_l_L << " , dis_r_L = " << dis_r_L << " , target_l_x= " << target_l_x << " , target_r_x= " << target_r_x << ", target_y= " << target_y << endl;

	if (dis_l_L > car0->car_width * 1.2)//if left distance is large enough then change to left lane first
	{
		laneChange(target_l_point, LaneChangeType::doubleType);
	}
	else
	{
		if (dis_r_L > car0->car_width * 1.2)
		{
			laneChange(target_r_point, LaneChangeType::doubleType);
		}
		else//neither left or right side is wide enough for passing through
		{
			cout << "stop for insufficient space" << endl;
			double stopline = cone->p_center->y + cone->r + 50.0;
			uniformAccByDis(car0->pmidf->y - stopline, 0.0);
			return false;
		}
	}

	car0->updatepmidr();
	uniformStraight(car0->pmidr->y - 0.0);
	return true;

}