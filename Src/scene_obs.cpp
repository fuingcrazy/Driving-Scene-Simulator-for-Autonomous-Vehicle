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

OvertakeObs::OvertakeObs()
{
	double Rwidth = 100.0;
	road0 = make_unique<RoadDoubleLine>(Rwidth);    //create a road with double lanes
	carObs = make_unique<carNormal>(Swidth / 2.0 + Rwidth / 2.0, Sheight - 600.0, 0.0, 50.0, 100.0);
	car0 = make_unique<carNormal>(Swidth / 2.0 + Rwidth / 2.0, Sheight - 300.0, 0.0, 40.0, 80.0);

	carObs->speed = -1.0;
	car0->speed = -5.0;

	carObs->updateStraightInfo();//Let car Obstacle drive straightly

	car0->coutInfo();
	carObs->coutInfo();

	showScene();
	system("pause");
}

void OvertakeObs::showScene()
{
	BeginBatchDraw();
	cleardevice();

	road0->showRoad();
	carObs->showCar(RED);
	car0->showCar(BLACK);

	EndBatchDraw();
	Delay(DELAYTIME);
}

void OvertakeObs::obsMoveStep()
{
	carObs->moveStraightStep();
}

bool OvertakeObs::planning_process()
{
	double delta_speed = fabs(car0->speed) - fabs(carObs->speed);//speed difference, will not change in this scene
	cout << "delta_speed = " << delta_speed << endl;
	if (delta_speed <= 0.0)//car0 is slower, can not overtake
	{
		cout << "Main car is slower, can not take over" << endl;
		return false;
	}

	double dis_l_L = carObs->plr->x - road0->left_boundary;
	double target_l_x = road0->left_boundary + dis_l_L / 2.0;//left target point x
	cout << "dis_l_L = " << dis_l_L << ", target_l_x = " << target_l_x << endl;
	if (dis_l_L <= car0->car_width * 1.2)
	{
		cout << "Left distance not enough!, fail to overtake" << endl;
		return false;
	}

	start_dis = car0->car_length * 3;
	double dis0 = car0->pmidr->y - carObs->pmidr->y;   //Initial distance
	cout << "start_dis = " << start_dis << ", dis0 = " << dis0 << endl;
	if (dis0 < start_dis)
	{
		cout << "Too close to the car, can not change lane" << endl;
		return false;
	}

	uniformStraight(dis0 - start_dis);
	double time_lane_change = dis0 / delta_speed;//time to change lane
	double dis_target = carObs->car_length;//drive to a car length ahead of the obstacle car
	double target_y = car0->pmidr->y + car0->speed * time_lane_change;//target point y
	double time_straight = dis_target / delta_speed;//time to drive straightly
	double s = fabs(car0->speed) * time_straight;//distance to drive straight after overtaking
	cout << "dis_target = " << dis_target << ", target_y = " << target_y << ", s = " << s << endl;
	Point target_l_point(target_l_x, target_y);//left target point
	laneChange(target_l_point, LaneChangeType::doubleType, s);//change lane when surpass the slow car for 1 car length

	car0->updatepmidr();
	uniformStraight(car0->pmidr->y - 0.0);

	return true;
}

MeetingObs::MeetingObs()     //meeting and avoiding
{
	road0 = make_unique<RoadDoubleLine>(100.0);
	carObs = make_unique<carNormal>(Swidth / 2.0, 50.0, PI, 50.0, 100.0);
	car0 = make_unique<carNormal>(Swidth / 2.0, Sheight - 70.0, 0.0, 40.0, 80.0);
	carObs->speed = -3.0;
	car0->speed = -4.0;

	carObs->updateStraightInfo();

	car0->coutInfo();
	carObs->coutInfo();
	showScene();
	system("pause");
}

void MeetingObs::showScene()
{
	BeginBatchDraw();
	cleardevice();

	road0->showRoad();
	carObs->showCar(RED);
	car0->showCar(BLACK);

	EndBatchDraw();
	Delay(DELAYTIME);
}


void MeetingObs::obsMoveStep()
{
	carObs->moveStraightStep();
}

bool MeetingObs::planning_process()
{
	double total_speed = fabs(car0->speed) + fabs(carObs->speed);// sum of speeds
	double dis0 = fabs(car0->pmidf->y - carObs->pmidf->y);//initial distance
	if (total_speed <= 0.0)
	{
		cout << "Both car is static" << endl;
		return false;
	}

	double dis_r_L = road0->right_boundary - carObs->plr->x;
	double target_r_x = road0->right_boundary - dis_r_L / 2.0;//left target point
	cout << "dis_r_L = " << dis_r_L << ", target_r_x = " << target_r_x << endl;
	if (dis_r_L <= car0->car_width * 1.2)
	{
		cout << "Left margin not enough!" << endl;
		return false;
	}

	double time = dis0 / total_speed;
	double meeting_point_y = car0->pmidf->y + car0->speed * time;//anticipated collision position
	double s0 = car0->pmidf->y - (meeting_point_y + start_dis);
	cout << "total_speed = " << total_speed << ", dis0 = " << dis0 << ", meeting_point_y = " << meeting_point_y << ", s0 = " << s0 << endl;
	if (s0 < 0.0)
	{
		cout << "Distance too close, can't avoid!" << endl;
		return false;
	}

	uniformStraight(s0);
	Point target_r_point(target_r_x, meeting_point_y);//right target point
	laneChange(target_r_point, LaneChangeType::doubleType);

	car0->updatepmidr();
	uniformStraight(car0->pmidr->y - 0.0);
	return true;
}