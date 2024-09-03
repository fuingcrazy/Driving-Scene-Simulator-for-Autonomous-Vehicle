#include "scene_base.h"

void sceneBase::showScene()
{
	BeginBatchDraw();   //start drawing
	cleardevice();
	road0->showRoad();   
	car0->showCar(BLACK);

	if (ShowCircle && car0->p_center)
		car0->showCurve();

	EndBatchDraw();   //end drawing
	Delay(DELAYTIME);
}

void sceneBase::uniformStraight(const double& total_s)
{
	car0->updateStraightInfo(); 

	double s = 0;
	while (s < total_s) {
		s += fabs(car0->speed);
		car0->moveStraightStep();
		obsMoveStep();
		showScene();
	}
	car0->coutInfo();
}

void sceneBase::uniformAccBySpeed(const double& t_speed_y)
{
	while (car0->pmidr->y > 0.0) {
		car0->moveStraightStep();
		obsMoveStep();   //if there is an obstacle car, move
		if (fabs(car0->speed_y - t_speed_y) > fabs(car0->a_y))   //speed difference between now and target is large enough
			car0->speed_y += car0->a_y;
		else
		{
			car0->speed_y = t_speed_y;   //slight difference, make two equal
			car0->a_y = 0.0;
			if (t_speed_y == 0.0) break; //pull over
		}
		showScene();   
	}
	car0->coutInfo();
}

void sceneBase::uniformAccByDis(const double& dis, const double &t_speed_y)
{
	car0->a_y = (pow(car0->speed_y, 2) - pow(t_speed_y, 2)) / dis / 2.0;   //calculate accelerate
	cout << "Accelerate now: " << car0->a_y << ", dis= " << dis << endl;
	uniformAccBySpeed(t_speed_y);
}

void sceneBase::uniformAccByTime(const double& t_speed_y, const double& t_time)  //go straight and reach target speed within t_time
{
	double freq = t_time * 1000 / DELAYTIME;
	car0->a_y = (t_speed_y - car0->speed_y) / freq;
	cout << "Car accelerate: " << car0->a_y << endl;

	uniformAccBySpeed(t_speed_y);
}

void sceneBase::carTurn(const int& turn_state, const double& R, const double& total_theta)
{
	car0->updateTurnInfo(turn_state, R);   

	double theta = 0.0;
	while (theta < total_theta)
	{
		theta += fabs(car0->delta_theta);    //accumulated turning angle
		correctAngleError(car0->delta_theta, theta - total_theta);   //correct the error
		car0->TurnStep();
		obsMoveStep();
		showScene();
	}
	car0->coutInfo();
}

void sceneBase::laneChange(const Point& target_point, const int& type, const double& s)
{
	double dis = car0->pmidr->DistanceTo(target_point);
	Vec2d vec0(dis, car0->heading_theta + PI / 2.0);
	Vec2d vec(*car0->pmidr, target_point);
	double L = vec0.crossProd(vec) / dis / 2.0;
	double H = vec0.innerProd(vec) / dis / 2.0;

	if (fabs(L) < 1e-10)//target point lies on the track of vehicle
	{
		uniformStraight(car0->pmidr->DistanceTo(target_point));
		return;
	}

	double R = (pow(L, 2) + pow(H, 2)) / fabs(L) / 2.0;//turning radius
	double target_theta = asin(H / R);
	double target_delta_theta = fabs(car0->speed / R);//absolute value of angular velocity

	if (L > 0.0)//turning left
	{
		car0->delta_theta = target_delta_theta;
		carTurn(TurnDirection::TurnLeft, R, target_theta);

		if (type == singleType)
		{
			car0->delta_theta = -target_delta_theta;
			carTurn(TurnDirection::TurnRight, R, target_theta);
		}
		else
		{
			car0->delta_theta = -target_delta_theta;
			carTurn(TurnDirection::TurnRight, R, target_theta);

			uniformStraight(s);

			car0->delta_theta = -target_delta_theta;
			carTurn(TurnDirection::TurnRight, R, target_theta);

			car0->delta_theta = target_delta_theta;
			carTurn(TurnDirection::TurnLeft, R, target_theta);
		}
	}
	else if (L < 0.0)//turning right
	{
		car0->delta_theta = -target_delta_theta;
		carTurn(TurnDirection::TurnRight, R, target_theta);

		if (type == singleType)
		{
			car0->delta_theta = target_delta_theta;
			carTurn(TurnDirection::TurnLeft, R, target_theta);
		}
		else
		{
			car0->delta_theta = target_delta_theta;
			carTurn(TurnDirection::TurnLeft, R, target_theta);

			uniformStraight(s);//if overtake, keep going straight for a distance

			car0->delta_theta = target_delta_theta;
			carTurn(TurnDirection::TurnLeft, R, target_theta);

			car0->delta_theta = -target_delta_theta;
			carTurn(TurnDirection::TurnRight, R, target_theta);
		}
	}


}
