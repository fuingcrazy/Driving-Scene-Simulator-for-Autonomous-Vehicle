#include "scene_base.h"

void sceneBase::showScene()
{
	BeginBatchDraw();   //开始绘制
	cleardevice();
	road0->showRoad();   //父类指针调用子类成员
	car0->showCar(BLACK);

	if (ShowCircle && car0->p_center)
		car0->showCurve();

	EndBatchDraw();   //结束绘制
	Delay(DELAYTIME);
}

void sceneBase::uniformStraight(const double& total_s)
{
	car0->updateStraightInfo();   //更新一下直行的信息

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
		obsMoveStep();   //如有障碍物，则移动
		if (fabs(car0->speed_y - t_speed_y) > fabs(car0->a_y))   //当前车速和目标车速相差足够大
			car0->speed_y += car0->a_y;
		else
		{
			car0->speed_y = t_speed_y;   //差的不大，直接相等
			car0->a_y = 0.0;
			if (t_speed_y == 0.0) break; //停车
		}
		showScene();   //绘制
	}
	car0->coutInfo();
}

void sceneBase::uniformAccByDis(const double& dis, const double &t_speed_y)
{
	car0->a_y = (pow(car0->speed_y, 2) - pow(t_speed_y, 2)) / dis / 2.0;   //计算加速度
	cout << "当前加速度: " << car0->a_y << ", dis= " << dis << endl;
	uniformAccBySpeed(t_speed_y);
}

void sceneBase::uniformAccByTime(const double& t_speed_y, const double& t_time)  //直行在目标时间内达到规定速度
{
	double freq = t_time * 1000 / DELAYTIME;
	car0->a_y = (t_speed_y - car0->speed_y) / freq;
	cout << "加速度: " << car0->a_y << endl;

	uniformAccBySpeed(t_speed_y);
}

void sceneBase::carTurn(const int& turn_state, const double& R, const double& total_theta)
{
	car0->updateTurnInfo(turn_state, R);   //更新转向信息

	double theta = 0.0;
	while (theta < total_theta)
	{
		theta += fabs(car0->delta_theta);    //累积已转向的角度
		correctAngleError(car0->delta_theta, theta - total_theta);   //误差修正
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

	if (fabs(L) < 1e-10)//target_point在车辆直行方向上，不需要做绕行动作
	{
		uniformStraight(car0->pmidr->DistanceTo(target_point));
		return;
	}

	double R = (pow(L, 2) + pow(H, 2)) / fabs(L) / 2.0;//转向半径 //L为0的情况在上面已经return了
	double target_theta = asin(H / R);//目标转角
	double target_delta_theta = fabs(car0->speed / R);//角速度绝对值
	//cout << "dis = " << dis << ", L = " << L << ", H = " << H << ", R = " << R << ", target_theta = " << target_theta / PI << ", target_delta_theta = " << target_delta_theta / PI << endl;

	if (L > 0.0)//左绕
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
	else if (L < 0.0)//右绕
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

			uniformStraight(s);//如果是超车，这里需要直行一段距离

			car0->delta_theta = target_delta_theta;
			carTurn(TurnDirection::TurnLeft, R, target_theta);

			car0->delta_theta = -target_delta_theta;
			carTurn(TurnDirection::TurnRight, R, target_theta);
		}
	}


}
