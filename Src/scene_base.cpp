#include "scene_base.h"

void sceneBase::showScene()
{
	BeginBatchDraw();   //��ʼ����
	cleardevice();
	road0->showRoad();   //����ָ����������Ա
	car0->showCar(BLACK);

	if (ShowCircle && car0->p_center)
		car0->showCurve();

	EndBatchDraw();   //��������
	Delay(DELAYTIME);
}

void sceneBase::uniformStraight(const double& total_s)
{
	car0->updateStraightInfo();   //����һ��ֱ�е���Ϣ

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
		obsMoveStep();   //�����ϰ�����ƶ�
		if (fabs(car0->speed_y - t_speed_y) > fabs(car0->a_y))   //��ǰ���ٺ�Ŀ�공������㹻��
			car0->speed_y += car0->a_y;
		else
		{
			car0->speed_y = t_speed_y;   //��Ĳ���ֱ�����
			car0->a_y = 0.0;
			if (t_speed_y == 0.0) break; //ͣ��
		}
		showScene();   //����
	}
	car0->coutInfo();
}

void sceneBase::uniformAccByDis(const double& dis, const double &t_speed_y)
{
	car0->a_y = (pow(car0->speed_y, 2) - pow(t_speed_y, 2)) / dis / 2.0;   //������ٶ�
	cout << "��ǰ���ٶ�: " << car0->a_y << ", dis= " << dis << endl;
	uniformAccBySpeed(t_speed_y);
}

void sceneBase::uniformAccByTime(const double& t_speed_y, const double& t_time)  //ֱ����Ŀ��ʱ���ڴﵽ�涨�ٶ�
{
	double freq = t_time * 1000 / DELAYTIME;
	car0->a_y = (t_speed_y - car0->speed_y) / freq;
	cout << "���ٶ�: " << car0->a_y << endl;

	uniformAccBySpeed(t_speed_y);
}

void sceneBase::carTurn(const int& turn_state, const double& R, const double& total_theta)
{
	car0->updateTurnInfo(turn_state, R);   //����ת����Ϣ

	double theta = 0.0;
	while (theta < total_theta)
	{
		theta += fabs(car0->delta_theta);    //�ۻ���ת��ĽǶ�
		correctAngleError(car0->delta_theta, theta - total_theta);   //�������
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

	if (fabs(L) < 1e-10)//target_point�ڳ���ֱ�з����ϣ�����Ҫ�����ж���
	{
		uniformStraight(car0->pmidr->DistanceTo(target_point));
		return;
	}

	double R = (pow(L, 2) + pow(H, 2)) / fabs(L) / 2.0;//ת��뾶 //LΪ0������������Ѿ�return��
	double target_theta = asin(H / R);//Ŀ��ת��
	double target_delta_theta = fabs(car0->speed / R);//���ٶȾ���ֵ
	//cout << "dis = " << dis << ", L = " << L << ", H = " << H << ", R = " << R << ", target_theta = " << target_theta / PI << ", target_delta_theta = " << target_delta_theta / PI << endl;

	if (L > 0.0)//����
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
	else if (L < 0.0)//����
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

			uniformStraight(s);//����ǳ�����������Ҫֱ��һ�ξ���

			car0->delta_theta = target_delta_theta;
			carTurn(TurnDirection::TurnLeft, R, target_theta);

			car0->delta_theta = -target_delta_theta;
			carTurn(TurnDirection::TurnRight, R, target_theta);
		}
	}


}
