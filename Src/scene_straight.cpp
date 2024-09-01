#include "scene_straight.h"

StraightStopObs::StraightStopObs()
{
	road0 = make_unique<RoadNormal>();   //����ָ�봴���������
	cone = make_unique<Cone>(Swidth / 2.0, Swidth / 4.0, 50);
	car0 = make_unique<carNormal>(Swidth/2.0,Sheight-70.0);    //�����������
	car0->speed_y = -6.0;   //ָ����ʼ�ٶ�

	car0->coutInfo();
	showScene();    //��Ҫ��д���麯��
	system("pause");   //�ֶ�����
}

void StraightStopObs::showScene()
{
	BeginBatchDraw();
	cleardevice();

	road0->showRoad();
	cone->showCone();
	car0->showCar(BLACK);

	EndBatchDraw();
	Delay(DELAYTIME);
}

bool StraightStopObs::planning_process()
{
	double stopline = cone->p_center->y + cone->r + safedis;
	uniformAccByDis(car0->pmidf->y - stopline, 0.0);   //���վ�����ٵ�0
	return true;

}

StraightStation::StraightStation()
{
	road0 = make_unique<RoadNormal>();
	car0 = make_unique<carNormal>(Swidth / 2.0, Sheight - 70.0);
	station = make_unique<Point>(Swidth / 2.0, Sheight / 2.0);    //����վ��

	car0->speed_y = -6.0;
	car0->coutInfo();
	showScene();    //��Ҫ��д���麯��
	system("pause");   //�ֶ�����
}

void StraightStation::showScene()
{
	BeginBatchDraw();
	cleardevice();

	road0->showRoad();
	station->showPoint();
	car0->showCar(BLACK);

	EndBatchDraw();
	Delay(DELAYTIME);
}

bool StraightStation::planning_process()
{
	uniformAccByDis(car0->pmid->y - station->y, 0.0);   //���ٽ�վ
	Delay(stop_time * 1000);   //ͣ3��
	uniformAccByTime(speedlimit, 2.0);  //2s�ڼ��ٵ�����
	return true;

}

StraightFollow::StraightFollow()
{
	road0 = make_unique<RoadNormal>();
	carObs = make_unique<carNormal>(Swidth / 2.0, Sheight / 2.0, 0.0, 50.0, 100.0);   //�ϰ���
	carObs->speed_y = -2.0;
	car0 = make_unique<carNormal>(Swidth / 2.0, Sheight - 70.0);
	car0->speed_y = -6.0;

	car0->coutInfo();
	carObs->coutInfo();

	showScene();
	system("pause");
}

void StraightFollow::showScene()
{
	BeginBatchDraw();
	cleardevice();

	road0->showRoad();
	carObs->showCar(RED);
	car0->showCar(BLACK);

	EndBatchDraw();
	Delay(DELAYTIME);
}

bool StraightFollow::planning_process()
{
	double dis = car0->pmid->y - carObs->pmid->y-safedis;
	double d_speed = car0->speed_y - carObs->speed_y;
	//cout << " ���ﲻ��!,d_speed: "<<d_speed<<" �����:  "<<dis << endl;
	if (d_speed > 0 || dis <= 0) return false;
	car0->a_y = pow(d_speed, 2) / dis / 2.0;

	while (car0->pmidr->y > 0)
	{
		car0->moveStraightStep();
		carObs->moveStraightStep();
		if (fabs(car0->speed_y - carObs->speed_y) > fabs(carObs->speed_y))
			car0->speed_y += car0->a_y;
		else
		{
			car0->speed_y = carObs->speed_y;
			car0->a_y = 0.0;
		}
		showScene();
	}
	car0->coutInfo();
	return true;
}

StraightCrossWalk::StraightCrossWalk()
{
	road0 = make_unique<RoadCross>();
	car0 = make_unique<carNormal>(Swidth / 2.0, Sheight - 70.0);
	car0->speed_y = -3.0;

	for (int i = 0; i < people_n; i++)
	{
		unique_ptr<Person> ps = make_unique<Person>(road0->right_boundary + 20 * (i * 3 + 1), road0->getMidLine());  //ָ����������
		ps->speed = -2.5;
		people.emplace_back(move(ps));   //����move�൱�ڸ��Ƶ�ַ��������
	}
	car0->coutInfo();
	showScene();
	system("pause");
}

void StraightCrossWalk::showScene()
{
	BeginBatchDraw();
	cleardevice();

	road0->showRoad();
	car0->showCar(BLACK);
	car0->moveStraightStep();
	car0->speed_y += car0->a_y;

	for (auto& i : people)
	{
		i->PersonDraw(); 
		i->PersonMove();
	}

	EndBatchDraw();
	Delay(DELAYTIME);
}

bool StraightCrossWalk::peopleInCross()  //�ж��Ƿ�����
{
	for (auto& i : people)
	{
		if (i->p_center->x >= road0->left_boundary - i->r && i->p_center->x <= road0->right_boundary + i->r)
			return true;
	}
	return false;
}

bool StraightCrossWalk::planning_process()
{
	double dis = car0->pmidf->y - road0->getDownLine();
	car0->a_y = pow(car0->speed_y, 2) / 2.0 / dis;
	while (dis > 0.0)
	{
		dis = car0->pmidf->y - road0->getDownLine();
		if (!peopleInCross()) {
			if (car0->speed_y >= cross_limit)
				car0->a_y = 0.0; 
		}
		else
		{
			if (dis <= 7.0) {    //ֹͣ��ǰ��ȫͣ����ֹ����
				car0->speed_y = 0.0;
				car0->a_y = 0.0;
				break;
			}
		}
		cout << "1����:  " << dis << " ,�����ٶ�: " << car0->speed_y << ", ���ٶ�: " << car0->a_y << endl;
		showScene();
	}
	while (car0->pmidr->y > road0->getUpLine())
	{
		if (!peopleInCross())
		{
			if (car0->speed_y > cross_limit)   //û�����پͼ��٣�
				car0->a_y = -0.1;
			else
				car0->a_y = 0;
		}
		cout << "2����:  " << dis << " ,�����ٶ�: " << car0->speed_y << ", ���ٶ�: " << car0->a_y << endl;
		showScene();
	}
	while (car0->pmidr->y > 0.0)
	{
		if (car0->speed_y > speedlimit)
			car0->a_y = -0.05;
		else
			car0->a_y = 0.0;
		cout << "3����: " << car0->speed_y << " ���ٶ�: " << car0->a_y << endl;
		showScene();
	}
	return true;
}