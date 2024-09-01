#include "scene_straight.h"

StraightStopObs::StraightStopObs()
{
	road0 = make_unique<RoadNormal>();   //父类指针创建子类对象
	cone = make_unique<Cone>(Swidth / 2.0, Swidth / 4.0, 50);
	car0 = make_unique<carNormal>(Swidth/2.0,Sheight-70.0);    //创建子类对象
	car0->speed_y = -6.0;   //指定初始速度

	car0->coutInfo();
	showScene();    //需要重写的虚函数
	system("pause");   //手动启动
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
	uniformAccByDis(car0->pmidf->y - stopline, 0.0);   //按照距离减速到0
	return true;

}

StraightStation::StraightStation()
{
	road0 = make_unique<RoadNormal>();
	car0 = make_unique<carNormal>(Swidth / 2.0, Sheight - 70.0);
	station = make_unique<Point>(Swidth / 2.0, Sheight / 2.0);    //设置站点

	car0->speed_y = -6.0;
	car0->coutInfo();
	showScene();    //需要重写的虚函数
	system("pause");   //手动启动
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
	uniformAccByDis(car0->pmid->y - station->y, 0.0);   //减速进站
	Delay(stop_time * 1000);   //停3秒
	uniformAccByTime(speedlimit, 2.0);  //2s内加速到限速
	return true;

}

StraightFollow::StraightFollow()
{
	road0 = make_unique<RoadNormal>();
	carObs = make_unique<carNormal>(Swidth / 2.0, Sheight / 2.0, 0.0, 50.0, 100.0);   //障碍车
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
	//cout << " 这里不对!,d_speed: "<<d_speed<<" 距离差:  "<<dis << endl;
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
		unique_ptr<Person> ps = make_unique<Person>(road0->right_boundary + 20 * (i * 3 + 1), road0->getMidLine());  //指向新生成人
		ps->speed = -2.5;
		people.emplace_back(move(ps));   //不加move相当于复制地址，不允许
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

bool StraightCrossWalk::peopleInCross()  //判断是否有人
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
			if (dis <= 7.0) {    //停止线前完全停车防止倒车
				car0->speed_y = 0.0;
				car0->a_y = 0.0;
				break;
			}
		}
		cout << "1距离:  " << dis << " ,汽车速度: " << car0->speed_y << ", 加速度: " << car0->a_y << endl;
		showScene();
	}
	while (car0->pmidr->y > road0->getUpLine())
	{
		if (!peopleInCross())
		{
			if (car0->speed_y > cross_limit)   //没到限速就加速？
				car0->a_y = -0.1;
			else
				car0->a_y = 0;
		}
		cout << "2距离:  " << dis << " ,汽车速度: " << car0->speed_y << ", 加速度: " << car0->a_y << endl;
		showScene();
	}
	while (car0->pmidr->y > 0.0)
	{
		if (car0->speed_y > speedlimit)
			car0->a_y = -0.05;
		else
			car0->a_y = 0.0;
		cout << "3车速: " << car0->speed_y << " 加速度: " << car0->a_y << endl;
		showScene();
	}
	return true;
}