
#include "car.h"

void carBase::initCar(const double& pos_x, const double& pos_y, const double& heading, const double& width, const double& length) 
{
	car_length = length;
	car_width = width;
	heading_theta = heading;
	R0 = hypot(car_width / 2, car_length / 2);    //转向半径(算直角三角形斜边长度)
	theta0 = atan(car_length / car_width);
	theta1 = atan(car_width / car_length);
	pmid = make_unique<Point>(pos_x, pos_y);    //声明中心点
	Rin = sqrt(pow(car_length, 2) + pow(car_width, 2));    //计算中心半径
	plf = make_unique<Point>(pmid->x - car_width / 2, pmid->y - car_length / 2, PI - theta0, R0);   //前左
	prf = make_unique<Point>(pmid->x + car_width / 2, pmid->y - car_length / 2, theta0, R0);   //前右
	plr = make_unique<Point>(pmid->x - car_width / 2, pmid->y + car_length / 2, PI + theta0, R0);  //左后
	prr = make_unique<Point>(pmid->x + car_width / 2, pmid->y + car_length / 2, -theta0, R0);   //右后

	plf->PointTurn(*pmid, heading_theta);
	prf->PointTurn(*pmid, heading_theta);
	plr->PointTurn(*pmid, heading_theta);
	prr->PointTurn(*pmid, heading_theta);

	updatepmidf();
	updatepmidr();   //更新前后轴中点
}

void carBase::updatepmidf()   //更新前轴中点xy值
{	
	if (pmidf)    //如果已经存在pmidf这个对象
	{
		pmidf->x = plf->x / 2 + prf->x / 2;
		pmidf->y = plf->y / 2 + prf->y / 2;
	}
	else
		pmidf = make_unique<Point>(plf->x / 2 + prf->x / 2, plf->y / 2 + prf->y / 2);    //没有就创建
}

void carBase::updatepmidr()
{
	if (pmidr)    //如果已经存在pmidf这个对象
	{
		pmidr->x = plr->x / 2 + prr->x / 2;
		pmidr->y = plr->y / 2 + prr->y / 2;
	}
	else
		pmidr = make_unique<Point>(plr->x / 2 + prr->x / 2, plr->y / 2 + prr->y / 2);    //没有就创建
}

void carBase::updatepmid()
{
	double x = plf->x / 2.0 + prr->x / 2.0;
	double y = plf->y / 2.0 + prr->y / 2.0;

	if (pmid)
	{
		pmid->x = x;
		pmid->y = y;
	}
	else
		pmid = make_unique<Point>(x, y);
}

void carBase::showCar(const COLORREF &color)
{
	setlinestyle(PS_SOLID, 4);    //设置线宽
	setlinecolor(color);
	line(plf->x, plf->y, prf->x, prf->y);
	setlinecolor(BLUE);
	line(prf->x, prf->y, prr->x, prr->y);
	setlinecolor(GREEN);
	line(prr->x, prr->y, plr->x, plr->y);
	setlinecolor(YELLOW);
	line(plr->x, plr->y, plf->x, plf->y);   //四点连线

}

void carBase::showCurve()    //绘制轨迹线
{
	setlinestyle(PS_DOT, 2);
	setlinecolor(MAGENTA);
	circle(p_center->x, p_center->y, Rof);
	circle(p_center->x, p_center->y, Ror);
	circle(p_center->x, p_center->y, Rif);
	circle(p_center->x, p_center->y, Rir);
}

void carBase::coutInfo()
{
	cout << "车辆x坐标： " << pmid->x << "车辆y坐标: " << pmid->y << "旋转半径: " << pmid->R << "旋转角度: " << pmid->theta << endl;
	cout << "speed: " << speed_y << "加速度: " << a_y << "角速度: " << delta_theta << "自转角速度: " << delta_theta_rot << "航向角： " << heading_theta << endl;

}

void carBase::moveStraightStep()   //单帧直行
{
	plf->PointMove(speed_x, speed_y);
	prf->PointMove(speed_x, speed_y);
	plr->PointMove(speed_x, speed_y);
	prr->PointMove(speed_x, speed_y);
	pmidr->PointMove(speed_x, speed_y);
	pmidf->PointMove(speed_x, speed_y);
	pmid->PointMove(speed_x, speed_y);
}

void carBase::TurnStep()   //单帧转向
{
	pmidr->PointTurn(*p_center, delta_theta);
	plf->PointTurn(*p_center, delta_theta);
	prf->PointTurn(*p_center, delta_theta);
	plr->PointTurn(*p_center, delta_theta);
	prr->PointTurn(*p_center, delta_theta);
}

void carBase::TurnByAngle(Point first, Point Second)    //按照角度更新四点位置
{
	double dis = first.DistanceTo(Second);
	heading_theta = asin((Second.x - first.x) / dis);
	pmid->x = first.x;
	pmid->y = first.y;
	double alpha = theta1 - heading_theta;
	plf->x = pmid->x - 0.5 * Rin * sin(alpha);
	plf->y = pmid->y - 0.5 * Rin * cos(alpha);
	double beta = 0.5 * PI - 2 * heading_theta - alpha;
	prf->x = pmid->x + 0.5*Rin*cos(beta);
	prf->y = pmid->y - 0.5 * Rin * sin(beta);
	plr->x = plf->x - car_length * sin(heading_theta);
	plr->y = plf->y + car_length * cos(heading_theta);
	prr->x = prf->x - car_length * sin(heading_theta);
	prr->y = prf->y + car_length * cos(heading_theta);
}

void carBase::updateRinRout(const double& R)   //根据转向半径更新四点半径
{
	Ror = R + car_width / 2.0;
	Rir = R - car_width / 2.0;
	Rof = hypot(Ror, car_length);  //平方根
	Rif = hypot(Rir, car_length);
}

void carBase::updateTurnInfo(const int& turn_state, const double& R)  //已知转向方向及半径求相关参数
{
	double x = 0.0;
	double y = 0.0;
	updateRinRout(R);  //更新四点半径
	if (turn_state == TurnDirection::TurnRight)  //右转
	{ 
		x = pmidr->x + R * cos(heading_theta);   //计算转向中心点坐标
		y = pmidr->y - R * sin(heading_theta);

		pmidr->theta = heading_theta + PI;
		pmidr->R = R;

		plr->theta = pmidr->theta;
		plr->R = Ror;
		prr->theta = pmidr->theta;
		prr->R = Rir;
		plf->theta = pmidr->theta - atan(car_length / Ror);
		plf->R = Rof;
		prf->theta = pmidr->theta - atan(car_length / Rir);
		prf->R = Rif;
	}
	else
	{ 
		x = pmidr->x - R * cos(heading_theta);
		y = pmidr->y + R * sin(heading_theta);

		//更新5个点角度和半径
		pmidr->theta = heading_theta;
		pmidr->R = R;

		plr->theta = pmidr->theta;
		plr->R = Rir;

		prr->theta = pmidr->theta;
		prr->R = Ror;

		plf->theta = pmidr->theta + atan(car_length / Rir);
		plf->R = Rif;

		prf->theta = pmidr->theta + atan(car_length / Ror);
		prf->R = Rof;

	}
	cout << "center_turn.x: ," << x << " center_turn.y: ," << y << endl;
	if (p_center)
	{
		p_center->x = x;
		p_center->y = y;
	}
	else
		p_center = make_unique<Point>(x, y);
}

void carBase::updateXYva()
{
	speed_x = speed * sin(heading_theta);
	speed_y = speed * cos(heading_theta);
	a_x = a * sin(heading_theta);
	a_y = a * cos(heading_theta);

}
void carBase::updateStraightInfo()
{
	updatepmidr();
	updatepmidf();
	updatepmid();
	updateXYva();
	p_center.reset();   //释放指向的内存

	Ror = 0.0;
	Rir = 0.0;
	Rof = 0.0;
	Rif = 0.0;

	pmidr->theta = 0.0;
	pmidr->R = 0.0;   //设置转向相关参数都为0
	pmidf->theta = 0.0;
	pmidf->R = 0.0;
	pmid->theta = 0.0;
	pmid->R = 0.0;
	plr->theta = 0.0;
	plr->R = 0.0;
	prr->theta = 0.0;
	prr->R = 0.0;
	plf->theta = 0.0;
	plf->R = 0.0;
	prf->theta = 0.0;
	prf->R = 0.0;

	delta_theta = 0.0;
	delta_theta_rot = 0.0;

}

carNormal::carNormal(const double& pos_x, const double& pos_y, const double& heading, const double& width, const double& length)
{
	initCar(pos_x, pos_y, heading, width, length);
}