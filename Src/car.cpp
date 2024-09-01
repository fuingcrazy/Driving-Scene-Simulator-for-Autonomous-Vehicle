
#include "car.h"

void carBase::initCar(const double& pos_x, const double& pos_y, const double& heading, const double& width, const double& length) 
{
	car_length = length;
	car_width = width;
	heading_theta = heading;
	R0 = hypot(car_width / 2, car_length / 2);    //ת��뾶(��ֱ��������б�߳���)
	theta0 = atan(car_length / car_width);
	theta1 = atan(car_width / car_length);
	pmid = make_unique<Point>(pos_x, pos_y);    //�������ĵ�
	Rin = sqrt(pow(car_length, 2) + pow(car_width, 2));    //�������İ뾶
	plf = make_unique<Point>(pmid->x - car_width / 2, pmid->y - car_length / 2, PI - theta0, R0);   //ǰ��
	prf = make_unique<Point>(pmid->x + car_width / 2, pmid->y - car_length / 2, theta0, R0);   //ǰ��
	plr = make_unique<Point>(pmid->x - car_width / 2, pmid->y + car_length / 2, PI + theta0, R0);  //���
	prr = make_unique<Point>(pmid->x + car_width / 2, pmid->y + car_length / 2, -theta0, R0);   //�Һ�

	plf->PointTurn(*pmid, heading_theta);
	prf->PointTurn(*pmid, heading_theta);
	plr->PointTurn(*pmid, heading_theta);
	prr->PointTurn(*pmid, heading_theta);

	updatepmidf();
	updatepmidr();   //����ǰ�����е�
}

void carBase::updatepmidf()   //����ǰ���е�xyֵ
{	
	if (pmidf)    //����Ѿ�����pmidf�������
	{
		pmidf->x = plf->x / 2 + prf->x / 2;
		pmidf->y = plf->y / 2 + prf->y / 2;
	}
	else
		pmidf = make_unique<Point>(plf->x / 2 + prf->x / 2, plf->y / 2 + prf->y / 2);    //û�оʹ���
}

void carBase::updatepmidr()
{
	if (pmidr)    //����Ѿ�����pmidf�������
	{
		pmidr->x = plr->x / 2 + prr->x / 2;
		pmidr->y = plr->y / 2 + prr->y / 2;
	}
	else
		pmidr = make_unique<Point>(plr->x / 2 + prr->x / 2, plr->y / 2 + prr->y / 2);    //û�оʹ���
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
	setlinestyle(PS_SOLID, 4);    //�����߿�
	setlinecolor(color);
	line(plf->x, plf->y, prf->x, prf->y);
	setlinecolor(BLUE);
	line(prf->x, prf->y, prr->x, prr->y);
	setlinecolor(GREEN);
	line(prr->x, prr->y, plr->x, plr->y);
	setlinecolor(YELLOW);
	line(plr->x, plr->y, plf->x, plf->y);   //�ĵ�����

}

void carBase::showCurve()    //���ƹ켣��
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
	cout << "����x���꣺ " << pmid->x << "����y����: " << pmid->y << "��ת�뾶: " << pmid->R << "��ת�Ƕ�: " << pmid->theta << endl;
	cout << "speed: " << speed_y << "���ٶ�: " << a_y << "���ٶ�: " << delta_theta << "��ת���ٶ�: " << delta_theta_rot << "����ǣ� " << heading_theta << endl;

}

void carBase::moveStraightStep()   //��ֱ֡��
{
	plf->PointMove(speed_x, speed_y);
	prf->PointMove(speed_x, speed_y);
	plr->PointMove(speed_x, speed_y);
	prr->PointMove(speed_x, speed_y);
	pmidr->PointMove(speed_x, speed_y);
	pmidf->PointMove(speed_x, speed_y);
	pmid->PointMove(speed_x, speed_y);
}

void carBase::TurnStep()   //��֡ת��
{
	pmidr->PointTurn(*p_center, delta_theta);
	plf->PointTurn(*p_center, delta_theta);
	prf->PointTurn(*p_center, delta_theta);
	plr->PointTurn(*p_center, delta_theta);
	prr->PointTurn(*p_center, delta_theta);
}

void carBase::TurnByAngle(Point first, Point Second)    //���սǶȸ����ĵ�λ��
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

void carBase::updateRinRout(const double& R)   //����ת��뾶�����ĵ�뾶
{
	Ror = R + car_width / 2.0;
	Rir = R - car_width / 2.0;
	Rof = hypot(Ror, car_length);  //ƽ����
	Rif = hypot(Rir, car_length);
}

void carBase::updateTurnInfo(const int& turn_state, const double& R)  //��֪ת���򼰰뾶����ز���
{
	double x = 0.0;
	double y = 0.0;
	updateRinRout(R);  //�����ĵ�뾶
	if (turn_state == TurnDirection::TurnRight)  //��ת
	{ 
		x = pmidr->x + R * cos(heading_theta);   //����ת�����ĵ�����
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

		//����5����ǶȺͰ뾶
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
	p_center.reset();   //�ͷ�ָ����ڴ�

	Ror = 0.0;
	Rir = 0.0;
	Rof = 0.0;
	Rif = 0.0;

	pmidr->theta = 0.0;
	pmidr->R = 0.0;   //����ת����ز�����Ϊ0
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