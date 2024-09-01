#include "traffic.h"

Cone::Cone(const double& pos_x, const double& pos_y, const double& R) :r(R)
{
	p_center = make_unique<Point>(pos_x, pos_y);
}

void Cone::showCone()
{
	setfillcolor(RGB(255, 127, 0));
	solidcircle(p_center->x, p_center->y, r);   //»æÖÆÊµÐÄÔ²
}

Person::Person(const double& pos_x, const double& pos_y)
{
	p_center = make_unique<Point>(pos_x, pos_y);
}

void Person::PersonMove()
{
	p_center->x += speed;
}

void Person::PersonDraw()
{
	setfillcolor(YELLOW);
	fillcircle(p_center->x, p_center->y, r);
}