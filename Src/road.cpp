#include "road.h"

RoadNormal::RoadNormal(const double& r_width)
{
	Rwidth = r_width;
	left_boundary = Swidth / 2.0 - Rwidth;
	right_boundary = Swidth / 2.0 + Rwidth;   //road boundaries

}
void RoadNormal::showRoad()
{
	setlinestyle(PS_SOLID, 4);
	setlinecolor(BLUE);
	line(left_boundary, 0.0, left_boundary, Sheight);
	line(right_boundary, 0.0, right_boundary, Sheight);
}

RoadCross::RoadCross(const double& r_width)
{
	Rwidth = r_width;
	left_boundary = Swidth / 2.0 - Rwidth;
	right_boundary = Swidth / 2.0 + Rwidth;
	mid_line = Sheight / 2.0 - 250;
	up_line = mid_line - Rwidth / 2.0;
	down_line = mid_line + Rwidth / 2.0;
}

void RoadCross::showRoad()
{
	setlinestyle(PS_SOLID, 4);
	setlinecolor(MAGENTA);

	line(left_boundary, 0.0, left_boundary, Sheight);
	line(right_boundary, 0.0, right_boundary, Sheight);
	line(right_boundary, up_line, right_boundary, up_line);
	line(right_boundary, down_line, right_boundary, down_line);

	int i = 0;
	while (true)
	{
		double up_bound = up_line + disRec;
		double down_bound = down_line - disRec;
		double left_bound = left_boundary + disRec * (1 + 2 * i);
		double right_bound = left_boundary + disRec * 2 * (i + 1);
		if (right_bound >= right_boundary)
		{
			break;
		}
		rectangle(left_bound, up_bound, right_bound, down_bound);
		i++;
	}
}

RoadDoubleLine::RoadDoubleLine(const double& r_width)
{
	Rwidth = r_width;
	left_boundary = Swidth / 2.0 - Rwidth;
	right_boundary = Swidth / 2.0 + Rwidth;
}
void RoadDoubleLine::showRoad()
{
	setlinestyle(PS_SOLID, 4);
	setlinecolor(BLACK);

	line(left_boundary, 0.0, left_boundary, Sheight);
	line(right_boundary, 0.0, right_boundary, Sheight);

	setlinestyle(PS_DASH, 4);
	line(Swidth / 2.0, 0.0, Swidth / 2.0, Sheight);    //lane
}