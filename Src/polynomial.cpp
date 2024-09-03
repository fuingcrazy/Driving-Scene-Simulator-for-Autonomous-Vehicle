#include "polynomial.h"

/*This is to solve a linear equation for 5th polynomial curve when changing lanes,
we set start time as t0, end time as t1. We use (x(t0),y(t0)) to represent the position of start, (x'(t0),y'(t0))
as velocity and (x''(t0),y''(t0)) as accelerate*/
/*Here we create a T matrice, it has 6 rows and 6 columns:
X = [X0  =  [t0^5    t0^4   t0^3    t0^2    t0    1   [a5
     X0'     5t0^4   4t0^3  3t0^2   2t0      1    0    a4
	 X0''    20t0^3  12t0^2 6t0      2       0    0    a3  = T x A
	 X1      t1^5    t1^4   t1^3    t1^2    t1    1    a2
     X1'     5t1^4   4t1^3  3t1^2   2t1      1    0    a1
	 X1'']   20t1^3  12t1^2 6t1      2       0    0]   a0]

Y = [Y0  =  [t0^5    t0^4   t0^3    t0^2    t0    1   [b5
     Y0'     5t0^4   4t0^3  3t0^2   2t0      1    0    b4
	 Y0''    20t0^3  12t0^2 6t0      2       0    0    b3  = T x B
	 Y1      t1^5    t1^4   t1^3    t1^2    t1    1    b2
     Y1'     5t1^4   4t1^3  3t1^2   2t1      1    0    b1
	 Y1'']   20t1^3  12t1^2 6t1      2       0    0]   b0]
  Thus, we can calculate matrice A and B via solving A = T^(-1)*X, B = T^(-1)*Y*/

void Matrix::createMat(double t0, double t1)   //create matrice T
{
	m = 6;
	n = 6;
	for (int j = 1; j <= n; ++j)
	{
		mat[1][j] = pow(t0, n - j);
		mat[2][j] = (n - j) * pow(t0, n - j - 1);
		mat[3][j] = (n - j) * (n - j - 1) * pow(t0, n - j - 2);
		if (n - j - 1< 0) mat[2][j] = 0.0;
		if (n - j - 2 < 0) mat[3][j] = 0.0;
	}
	for (int j = 1; j <= n; ++j)
	{
		mat[4][j] = pow(t1, n - j);
		mat[5][j] = (n - j) * pow(t1, n - j - 1);
		mat[6][j] = (n - j) * (n - j - 1) * pow(t1, n - j - 2);
		if (n - j - 1 < 0) mat[5][j] = 0.0;
		if (n - j - 2 < 0) mat[6][j] = 0.0;
	}
}

void Matrix::createVector(double nm1, double nm2, double nm3, double nm4, double nm5, double nm6) //create vector according to start and end state
{
	m = 6;
	n = 1;
	mat[1][1] = nm1;
	mat[2][1] = nm2;
	mat[3][1] = nm3;
	mat[4][1] = nm4;
	mat[5][1] = nm5;
	mat[6][1] = nm6;
}

void Matrix::print()   //print matrice
{
	for (int i = 1; i <= m; i++)
	{
		for (int j = 1; j <= n; j++)
		{
			cout << mat[i][j] << "\t";
		}
		cout << endl;
	}
	cout << endl;
}

void Matrix::augMat(Matrix a, Matrix b)   //create augmented matrix
{
	m = a.m;
	n = a.n + 1;   //columns + 1
	for (int i = 1; i <= m; ++i)
	{
		for (int j = 1; j <= n; ++j) {
			mat[i][j] = a.mat[i][j];
		}
		mat[i][n] = b.mat[i][1];    
	}
}

bool Matrix::solve(Matrix a, Matrix b)
{
	if (a.m != a.n)
	{
		cout << "A is not a squared matrice£¬can not solve!" << endl;
		return false;
	}
	m = a.m;
	n = 1;
	Matrix aa;
	aa.augMat(a,b);   
	for (int i = 1; i <= aa.m; ++i)    //transform aa into upper triangular matrice
	{
		int k;
		for (k = i; k <= aa.m; ++k) {
			if (fabs(aa.mat[k][i]) > 1e-10) break;     //looking for non-zero position for each column
		}
		if (k <= aa.m)   //non-zero
		{
			for (int j = 1; j <= aa.n; ++j)
			{
				aa.mat[0][j] = aa.mat[i][j];
				aa.mat[i][j] = aa.mat[k][j];
				aa.mat[k][j] = aa.mat[0][j];    //swap row i and j
			}
			double c;
			for (int j = i + 1; j <= aa.m; ++j) {
				c = -aa.mat[j][i] / aa.mat[i][i];    
				for (k = i; k <= aa.n; k++)
					aa.mat[j][k] += c * aa.mat[i][k];
			}
		}
		else
			return false;
	}
	for (int i = a.m; i >= 1; i--)
	{
		mat[i][1] = aa.mat[i][aa.n];
		for (int j = a.m; j > i; j--)
		{
			mat[i][1] -= mat[j][1] * aa.mat[i][j];
		}
		mat[i][1] /= aa.mat[i][i];
	}
	return true;

}

Polynomial::Polynomial()
{
	road0 = make_unique<RoadNormal>(180.0);    //create main road
	car_obs = make_unique<carNormal>(385, 100, 0.0, 50.0, 100.0);   //create obstacle vehicle
	car0 = make_unique<carNormal>(385, Sheight - 60.0,0.0,50.0,100.0);
	car0->speed_y = -5.0;
	
	car0->coutInfo();
	showScene();
    
	system("pause");
}

void Polynomial::showScene()
{
	BeginBatchDraw();
	cleardevice();

	road0->showRoad();
	car0->showCar(RED);
	car_obs->showCar(BLACK);
	setlinestyle(PS_DASHDOT);
	line(Swidth / 2.0, Sheight, Swidth / 2.0, 0);
	if (state == 0 || state == 2)
		drawDotStraight();
	else
		drawDotLane();     
	EndBatchDraw();
	Delay(DELAYTIME);
}

void Polynomial::changeLane(Point pt, Point next)
{
	BeginBatchDraw();
	cleardevice();

	road0->showRoad();
	car_obs->showCar(BLACK);
	setlinecolor(PS_DASHDOT);
	drawDotLane();
	line(Swidth / 2.0, Sheight, Swidth / 2.0, 0);
	car0->TurnByAngle(pt, next);
	car0->showCar(RED);

	EndBatchDraw();
}

void Polynomial::drawDotStraight()
{
	setfillcolor(BLUE);
	Point pt(car0->pmid->x,car0->pmid->y);
	for (int i = 0; i < 30; i++)
	{
		pt.y -= safe_dis / 30;
		solidcircle(385, pt.y, 3);
	}
}

void Polynomial::drawDotLane()
{
	setfillcolor(GREEN);
	auto it = trackPoint.begin();
	while (it != trackPoint.end())
	{
		solidcircle(it->x, it->y, 3);
		it++;
	}

}
void Polynomial::calMat()
{
	x0 = car0->pmid->x;
	y0 = car0->pmid->y;
	x1 = 215;
	y1 = car_obs->pmid->y;    //y of target point(on the left of the obstacle vehicle)
	X.createVector(x0, 0.0, 0.0, x1,0.0,0.0);
	Y.createVector(y0, -200, 0.0, y1, -200, 0.0);
	T.createMat(t0, t1);
	if (A.solve(T, X)) {
		cout << "A matrice:" << endl;
		A.print();
	}
	if (B.solve(T, Y)) {
		cout << "B matrice£º" << endl;
		B.print();
	}
	double temp_x;
	double temp_y;
	setfillcolor(GREEN);
	for (double t = t0; t <= t1; t += delta_t)
	{
		temp_x = A.mat[1][1] * pow(t, 5) + A.mat[2][1] * pow(t, 4) + A.mat[3][1] * pow(t, 3) + A.mat[4][1] * pow(t, 2) + A.mat[5][1] * pow(t,1) + A.mat[6][1];
		temp_y = B.mat[1][1] * pow(t, 5) + B.mat[2][1] * pow(t, 4) + B.mat[3][1] * pow(t, 3) + B.mat[4][1] * pow(t, 2) + B.mat[5][1] * pow(t,1) + B.mat[6][1];
		Point dot(temp_x, temp_y);
		trackPoint.emplace_back(dot);    //put track points in a vector
		solidcircle(temp_x, temp_y, 3);
	}
}

bool Polynomial::planning_process()
{
	double dis = car0->pmidf->y - car_obs->pmidr->y;
	if (dis < safe_dis) {
		cout << "Distance too close, can not manoeuvre!" << endl;
		return false;
	}
	while (dis >= safe_dis) {
		dis = car0->pmid->y - car_obs->pmid->y;
		car0->moveStraightStep();
		showScene();
	}
	state = 1;    //state 1 ends
	calMat();   
	auto it = trackPoint.begin();
	while (it+1 != trackPoint.end()) {   //follow the track
		changeLane(*it,*(it+1));
		Delay(100/fabs(car0->speed_y));
		it++;
	}
	while (car0->pmid->y >= 0)
	{
		car0->moveStraightStep();
		showScene();
	}

	return true;
}