#include "polynomial.h"

void Matrix::createMat(double t0, double t1)
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

void Matrix::createVector(double nm1, double nm2, double nm3, double nm4, double nm5, double nm6)
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

void Matrix::print()
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

void Matrix::augMat(Matrix a, Matrix b)
{
	m = a.m;
	n = a.n + 1;   //列数+1
	for (int i = 1; i <= m; ++i)
	{
		for (int j = 1; j <= n; ++j) {
			mat[i][j] = a.mat[i][j];
		}
		mat[i][n] = b.mat[i][1];    //在最后一行赋值
	}
}

bool Matrix::solve(Matrix a, Matrix b)
{
	if (a.m != a.n)
	{
		cout << "A不是方阵，不能求解!" << endl;
		return false;
	}
	m = a.m;
	n = 1;
	Matrix aa;
	aa.augMat(a,b);   //增广矩阵
	for (int i = 1; i <= aa.m; ++i)    //化增广矩阵为上三角矩阵
	{
		int k;
		for (k = i; k <= aa.m; ++k) {
			if (fabs(aa.mat[k][i]) > 1e-10) break;     //按照上三角查找每列的非零位置
		}
		if (k <= aa.m)   //存在非零元素
		{
			for (int j = 1; j <= aa.n; ++j)
			{
				aa.mat[0][j] = aa.mat[i][j];
				aa.mat[i][j] = aa.mat[k][j];
				aa.mat[k][j] = aa.mat[0][j];    //交换k和i行
			}
			double c;
			for (int j = i + 1; j <= aa.m; ++j) {
				c = -aa.mat[j][i] / aa.mat[i][i];    //相加的倍数
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
	road0 = make_unique<RoadNormal>(180.0);    //创建主路
	car_obs = make_unique<carNormal>(385, 100, 0.0, 50.0, 100.0);   //创建障碍物车辆
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
		drawDotLane();     //状态1就是绘制多项式拟合的轨迹
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
	/*car0->plf->x = pt.x - car0->car_width / 2.0;
	car0->plf->y = pt.y - car0->car_length / 2.0;
	car0->prf->x = pt.x + car0->car_width/2.0;
	car0->prf->y = car0->plf->y;
	car0->plr->x = car0->plf->x;
	car0->plr->y = pt.y + car0->car_length / 2.0;
	car0->prr->x = car0->prf->x;
	car0->prr->y = car0->plr->y;*/
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
	y1 = car_obs->pmid->y;    //定义目标点的坐标
	X.createVector(x0, 0.0, 0.0, x1,0.0,0.0);
	Y.createVector(y0, -200, 0.0, y1, -200, 0.0);
	T.createMat(t0, t1);
	if (A.solve(T, X)) {
		cout << "A矩阵如下:" << endl;
		A.print();
	}
	if (B.solve(T, Y)) {
		cout << "B矩阵如下：" << endl;
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
		trackPoint.emplace_back(dot);    //算出的点放到容器里面
		solidcircle(temp_x, temp_y, 3);
	}
}

bool Polynomial::planning_process()
{
	double dis = car0->pmidf->y - car_obs->pmidr->y;
	if (dis < safe_dis) {
		cout << "距离过近！不能换道超车" << endl;
		return false;
	}
	while (dis >= safe_dis) {
		dis = car0->pmid->y - car_obs->pmid->y;
		car0->moveStraightStep();
		showScene();
	}
	state = 1;    //第一阶段结束
	calMat();    //计算AB矩阵、生成目标点集
	auto it = trackPoint.begin();
	while (it+1 != trackPoint.end()) {
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