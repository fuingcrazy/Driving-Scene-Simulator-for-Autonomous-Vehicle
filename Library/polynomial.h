#pragma once
#include <cmath>;
#include <iostream>;
#include "scene_base.h"
#include "car.h"
constexpr auto N = 10;
using namespace std;
class Matrix
{
public:
	int m, n = 1;
	double mat[N][N] = { 0 };   //默认矩阵维度
	Matrix() = default;
	Matrix(int mm, int nn) {
		m = mm;
		n = nn;
	}
	void augMat(Matrix a, Matrix b);
	void createMat(double t0, double t1);    //根据始末时间创建6*6方阵
	void createVector(double nm1, double nm2, double nm3, double nm4, double nm5, double nm6);    //创建一组向量表示车辆状态
	bool solve(Matrix a, Matrix b);   //求解线性方程组
	void print();
};

class Polynomial :public sceneBase
{
public:
	Polynomial();
	void showScene();
	bool planning_process() override;
	void drawDotStraight();
	void calMat();    
	void drawDotLane();
	void changeLane(Point pt, Point nextpt);    //换道变角度
public:
	unique_ptr<carNormal> car_obs;    //障碍物车辆
	double safe_dis = 500.0;    //纵向安全距离
	double speed = -3.0;
	vector<Point> trackPoint;    //轨迹点集
	double changeLaneDis = 640;    //纵向换道距离

	double t0 = 0.0, t1 = 3.0;
	double x0, y0, x1, y1;
	double delta_t = 0.05;   //规划时间间隔
	int state = 0;

	Matrix A, B, X, Y, T;

};