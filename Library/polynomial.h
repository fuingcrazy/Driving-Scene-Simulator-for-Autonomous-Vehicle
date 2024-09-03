/*New Scene: apply polynomial to change lane*/
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
	double mat[N][N] = { 0 };   
	Matrix() = default;
	Matrix(int mm, int nn) {
		m = mm;
		n = nn;
	}
	void augMat(Matrix a, Matrix b);
	void createMat(double t0, double t1);    //create 6*6 matrice according to start-end time
	void createVector(double nm1, double nm2, double nm3, double nm4, double nm5, double nm6);    //x1,v1,a1;x2,v2,a2
	bool solve(Matrix a, Matrix b);   //solve linear equation
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
	void changeLane(Point pt, Point nextpt);    
public:
	unique_ptr<carNormal> car_obs;    
	double safe_dis = 500.0;    //safe distance
	double speed = -3.0;
	vector<Point> trackPoint;    //point set of track
	double changeLaneDis = 640;   

	double t0 = 0.0, t1 = 3.0;
	double x0, y0, x1, y1;
	double delta_t = 0.05;   //planning time interval
	int state = 0;

	Matrix A, B, X, Y, T;

};