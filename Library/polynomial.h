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
	double mat[N][N] = { 0 };   //Ĭ�Ͼ���ά��
	Matrix() = default;
	Matrix(int mm, int nn) {
		m = mm;
		n = nn;
	}
	void augMat(Matrix a, Matrix b);
	void createMat(double t0, double t1);    //����ʼĩʱ�䴴��6*6����
	void createVector(double nm1, double nm2, double nm3, double nm4, double nm5, double nm6);    //����һ��������ʾ����״̬
	bool solve(Matrix a, Matrix b);   //������Է�����
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
	void changeLane(Point pt, Point nextpt);    //������Ƕ�
public:
	unique_ptr<carNormal> car_obs;    //�ϰ��ﳵ��
	double safe_dis = 500.0;    //����ȫ����
	double speed = -3.0;
	vector<Point> trackPoint;    //�켣�㼯
	double changeLaneDis = 640;    //���򻻵�����

	double t0 = 0.0, t1 = 3.0;
	double x0, y0, x1, y1;
	double delta_t = 0.05;   //�滮ʱ����
	int state = 0;

	Matrix A, B, X, Y, T;

};