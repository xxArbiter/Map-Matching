/******************************************************
*
*	Copyright (C) 2018, SJTU
*	All rights reserved.
*
*	@ version 1.0
*	@ author xxArbiter
*	@ date 2018.3.1 21:37
*
******************************************************/
#pragma once

#include "stdafx.h"

using namespace std;

/******************************************************
*
*	Struct:	�����
*
*	Intro:	��¼һ�� GPS �������
*
*	Members:@ x: ���� Longitude
*			@ y: γ�� Latitude
*
******************************************************/
struct Coordinate
{
	double x, y;

	Coordinate() {}
	Coordinate(double _x, double _y) :x(_x), y(_y) {}
	Coordinate(const Coordinate& coor) :x(coor.x), y(coor.y) {}

	// ������������������� ����ƽ�� ��λm
	double Distance(Coordinate &p) {
		return sqrt(pow(p.x*LNG2M - x*LNG2M, 2) + pow((p.y*LAT2M - y*LAT2M), 2));
	}

	// ������������������� ʹ�������� ��λm
	float gpsDistance(Coordinate &p) {
		double radLat1 = y*M_PI / 180;
		double radLat2 = p.y*M_PI / 180;
		double a = radLat1 - radLat2;
		double b = x*M_PI / 180 - p.x*M_PI / 180;
		double ans = 2 * asin(sqrt(pow(sin(a / 2), 2) + cos(radLat1)*cos(radLat2)*pow(sin(b / 2), 2)));
		ans *= 6378.137;
		return round(ans * 10000) / 10;
	}

	void set(double _x, double _y) {
		x = _x; y = _y;
	}

	Coordinate WGS2m(void) {
		Coordinate resCoor;
		resCoor.x = x*LNG2M;
		resCoor.y = y*LAT2M;
		return resCoor;
	}

	Coordinate m2WGS(void) {
		Coordinate resCoor;
		resCoor.x = x*M2LNG;
		resCoor.y = y*M2LAT;
		return resCoor;
	}

	// ���������
	Coordinate operator+(Coordinate &p) {
		Coordinate resCoor;
		resCoor.x = x + p.x;
		resCoor.y = y + p.y;
		return resCoor;
	}

	Coordinate operator-(Coordinate &p) {
		Coordinate resCoor;
		resCoor.x = x - p.x;
		resCoor.y = y - p.y;
		return resCoor;
	}

	Coordinate operator*(double a) {
		Coordinate resCoor;
		resCoor.x = x*a;
		resCoor.y = y*a;
		return resCoor;
	}

	double operator*(Coordinate &p) {
		return x*p.x + y*p.y;
	}
};

/******************************************************
*
*	Struct:	·��
*
*	Intro:	��¼һ�� GPS ���ĳ���ο���
*
*	Members:@ initN, termN: ��ʼ�� ��ֹ����
*			@ initP, termP: ��ʼ�� ��ֹ��
*			@ orien: ·�γ���� y ������Ϊ 0�� ˳ʱ�����
*			@ length: ·�γ��� ��λ m
*
******************************************************/
struct Segment
{
	int initN, termN;
	Coordinate initP, termP;
	float orien;
	float length;

public:
	Segment() {}

	Segment(int _initN, int _termN, Coordinate &coor1, Coordinate &coor2) :
		initN(_initN),
		termN(_termN),
		initP(coor1),
		termP(coor2)
	{
		orien = atan2(termP.x - initP.x, termP.y - initP.y) * 180 / M_PI;
		orien = orien >= 0 ? orien : orien + 360;
		length = initP.gpsDistance(termP);
	}
};

/******************************************************
*
*	Struct:	��С�������
*
*	Intro:	��¼һ��·�ε���С������� ���Ĳ�������ʹ��
*
*	Members:@ minX, maxX, minY, maxY: ��С������ζ˵�
*			@ num: ·�α��
*
******************************************************/
class GeoEnvelope
{
public:
	double minX;
	double maxX;
	double minY;
	double maxY;

private:
	int num;

public:
	GeoEnvelope();
	GeoEnvelope(double _minX, double _maxX, double _minY, double _maxY);
	GeoEnvelope(const GeoEnvelope& envelope);
	// ʹ��·�����˵��ʼ��
	GeoEnvelope(Coordinate *coord1, Coordinate *coord2, int _num);

	virtual ~GeoEnvelope(void);

	// ��þ��ε����ĵ�����
	Coordinate * Center() const;

	// �����Ƿ������һ��MBR
	bool IsContain(const GeoEnvelope &env) const;

	// �ж�������С��������Ƿ��ཻ(��������ڻ�����)
	bool IsCross(const GeoEnvelope &otherEvp) const;

	// �ж�һ�����Ƿ��ھ�����
	bool IsInMBR(const Coordinate &pt) const;

	// �ж�һ�����Ƿ��ھ�����
	bool IsInMBR(double x, double y) const;

	// ����·�α��
	int GetNum(void) const;

	// �ж�p1,p2���ɵľ��κ�q1,q2���ɵľ����Ƿ��ཻ
	static bool Intersects(Coordinate &p1, Coordinate &p2, Coordinate &q1, Coordinate &q2);
};