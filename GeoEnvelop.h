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
*	Struct:	坐标点
*
*	Intro:	记录一个 GPS 点的坐标
*
*	Members:@ x: 经度 Longitude
*			@ y: 纬度 Latitude
*
******************************************************/
struct Coordinate
{
	double x, y;

	Coordinate() {}
	Coordinate(double _x, double _y) :x(_x), y(_y) {}
	Coordinate(const Coordinate& coor) :x(coor.x), y(coor.y) {}

	// 根据两坐标点计算点间距离 假设平面 单位m
	double Distance(Coordinate &p) {
		return sqrt(pow(p.x*LNG2M - x*LNG2M, 2) + pow((p.y*LAT2M - y*LAT2M), 2));
	}

	// 根据两坐标点计算点间距离 使用球坐标 单位m
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

	// 运算符重载
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
*	Struct:	路段
*
*	Intro:	记录一个 GPS 点的某个参考点
*
*	Members:@ initN, termN: 起始点 终止点编号
*			@ initP, termP: 起始点 终止点
*			@ orien: 路段朝向角 y 轴向上为 0° 顺时针计数
*			@ length: 路段长度 单位 m
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
*	Struct:	最小外包矩形
*
*	Intro:	记录一个路段的最小外包矩形 供四叉树索引使用
*
*	Members:@ minX, maxX, minY, maxY: 最小外包矩形端点
*			@ num: 路段编号
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
	// 使用路段两端点初始化
	GeoEnvelope(Coordinate *coord1, Coordinate *coord2, int _num);

	virtual ~GeoEnvelope(void);

	// 获得矩形的中心点坐标
	Coordinate * Center() const;

	// 测试是否包含另一个MBR
	bool IsContain(const GeoEnvelope &env) const;

	// 判断两个最小外包矩形是否相交(计算出在内还在外)
	bool IsCross(const GeoEnvelope &otherEvp) const;

	// 判断一个点是否在矩形内
	bool IsInMBR(const Coordinate &pt) const;

	// 判断一个点是否在矩形内
	bool IsInMBR(double x, double y) const;

	// 返回路段编号
	int GetNum(void) const;

	// 判断p1,p2构成的矩形和q1,q2构成的矩形是否相交
	static bool Intersects(Coordinate &p1, Coordinate &p2, Coordinate &q1, Coordinate &q2);
};