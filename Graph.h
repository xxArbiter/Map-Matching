#pragma once

#include "stdafx.h"
#include "GeoEnvelop.h"

using namespace std;

struct ArcNode		// 转弯作为有向图连边
{
	int Num;
	ArcNode *next;
	ArcNode(int x) : Num(x), next(NULL) {}
};

struct Road	// 路段作为有向图节点
{
	int Num, iDegree;
	Coordinate oJunc, tJunc;	// 起始、终止路口
	float Orien;				// 朝向角 起点指向终点
	ArcNode *firArc;
	Road(int x, Coordinate a, Coordinate b) : Num(x), iDegree(0), oJunc(a), tJunc(b), firArc(NULL) {}
};

class Graph
{
public:
	Graph();
	~Graph();

private:
	int RoadNum;

};

Graph::Graph()
{
}

Graph::~Graph()
{
}


