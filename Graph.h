#pragma once

#include "stdafx.h"
#include "GeoEnvelop.h"

using namespace std;

struct ArcNode		// ת����Ϊ����ͼ����
{
	int Num;
	ArcNode *next;
	ArcNode(int x) : Num(x), next(NULL) {}
};

struct Road	// ·����Ϊ����ͼ�ڵ�
{
	int Num, iDegree;
	Coordinate oJunc, tJunc;	// ��ʼ����ֹ·��
	float Orien;				// ����� ���ָ���յ�
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


