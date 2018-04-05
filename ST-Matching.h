/******************************************************
*
*	Copyright (C) 2018, SJTU
*	All rights reserved.
*
*	@ file ST-Matching.h
*	@ bref Declaration of class "stMatch" and definition of struct "candidatePoint".
*		   A map-matching algorithm for hole sequence of GPS points.
*		   Algorithm from: Yin Lou, e.t. Map-Matching for Low-Sampling-Rate GPS Trajectories. SIG SPATIAL 2009.
*
*	@ version 1.0
*	@ author xxArbiter
*	@ date 2018.3.1 21:37
*
******************************************************/
#pragma once

#include "stdafx.h"
#include "GeoEnvelop.h"
#include "QuadTree.h"
#include "QuadTree.cpp"

/******************************************************
*
*	Struct:	参考点
*
*	Intro:	记录一个 GPS 点的某个参考点
*
*	Members:@ seg: 该参考点所在路段
*			@ parent: 记录 ST-Matching 得到的该点父参考点在参考点集中的位置
*			@ pos: 该参考点距离所在路段起始点的距离 表征该参考点在路段上的位置
*			@ N: 该点观测可信度 在 ST-Matching 过程中除第一层参考点外 N 改为存储 st-权重
*			@ sigmaD, sigmaO: 正态分布方差 分别对应距离与角度
*			@ bestPath: 父参考点到该参考点的最短路径
*
******************************************************/
struct candPoint
{
	int seg, parent;
	float pos, N;
	float sigmaD = 20, sigmaO = 20;
	vector<int> bestPath;

	candPoint() :seg(0), pos(0), parent(0) {}

	candPoint(int _seg, double _pos) :seg(_seg), pos(_pos), parent(0) {}

	// 计算该参考点的观测可信度 N
	void getN(float dis, float ori) {
		N = exp(-dis*dis / (2 * sigmaD*sigmaD)) / (sqrt(2 * M_PI)*sigmaD);
		//N *= exp(ori*ori / (2 * sigmaO*sigmaO)) / (sqrt(2 * M_PI)*sigmaO);
	}
};

/******************************************************
*
*	Class:	ST-Matching Solution
*
*	Intro:	ST-Matching 解决方案类 算法来自论文 Y. Lou, e.t. SIG SPATIAL 2009
*
*	Members:@ files: 待处理文件列表
*			@ curFile: 已处理文件数量
*			@ nodes: 路网 - 节点列表 向量中位置即该节点编号
*			@ segments: 路网 - 路段列表 向量中位置即该路段编号
*			@ MBRs: 最小外包矩形列表 存储所有路段的外包矩形及其编号 以构建四叉树
*			@ outGraph: 参考点间的权重图 具体见论文
*			@ quadtree: 四叉树根节点
*			@ candidates: 所有 GPS 点的参考点集
*
******************************************************/
class stMatch {
private:
	vector<string> files;
	int curFile;
	vector<Coordinate *> nodes;
	vector<GeoEnvelope *> MBRs;
	vector<Segment *> segments;
	vector<vector<int>> outGraph;
	QuadTreeNode<GeoEnvelope> *quadTree;
	vector<vector<candPoint>> candidates;

private:
	static FILE* loadFile(const char* path, const char* opt);
	float p2sDistance(Coordinate &p, Segment &s, float &pos);
	vector<candPoint> getCandidates(Coordinate &p, float ori);
	float dijkstra(const candPoint &p1, const candPoint &p2, vector<int> &srtPath);


public:
	stMatch(string _path, int _startFile);
	void matching(void);
	bool eol(void);
};