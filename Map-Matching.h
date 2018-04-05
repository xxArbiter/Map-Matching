/******************************************************
*
*	Copyright (C) 2018, SJTU
*	All rights reserved.
*
*	@ file Map-Matching.h
*	@ bref Declaration of class "mapMatch".
*		   A naive map-matching algorithm which only consider one GPS point.
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

class mapMatch {
private:
	mutex flock;
	vector<string> files;
	int curFile;
	vector<Coordinate *> nodes;
	vector<GeoEnvelope *> MBRs;
	vector<Segment *> segments;
	QuadTreeNode<GeoEnvelope> *quadTree;

private:
	static FILE* loadFile(const char* path, const char* opt);
	static double p2sDistance(Coordinate &p, Segment &s);
	static int pointMatch(Coordinate &point, float orien, vector<int> cands, vector<Segment *> segs);

public:
	mapMatch(char* _path, int _startFile);
	bool eol(void) const;

public:
	void matching(int threadName);
	int pointMatch(double x, double y, float orien);
};
