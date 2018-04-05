/******************************************************
*
*	Copyright (C) 2018, SJTU
*	All rights reserved.
*
*	@ file Map-Matching.cpp
*	@ bref Definition of class "mapMatch".
*		   A naive map-matching algorithm which only consider one GPS point.
*
*	@ version 1.0
*	@ author xxArbiter
*	@ date 2018.3.1 21:37
*
******************************************************/
#include "stdafx.h"
#include "Map-Matching.h"

mapMatch::mapMatch(char* _path, int _startFile) :
	curFile(_startFile)
{
	ifstream fileList(_path);
	string tmp;
	while (!fileList.eof()) {
		fileList >> tmp;
		files.push_back(tmp);
	}
	fileList.close();

	clock_t t = clock();
	// 读取路网信息
	FILE *file;
	file = loadFile("D:\\2017 Taxi Data\\Map1\\cNode_cor.csv", "r");
	double x, y;
	int num;
	while (!feof(file)) {
		fscanf_s(file, "%d,%lf,%lf\n", &num, &x, &y);
		nodes.push_back(new Coordinate(x, y));
	}
	fclose(file);

	file = loadFile("D:\\2017 Taxi Data\\Map1\\cTurn.csv", "r");
	int a, b;
	while (!feof(file)) {
		fscanf_s(file, "%d,%d,%d\n", &num, &a, &b);
		segments.push_back(new Segment(a, b, *nodes[a - 1], *nodes[b - 1]));
		MBRs.push_back(new GeoEnvelope(nodes[a - 1], nodes[b - 1], num));
	}
	fclose(file);
	float elapsed = (float)(clock() - t) / CLOCKS_PER_SEC;
	cout << "Loading road network used: " << elapsed << "s" << endl;

	t = clock();
	// 建立四叉树
	quadTree = new QuadTreeNode<GeoEnvelope>(120.86227 - (THRESHOLD + 10) * M2LNG, 30.64229 - (THRESHOLD + 10) * M2LAT,
		122.05759 + (THRESHOLD + 10) * M2LNG, 31.85034 + (THRESHOLD + 10) * M2LAT, 1, 7, ROOT, nullptr);
	for (auto it : MBRs)
		quadTree->InsertObject(it);
	elapsed = (float)(clock() - t) / CLOCKS_PER_SEC;
	cout << "Building quad-tree used: " << elapsed << "s" << endl;
}

FILE* mapMatch::loadFile(const char* path, const char* opt) {
	FILE* file;
	errno_t err = fopen_s(&file, path, opt);
	while (err) {
		err = fopen_s(&file, path, opt);
		Sleep(0.1);
	}
	return file;
}

double mapMatch::p2sDistance(Coordinate &p, Segment &s) {
	Coordinate ab = s.termP - s.initP;
	Coordinate ac = p - s.initP;
	ab = ab.WGS2m();
	ac = ac.WGS2m();
	double f = ab*ac;
	if (f < 0) return p.Distance(s.initP);
	double d = ab*ab;
	if (f > d) return p.Distance(s.termP);
	f = f / d;
	Coordinate D = s.initP.WGS2m() + ab*f;
	return p.Distance(D.m2WGS());
}

int mapMatch::pointMatch(Coordinate &point, float orien, vector<int> cands, vector<Segment *> segs) {
	int bestMatch = 0;
	float bestScore = FLT_MAX, score;
	for (auto it : cands) {
		score = pow(p2sDistance(point, *(segs[it - 1]))/2, 2) + pow(abs(orien - segs[it - 1]->orien), 2);
		if (score < bestScore) {
			bestScore = score;
			bestMatch = it;
		}
	}
	return bestMatch;
}

int mapMatch::pointMatch(double x, double y, float orien) {
	vector<int> candidates = quadTree->GetObjectsContain(x, y);
	Coordinate point(x, y);
	int bestMatch = 0;
	float bestScore = FLT_MAX, score;
	for (auto it : candidates) {
		score = pow(p2sDistance(point, *(segments[it - 1]))/2, 2) + pow(abs(orien - segments[it - 1]->orien), 2);
		if (score < bestScore) {
			bestScore = score;
			bestMatch = it;
		}
		cout << it << "\t" << score << "\t" << p2sDistance(point, *(segments[it - 1]))/2 << "\t" << orien - segments[it - 1]->orien << endl;
	}
	return bestMatch;
}

void mapMatch::matching(int threadName) {
	flock.lock();
	int tmp = curFile;
	cout << "[Thread " << threadName << "] Parsing " << files[curFile] << endl;
	++curFile;
	flock.unlock();

	clock_t t = clock();
	// 查询
	FILE *file, *ofile;
	string &topo_file = string("D:\\2017 Taxi Data\\BUS Trajs\\") + files[tmp];
	file = loadFile(topo_file.c_str(), "r");
	topo_file = string("D:\\2017 Taxi Data\\BUS Trajs\\BUS match result\\") + files[tmp];
	ofile = loadFile(topo_file.c_str(), "w");
	float orien;
	double x, y;
	Coordinate tmpCor;
	while (!feof(file)) {
		fscanf_s(file, "%lf,%lf,%f\n", &x, &y, &orien);
		vector<int> candidates = quadTree->GetObjectsContain(x, y);
		tmpCor.x = x; tmpCor.y = y;
		fprintf_s(ofile, "%d\n", pointMatch(tmpCor, orien, candidates, segments));
	}
	fclose(file);
	fclose(ofile);
	float elapsed = (float)(clock() - t) / 1000;
	flock.lock();
	cout << "[Thread " << threadName << "] Parsed " << files[tmp] << " using: " << elapsed << "s" << endl;
	flock.unlock();
}

bool mapMatch::eol(void) const {
	return curFile >= files.size();
}