/******************************************************
*	
*	Copyright (C) 2018, SJTU
*	All rights reserved.
*
*	@ file ST-Matching.cpp
*	@ bref Definition of class "stMatch".
*		   A map-matching algorithm for hole sequence of GPS points.
*		   Algorithm from: Yin Lou, e.t. Map-Matching for Low-Sampling-Rate GPS Trajectories. SIG SPATIAL 2009.
*
*	@ version 1.0
*	@ author xxArbiter
*	@ date 2018.3.1 21:37
*	
******************************************************/
#include "stdafx.h"
#include "ST-Matching.h"

/******************************************************
*	Func:	计算点到线段的距离 以垂线最短距离优先
*			若点在线段的投影在两端点以外 计算点到端点距离
*	Paras:	@ p: GPS 点
*			@ s: 路段
*			@ pos: 引用传参 记录投影点距离路段起始点的距离 表征该投影点在路段上的位置
*	Return: @ dis: GPS 点到路段的距离 单位 m
******************************************************/
float stMatch::p2sDistance(Coordinate &p, Segment &s, float &pos) {
	Coordinate ab = s.termP - s.initP;
	Coordinate ac = p - s.initP;
	ab = ab.WGS2m();
	ac = ac.WGS2m();
	double f = ab*ac;
	pos = 0;
	if (f < 0) return p.Distance(s.initP);
	double d = ab*ab;
	pos = s.length;
	if (f > d) return p.Distance(s.termP);
	pos = f / s.length;
	f /= d;
	Coordinate D = s.initP.WGS2m() + ab*f;
	return p.Distance(D.m2WGS());
}

/******************************************************
*	Func:	计算一个 GPS 点误差半径内所有候选点 Cs_i
*	Paras:	@ p: GPS 点
*			@ ori: GPS 点朝向角
*	Return: @ candidates: 误差半径内的所有候选点
******************************************************/
vector<candPoint> stMatch::getCandidates(Coordinate &p, float ori) {
	vector<candPoint> ans;
	vector<int> inMBR = quadTree->GetObjectsContain(p.x, p.y);
	candPoint tmp;
	for (auto it : inMBR) {
		tmp.seg = it;
		double dis = p2sDistance(p, *(segments[it]), tmp.pos);
		if (dis <= THRESHOLD) {
			tmp.getN(dis, ori - segments[it]->orien);
			ans.push_back(tmp);
		}
	}
	return ans;
}

/******************************************************
*	Func:	安全装载文件
*	Paras:	@ path: 目标文件路径
*			@ opt: 操作类型
*	Return: @ handle: 文件句柄
******************************************************/
FILE* stMatch::loadFile(const char* path, const char* opt) {
	FILE* file;
	errno_t err = fopen_s(&file, path, opt);
	while (err) {
		err = fopen_s(&file, path, opt);
		Sleep(0.1);
	}
	return file;
}

/******************************************************
*	Func:	solution 类构造
*	Paras:	@ _path: 待处理文件的列表文件路径
*			@ _startFile: 跳过文件数 由第 _startFile 个文件开始处理
******************************************************/
stMatch::stMatch(string _path, int _startFile) :
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
	cout << "[Log] Loading road network...";
	file = loadFile("D:\\2017 Taxi Data\\Map-Shanghai\\LD\\nodes.csv", "r");
	double x, y;
	int num;
	while (!feof(file)) {
		fscanf_s(file, "%d,%lf,%lf\n", &num, &x, &y);
		nodes.push_back(new Coordinate(x, y));
	}
	fclose(file);

	file = loadFile("D:\\2017 Taxi Data\\Map-Shanghai\\LD\\segments.csv", "r");
	int a, b;
	while (!feof(file)) {
		fscanf_s(file, "%d,%d,%d\n", &num, &a, &b);
		segments.push_back(new Segment(a, b, *nodes[a], *nodes[b]));
		MBRs.push_back(new GeoEnvelope(nodes[a], nodes[b], num));
	}
	fclose(file);

	// Generate out-degree graph
	map<int, vector<int>> m;
	for (int i = 0; i < segments.size(); ++i) m[segments[i]->initN].push_back(i);
	for (int i = 0; i < segments.size(); ++i) outGraph.push_back(m[segments[i]->termN]);

	float elapsed = (float)(clock() - t) / CLOCKS_PER_SEC;
	cout << "\r[Info] Road network loaded successfully! Used: " << elapsed << "s" << endl;

	t = clock();
	// 建立四叉树
	cout << "[Log] Generating quad-tree...";
	quadTree = new QuadTreeNode<GeoEnvelope>(120.86227 - (THRESHOLD + 10) * M2LNG, 30.64229 - (THRESHOLD + 10) * M2LAT,
		122.05759 + (THRESHOLD + 10) * M2LNG, 31.85034 + (THRESHOLD + 10) * M2LAT, 1, 7, ROOT, nullptr);
	for (auto it : MBRs)
		quadTree->InsertObject(it);
	elapsed = (float)(clock() - t) / CLOCKS_PER_SEC;
	cout << "\r[Info] Quad-tree generated successfully! Used: " << elapsed << "s" << endl;
}

/******************************************************
 *	Func:	求两 candidate 点间最短路径和图上距离
 *	Paras:	@ p1: 起始参考点
 *			@ p2: 终止参考点
 *			@ srtPath: 引用传递 保存计算得到的最短路径
 *	Return: @ dis: 两参考点间路网实际距离
 *	Todo: 优先队列优化
******************************************************/
float stMatch::dijkstra(const candPoint &p1, const candPoint &p2, vector<int> &srtPath) {
	int i = p1.seg, j = p2.seg;
	int sz = segments.size();
	if (i == j) return (p2.pos - p1.pos >= 0) ? (p2.pos - p1.pos) : ((p1.pos - p2.pos) * 2);
	//if (i == j) return abs(p1.pos - p2.pos);
	float ans = segments[i]->length - p1.pos + p2.pos;
	if (segments[i]->termN == segments[j]->initN) {
		srtPath.push_back(j);
		return ans;
	}
	vector<bool> *h = new vector<bool>(sz, false);	// Already get the shortest path or not.
	vector<int> visited;							// Visited or not.
	vector<float> *dist = new vector<float>(sz, FLT_MAX);
	vector<int> *path = new vector<int>(sz, i);

	// Initialize
	for (auto it : outGraph[i]) {
		(*dist)[it] = 0;
		visited.push_back(it);
	}

	// Dijkstra
	float minCost, tmpCost;
	int minCostIndex;
	for (int k = 0; k < min(sz - 1, 1000) && (*h)[j] == false; ++k) {
		minCost = FLT_MAX;
		// Find the minimun weight in all visited vertexs which still have no solution.
		for (auto it : visited) {
			if ((*h)[it] == false && (*dist)[it] < minCost) {
				minCost = (*dist)[it];
				minCostIndex = it;
			}
		}
		(*h)[minCostIndex] = true;
		tmpCost = minCost + segments[minCostIndex]->length;
		for (auto iit : outGraph[minCostIndex]) {
			if (find(visited.begin(), visited.end(), iit) == visited.end()) visited.push_back(iit);
			if ((*h)[iit] == false && tmpCost < (*dist)[iit]) {
				(*dist)[iit] = tmpCost;
				(*path)[iit] = minCostIndex;
			}
		}
	}

	// Generate solution
	ans += (*dist)[j];
	srtPath.clear();
	if ((*h)[j] == false) return FLT_MAX;
	while (minCostIndex != i) {
		srtPath.push_back(minCostIndex);
		minCostIndex = (*path)[minCostIndex];
	}
	reverse(srtPath.begin(), srtPath.end());

	delete h;
	delete dist;
	delete path;
	return ans;
}

/******************************************************
*	Func:	匹配一个文件记录的路径 即一整条序列
*
*	Modify:	2018.3.2	Arbiter
*			删除折返路段 例: 123 -> 250 -> 251 -> 321 删除 250 -> 251
******************************************************/
void stMatch::matching(void) {
	FILE *file;
	string &topo_file = string("D:\\2017 Taxi Data\\BUS Trajs\\") + files[curFile];
	file = loadFile(topo_file.c_str(), "r");
	cout << "[Info] Processing " << files[curFile] << "...";

	// Load file and generate candidate graph
	clock_t t = clock();
	float orien;
	double x, y;
	Coordinate tmpCor;
	vector<candPoint> tmpCands;
	vector<vector<candPoint>> candGraph;
	vector<Coordinate> trajectory;
	vector<int> candNum;
	while (!feof(file)) {
		fscanf_s(file, "%lf,%lf,%f\n", &x, &y, &orien);
		tmpCor.set(x, y);
		tmpCands = getCandidates(tmpCor, orien);
		if (tmpCands.size() != 0) {
			candGraph.push_back(tmpCands);
			candNum.push_back(tmpCands.size());
			trajectory.push_back(tmpCor);
		}
	}
	fclose(file);
	for (auto it : candGraph) candNum.push_back(it.size());
	for (auto &it : candGraph[0]) it.bestPath.push_back(it.seg);
	float elapsed = (float)(clock() - t) / 1000;
	
	// Calculate parents
	vector<float> V;
	vector<vector<int>> paths;
	vector<int> path;
	float d, F, bestF;
	int bestFIndex;
	int sz = candGraph.size();
	// 逐层计算 parent
	for (int i = 1; i < sz; ++i) {
		d = trajectory[i].gpsDistance(trajectory[i - 1]);
		for (int j = 0; j < candNum[i]; ++j) {
			V.clear(); paths.clear();
			for (int k = 0; k < candNum[i - 1]; ++k) {
				path.clear();
				float dij = dijkstra(candGraph[i - 1][k], candGraph[i][j], path);
				V.push_back(dij < d ? 1 : d / dij);
				paths.push_back(path);

				/*cout << "[Log] Path: " << candGraph[i - 1][k].seg << " -> " << candGraph[i][j].seg << ", N1 score: " << candGraph[i - 1][k].N << ", N2 score: " << candGraph[i][j].N << endl;
				cout << "[Log] Direct distance: " << d << ", Dij distance: " << dij << ", F score: " << candGraph[i - 1][k].N + candGraph[i][j].N*(d/dij) << ", V score: " << d/dij << endl;
				cout << endl;*/

				/*cout << candGraph[i - 1][k].seg << " " << candGraph[i][j].seg << " " << dij << endl;
				for (auto it : path) cout << it << " " << segments[it]->length << endl;
				cout << endl;*/
			}
			bestF = 0;
			// 找到最大的 F 值，所在参考点即其 parent
			for (int k = 0; k < candNum[i - 1]; ++k) {
				F = candGraph[i - 1][k].N + candGraph[i][j].N*V[k];
				if (F > bestF) {
					bestF = F;
					bestFIndex = k;
				}
			}
			candGraph[i][j].N = bestF;
			candGraph[i][j].parent = bestFIndex;
			candGraph[i][j].bestPath = paths[bestFIndex];
		}
		// Schedule display.
		elapsed = (float)(clock() - t) / 1000;
		int j;
		cout << "\r[Log] [Processing " << files[curFile] <<  ": ";
		for (j = 0; j < (i * 20) / sz; ++j) cout << ">";
		for (; j < 20; ++j) cout << "-";
		cout << "] Calculated " << i << " points' parent. Used: " << elapsed << "s\t";
	}
	// 反向寻求最优解
	vector<candPoint *> solution(sz, nullptr);
	path.clear(); paths.clear();
	bestF = 0;
	for (int i = 0; i < candNum[sz - 1]; ++i)
		if (candGraph[sz - 1][i].N > bestF) {
			bestF = candGraph[sz - 1][i].N;
			solution[sz - 1] = &candGraph[sz - 1][i];
		}
	for (int i = sz - 2; i >= 0; --i) solution[i] = &candGraph[i][solution[i + 1]->parent];
	for (auto it : solution) path.insert(path.end(), it->bestPath.begin(), it->bestPath.end());

	// 删除折返路段
	for (int i = path.size() - 1; i > 0; --i) {
		if (path[i] % 2 == 0 && path[i - 1] == path[i] + 1 || path[i] % 2 == 1 && path[i - 1] == path[i] - 1) {
			path.erase(path.begin() + i - 1, path.begin() + i + 1);
			i--;
		}
	}

	topo_file = string("D:\\2017 Taxi Data\\BUS Trajs\\st match result\\") + files[curFile];
	file = loadFile(topo_file.c_str(), "w");
	for (auto it : path) fprintf_s(file, "%d\n", it);
	fclose(file);

	//for (int i = 0; i < sz; ++i) {
	//	/*cout << "Candidate Points: ";
	//	for (auto it : candGraph[i]) cout << it.seg << "\t";
	//	cout << endl;
	//	cout << "Score of stMatch: ";
	//	for (auto it : candGraph[i]) cout << it.N << '\t';
	//	cout << endl;
	//	cout << solution[i]->seg << "\t" << solution[i]->pos << '\t';
	//	for (auto iit : solution[i]->bestPath) cout << iit << '\t';
	//	cout << "\r\n" << endl;*/

	//	cout << solution[i]->seg << ":\t";
	//	for (auto it : solution[i]->bestPath) cout << it << "\t";
	//	cout << "\r\n" << endl;
	//}

	elapsed = (float)(clock() - t) / 1000;
	cout << "\r[Info] " << files[curFile] << " processed successfully! Used: " << elapsed << "s\t\t\t\t\t\t\t\t" << endl;
	curFile++;
}

/******************************************************
*	Func:	是否匹配至文件列表末端
*	Return: @ flag: 是否匹配结束
******************************************************/
bool stMatch::eol(void) {
	return curFile >= files.size();
}