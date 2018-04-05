/******************************************************
*
*	Copyright (C) 2012, SJTU
*	All rights reserved.
*
*	@ file main.cpp
*	@ bref Use st-matching algorithm to match a sequence of points with higher precision.
*		   Use map-matching algorithm to match each point with higher speed.
*
*	@ version 1.0
*	@ author xxArbiter
*	@ date 2018.3.1 21:37
*
******************************************************/
#include "stdafx.h"
#include "Map-Matching.h"
#include "ST-Matching.h"

using namespace std;

int main(int argc, char *argv[])
{
	char *path;
	path = "D:\\2017 Taxi Data\\busList.csv";
	
	/*mapMatch mm(path, 0);
	vector<thread> threads;
	for (int i = 0; i < THREAD_NUM; ++i) {
		threads.push_back(thread([&]() {
			int threadName = i + 1;
			while (!mm.eol()) {
				mm.matching(threadName);
			}
		}));
	}
	for (auto &it : threads) {
		it.join();
	}*/

	clock_t t = clock();
	stMatch sm(path, atoi(argv[1]));
	while (!sm.eol()) {
		sm.matching();
	}
	float elapsed = (float)(clock() - t) / 1000;
	int hours = elapsed / 3600;
	elapsed -= hours * 3600;
	int minutes = elapsed / 60;
	elapsed -= minutes * 60;
	cout << "\r\n[Info] Program finished successfully. " << hours << " h " << minutes << " m " << elapsed << " s totally used." << endl;
	return 0;
}