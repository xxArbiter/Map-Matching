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
#include "stdafx.h"
#include "GeoEnvelop.h"

GeoEnvelope::GeoEnvelope() {}

GeoEnvelope::GeoEnvelope(double _minX, double _maxX, double _minY, double _maxY) : num(0) {
	if (_minX > _maxX) {
		minX = _maxX;
		maxX = _minX;
	}
	else {
		minX = _minX;
		maxX = _maxX;
	}

	if (_minY > _maxY) {
		minY = _maxY;
		maxY = _minY;
	}
	else {
		minY = _minY;
		maxY = _maxY;
	}
}

GeoEnvelope::GeoEnvelope(const GeoEnvelope& envelope) {
	this->minX = envelope.minX;
	this->maxX = envelope.maxX;
	this->minY = envelope.minY;
	this->maxY = envelope.maxY;
	this->num = envelope.num;
}

// ʹ��·�����˵��ʼ��
GeoEnvelope::GeoEnvelope(Coordinate *coord1, Coordinate *coord2, int _num) : num(_num) {
	if (coord1->x < coord2->x) {
		minX = coord1->x - THRESHOLD * M2LNG;
		maxX = coord2->x + THRESHOLD * M2LNG;
	}
	else {
		minX = coord2->x - THRESHOLD * M2LNG;
		maxX = coord1->x + THRESHOLD * M2LNG;
	}

	if (coord1->y < coord2->y) {
		minY = coord1->y - THRESHOLD * M2LAT;
		maxY = coord2->y + THRESHOLD * M2LAT;
	}
	else {
		minY = coord2->y - THRESHOLD * M2LAT;
		maxY = coord1->y + THRESHOLD * M2LAT;
	}
}

GeoEnvelope::~GeoEnvelope(void) {}

// ��þ��ε����ĵ�����
Coordinate * GeoEnvelope::Center() const {
	return new Coordinate((minX + maxX) / 2, (minY + maxY) / 2);
}

// �Ƿ������һ��MBR
bool GeoEnvelope::IsContain(const GeoEnvelope &env) const {
	return env.minX > minX &&
		env.maxX < maxX &&
		env.minY > minY &&
		env.maxY < maxY;
}

bool GeoEnvelope::IsCross(const GeoEnvelope & otherEvp) const {
	// �ھ�����
	if (maxX < otherEvp.minX || minX > otherEvp.maxX ||
		maxY < otherEvp.minY || minY > otherEvp.maxY)
	{
		return 0;
	}

	// �ھ�����
	if (minX >= otherEvp.minX && maxX <= otherEvp.maxX &&
		minY >= otherEvp.minY && maxY <= otherEvp.maxY)
	{
		return 0;
	}

	return 1;	//�ཻ
}

// �ж�һ�����Ƿ��ڸþ�����
bool GeoEnvelope::IsInMBR(const Coordinate &pt) const {
	if (pt.x > minX && pt.x < maxX && pt.y > minY && pt.y < maxY) {
		return 1;
	}
	return 0;
}

// �ж�һ�����Ƿ��ڸþ�����
bool GeoEnvelope::IsInMBR(double x, double y) const {
	if (x > minX && x < maxX && y > minY && y < maxY) {
		return 1;
	}
	return 0;
}

int GeoEnvelope::GetNum(void) const {
	return num;
}

//�ж�p1,p2���ɵľ��κ�q1,q2���ɵľ����Ƿ��ཻ
bool GeoEnvelope::Intersects(Coordinate &p1, Coordinate &p2, Coordinate &q1, Coordinate &q2) {
	double minq = min(q1.x, q2.x);
	double maxq = max(q1.x, q2.x);
	double minp = min(p1.x, p2.x);
	double maxp = max(p1.x, p2.x);

	if (minp > maxq)
		return false;
	if (maxp < minq)
		return false;

	minq = min(q1.y, q2.y);
	maxq = max(q1.y, q2.y);
	minp = min(p1.y, p2.y);
	maxp = max(p1.y, p2.y);

	if (minp > maxq)
		return false;
	if (maxp < minq)
		return false;
	return true;
}

