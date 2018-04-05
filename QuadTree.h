/******************************************************
*
*	Copyright (C) 2018, SJTU
*	All rights reserved.
*
*	@ file	QuadTree.cpp
*	@ bref	Declaration of class "QuadTreeNode".
*			1. Use geographic coordinate system, only consider the intersection of
*			northen hemisphere and eastern hemisphere.
*			2. The direction of positive x(y) axis is same as the direction of
*			eastern longitude(northen latitude)'s increasement.
*			3. This is not a full quad-tree, the nodes of tree are assignmented dynamically.
*			4. The details of objects which contained by quad-tree are described in "GeoEnvelop.h"
*
*	@ version 1.0
*	@ author xxArbiter
*	@ date 2018.3.1 21:37
*
******************************************************/
#pragma once  

#include "stdafx.h"
#include <list>

enum QuadType
{
	ROOT,         // ��  
	UP_RIGHT,     // ���ޢ�  
	UP_LEFT,      // ���ޢ�  
	BOTTOM_LEFT,  // ���ޢ�  
	BOTTOM_RIGHT  // ���ޢ�  
};

template <typename T>
class QuadTreeNode
{
public:
	QuadTreeNode(double _minX, double _minY, double _maxX, double _maxY, int _level, int _maxLevel, QuadType _quadType, QuadTreeNode *_parent);
	~QuadTreeNode();
public:
	void InsertObject(T *object); // �������  
	std::list<T *> GetObjectsAt(double _minX, double _minY, double _maxX, double _maxY); // ��ѯ����,���һƬ������Ķ��������˴�ֻ������ȫ������
	void RemoveObjectsAt(double _minX, double _minY, double _maxX, double _maxY); // ɾ������ɾ��һƬ������Ķ���ͽڵ㣬�˴�ֻ������ȫ������
	std::vector<int> GetNums(double x, double y, std::list<T *> &objects); // �Ӷ��������л�ȡ�������
	std::vector<int> GetObjectsContain(double x, double y); // ��ѯ���󣬻�ð���һ��������ж���ı������

private:
	bool IsContain(double _minX, double _minY, double _maxX, double _maxY, T *object) const; // �ж�ĳ�������Ƿ����ĳ����  
	bool IsContain(double _minX, double _minY, double _maxX, double _maxY, QuadTreeNode<T> *quadTreeNode) const; // ���أ��ж�ĳ�������Ƿ����ĳ���ڵ�

private:
	std::list<T *> objects;		// upObjects Ҷ�ڵ�ʱ�洢�ڵ�ȫ�����ݣ���Ҷ�ڵ�ʱ�洢�ϰ�������
	std::list<T *> leftObjects;	// ��Ҷ�ڵ�ʱ�洢���������
	std::list<T *> downObjects;	// ��Ҷ�ڵ�ʱ�洢�°�������
	std::list<T *> rightObjects;// ��Ҷ�ڵ�ʱ�洢�Ұ�������

	// �����ӽڵ㣬���ĸ�����  
	QuadTreeNode *parent;
	QuadTreeNode *upRightNode;
	QuadTreeNode *upLeftNode;
	QuadTreeNode *bottomLeftNode;
	QuadTreeNode *bottomRightNode;

	QuadType quadType;	// �ڵ�����

	double minX, minY, maxX, maxY;

	int level; // ��ǰ���
	int maxLevel; // ������
};