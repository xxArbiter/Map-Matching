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
	ROOT,         // 根  
	UP_RIGHT,     // 象限Ⅰ  
	UP_LEFT,      // 象限Ⅱ  
	BOTTOM_LEFT,  // 象限Ⅲ  
	BOTTOM_RIGHT  // 象限Ⅳ  
};

template <typename T>
class QuadTreeNode
{
public:
	QuadTreeNode(double _minX, double _minY, double _maxX, double _maxY, int _level, int _maxLevel, QuadType _quadType, QuadTreeNode *_parent);
	~QuadTreeNode();
public:
	void InsertObject(T *object); // 插入对象  
	std::list<T *> GetObjectsAt(double _minX, double _minY, double _maxX, double _maxY); // 查询对象,获得一片区域里的对象链表，此处只考虑完全包含的
	void RemoveObjectsAt(double _minX, double _minY, double _maxX, double _maxY); // 删除对象，删除一片区域里的对象和节点，此处只考虑完全包含的
	std::vector<int> GetNums(double x, double y, std::list<T *> &objects); // 从对象链表中获取编号向量
	std::vector<int> GetObjectsContain(double x, double y); // 查询对象，获得包含一个点的所有对象的编号向量

private:
	bool IsContain(double _minX, double _minY, double _maxX, double _maxY, T *object) const; // 判断某个区域是否包含某对象  
	bool IsContain(double _minX, double _minY, double _maxX, double _maxY, QuadTreeNode<T> *quadTreeNode) const; // 重载，判断某个区域是否包含某个节点

private:
	std::list<T *> objects;		// upObjects 叶节点时存储节点全部数据，非叶节点时存储上半轴数据
	std::list<T *> leftObjects;	// 非叶节点时存储左半轴数据
	std::list<T *> downObjects;	// 非叶节点时存储下半轴数据
	std::list<T *> rightObjects;// 非叶节点时存储右半轴数据

	// 父、子节点，分四个象限  
	QuadTreeNode *parent;
	QuadTreeNode *upRightNode;
	QuadTreeNode *upLeftNode;
	QuadTreeNode *bottomLeftNode;
	QuadTreeNode *bottomRightNode;

	QuadType quadType;	// 节点类型

	double minX, minY, maxX, maxY;

	int level; // 当前深度
	int maxLevel; // 最大深度
};