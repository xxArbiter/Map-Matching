/******************************************************
*
*	Copyright (C) 2018, SJTU
*	All rights reserved.
*
*	@ file QuadTree.cpp
*	@ bref Definition of class "QuadTreeNode".
*		   A quad-tree can accelerate the seeking speed of MBR which contains a road segment.
*
*	@ version 1.0
*	@ author xxArbiter
*	@ date 2018.3.1 21:37
*
******************************************************/
#include "stdafx.h"
#include "QuadTree.h"  

template <typename T>
QuadTreeNode<T>::QuadTreeNode(
	double _minX, double _minY, double _maxX, double _maxY,
	int _level, int _maxLevel,
	QuadType _quadType,
	QuadTreeNode *_parent) :
	minX(_minX),
	minY(_minY),
	maxX(_maxX),
	maxY(_maxY),
	level(_level),
	maxLevel(_maxLevel),
	quadType(_quadType)
{
	parent = _parent;
	upLeftNode = nullptr;
	upRightNode = nullptr;
	bottomLeftNode = nullptr;
	bottomRightNode = nullptr;
}

template <typename T>
QuadTreeNode<T>::~QuadTreeNode()
{
	if (level == maxLevel)
		return;
	//如果不是叶子节点，就销毁子节点  
	parent = nullptr;

}

template <typename T>
bool QuadTreeNode<T>::IsContain(double _minX, double _minY, double _maxX, double _maxY, T *object) const
{
	if (object->minX > _minX
		&&object->maxX < _maxX
		&&object->minY > _minY
		&&object->maxY < _maxY)
		return true;
	return false;
}

template <typename T>
bool QuadTreeNode<T>::IsContain(double _minX, double _minY, double _maxX, double _maxY, QuadTreeNode<T> *quadTreeNode) const
{
	if (quadTreeNode->minX > _minX
		&&quadTreeNode->maxX < _maxX
		&&quadTreeNode->minY > _minY
		&&quadTreeNode->maxY < _maxY)
		return true;
	return false;
}

template <typename T>
void QuadTreeNode<T>::InsertObject(T *object)
{
	//如果是叶子节点，则存放在叶子节点  
	if (level == maxLevel)
	{
		objects.push_back(object);
		return;
	}

	//非叶子节点，如果下层节点可以包含该对象，则递归构建子节点并插入对象,边构建边插入
	if (IsContain((minX + maxX) / 2, (minY + maxY) / 2, maxX, maxY, object))
	{
		if (!upRightNode) //避免重复创建覆盖掉原来的节点  
			upRightNode = new QuadTreeNode((minX + maxX) / 2, (minY + maxY) / 2, maxX, maxY, level + 1, maxLevel, UP_RIGHT, this);//如果没有子节点就创建子节点，parent节点是当前节点  
		upRightNode->InsertObject(object);
		return;
	}
	if (IsContain(minX, (minY + maxY) / 2, (minX + maxX) / 2, maxY, object))
	{
		if (!upLeftNode)
			upLeftNode = new QuadTreeNode(minX, (minY + maxY) / 2, (minX + maxX) / 2, maxY, level + 1, maxLevel, UP_LEFT, this);
		upLeftNode->InsertObject(object);
		return;
	}
	if (IsContain(minX, minY, (minX + maxX) / 2, (minY + maxY) / 2, object))
	{
		if (!bottomLeftNode)
			bottomLeftNode = new QuadTreeNode(minX, minY, (minX + maxX) / 2, (minY + maxY) / 2, level + 1, maxLevel, BOTTOM_LEFT, this);
		bottomLeftNode->InsertObject(object);
		return;
	}
	if (IsContain((minX + maxX) / 2, minY, maxX, (minY + maxY) / 2, object))
	{
		if (!bottomRightNode)
			bottomRightNode = new QuadTreeNode((minX + maxX) / 2, minY, maxX, (minY + maxY) / 2, level + 1, maxLevel, BOTTOM_RIGHT, this);
		bottomRightNode->InsertObject(object);
		return;
	}
	//下层节点不能完全包含改对象，则插入到当前非叶子节点，并区分四个半区存储
	if ((minX + maxX) / 2 < object->maxX && (minX + maxX) / 2 > object->minX) {
		if (object->minY >= (minY + maxY) / 2) objects.push_back(object);
		else downObjects.push_back(object);
	}
	else {
		if (object->maxX < (minX + maxX) / 2) leftObjects.push_back(object);
		else rightObjects.push_back(object);
	}
	return;
}

template <typename T>
std::list<T *> QuadTreeNode<T>::GetObjectsAt(double _minX, double _minY, double _maxX, double _maxY)
{
	std::list<T *> resObjects;
	//如果当前节点完全被包含，把当前节点存的对象放到列表末尾,空链表也行  
	if (IsContain(_minX, _minY, _maxX, _maxY, this))
	{
		resObjects.insert(resObjects.end(), objects.begin(), objects.end());
		//最后一层  
		if (level == maxLevel)
			return resObjects;
	}

	//如果有下层节点就把下层节点包含的对象加进来  
	if (upRightNode)
	{
		std::list<T *> upRightChild;
		upRightChild = upRightNode->GetObjectsAt(_minX, _minY, _maxX, _maxY);
		resObjects.insert(resObjects.end(), upRightChild.begin(), upRightChild.end());
	}
	if (upLeftNode)
	{
		std::list<T *> upLeftChild;
		upLeftChild = upLeftNode->GetObjectsAt(_minX, _minY, _maxX, _maxY);
		resObjects.insert(resObjects.end(), upLeftChild.begin(), upLeftChild.end());
	}
	if (bottomLeftNode)
	{
		std::list<T *> bottomLeftChild;
		bottomLeftChild = bottomLeftNode->GetObjectsAt(_minX, _minY, _maxX, _maxY);
		resObjects.insert(resObjects.end(), bottomLeftChild.begin(), bottomLeftChild.end());
	}
	if (bottomRightNode)
	{
		std::list<T *> bottomRightChild;
		bottomRightChild = bottomRightNode->GetObjectsAt(_minX, _minY, _maxX, _maxY);
		resObjects.insert(resObjects.end(), bottomRightChild.begin(), bottomRightChild.end());
	}
	return resObjects;
}

template <typename T>
void QuadTreeNode<T>::RemoveObjectsAt(double _minX, double _minY, double _maxX, double _maxY)
{
	//如果本层节点被包含则删除本层节点的对象  
	//这个判断主要是对根节点起作用，其他子节点实际在上层都做了判断  
	if (IsContain(_minX, _minY, _maxX, _maxY, this))
	{
		//清除本节点层的对象  
		objects.clear();
		//最后一层  
		if (level == maxLevel)
			return;

	}
	//如果有子节点且被包含就销毁子节点，注意别产生野指针  
	//其实只要上层被包含了，下层肯定被包含，代码还需改进  
	if (upRightNode&&IsContain(_minX, _minY, _maxX, _maxY, upRightNode))
	{
		upRightNode->RemoveObjectsAt(_minX, _minY, _maxX, _maxY);
		delete upRightNode;
		upRightNode = nullptr;

	}
	if (upLeftNode&&IsContain(_minX, _minY, _maxX, _maxY, upLeftNode))
	{
		upLeftNode->RemoveObjectsAt(_minX, _minY, _maxX, _maxY);
		delete upLeftNode;
		upLeftNode = nullptr;

	}
	if (bottomLeftNode&&IsContain(_minX, _minY, _maxX, _maxY, bottomLeftNode))
	{
		bottomLeftNode->RemoveObjectsAt(_minX, _minY, _maxX, _maxY);
		delete bottomLeftNode;
		bottomLeftNode = nullptr;

	}
	if (bottomRightNode&&IsContain(_minX, _minY, _maxX, _maxY, bottomRightNode))
	{
		bottomRightNode->RemoveObjectsAt(_minX, _minY, _maxX, _maxY);
		delete bottomRightNode;
		bottomRightNode = nullptr;
	}
}

template <typename T>
std::vector<int> QuadTreeNode<T>::GetNums(double x, double y, std::list<T *> &objects) {
	std::vector<int> resNums;
	std::list<T *>::iterator it = objects.begin();
	while (it != objects.end()) {
		// 只给出包含点的MBR编号
		if ((*it)->IsInMBR(x, y))
			resNums.push_back((*it)->GetNum());
		it++;
	}
	return resNums;
}

template <typename T>
std::vector<int> QuadTreeNode<T>::GetObjectsContain(double x, double y) {
	std::vector<int> resObjects, temp;
	// 当前是叶节点,装入节点下所有对象编号
	if (upLeftNode == nullptr&&upRightNode == nullptr&&
		bottomLeftNode == nullptr&&bottomRightNode == nullptr)
	{
		temp = GetNums(x, y, objects);
		resObjects.insert(resObjects.end(), temp.begin(), temp.end());
		return resObjects;
	}
	// 否则先检查是否在当前节点下，再检查下层节点
	if (x >= (minX + maxX) / 2) {
		// 当前节点下
		if (x == (minX + maxX) / 2 && y >= (minY + maxY) / 2) {
			temp = GetNums(x, y, objects);
			resObjects.insert(resObjects.end(), temp.begin(), temp.end());
			return resObjects;
		}
		if (x == (minX + maxX) / 2 && y < (minY + maxY) / 2) {
			temp = GetNums(x, y, downObjects);
			resObjects.insert(resObjects.end(), temp.begin(), temp.end());
			return resObjects;
		}
		if (y == (minY + maxY) / 2 && x > (minX + maxX) / 2) {
			temp = GetNums(x, y, rightObjects);
			resObjects.insert(resObjects.end(), temp.begin(), temp.end());
			return resObjects;
		}
		// 点在下层节点中，需同时搜索本节点临近两轴上对象，若无节点返回空
		if (y > (minY + maxY) / 2) {
			if (upRightNode != nullptr) {
				temp = upRightNode->GetObjectsContain(x, y);
				resObjects.insert(resObjects.end(), temp.begin(), temp.end());
				temp = GetNums(x, y, objects);
				resObjects.insert(resObjects.end(), temp.begin(), temp.end());
				temp = GetNums(x, y, rightObjects);
				resObjects.insert(resObjects.end(), temp.begin(), temp.end());
			}
			return resObjects;
		}
		if (bottomRightNode != nullptr) {
			temp = bottomRightNode->GetObjectsContain(x, y);
			resObjects.insert(resObjects.end(), temp.begin(), temp.end());
			temp = GetNums(x, y, objects);
			resObjects.insert(resObjects.end(), temp.begin(), temp.end());
			temp = GetNums(x, y, rightObjects);
			resObjects.insert(resObjects.end(), temp.begin(), temp.end());
			temp = GetNums(x, y, downObjects);
			resObjects.insert(resObjects.end(), temp.begin(), temp.end());
		}
		return resObjects;
	}
	if (y == (minY + maxY) / 2) {
		temp = GetNums(x, y, leftObjects);
		resObjects.insert(resObjects.end(), temp.begin(), temp.end());
		return resObjects;
	}
	if (y > (minY + maxY) / 2) {
		if (upLeftNode != nullptr) {
			temp = upLeftNode->GetObjectsContain(x, y);
			resObjects.insert(resObjects.end(), temp.begin(), temp.end());
			temp = GetNums(x, y, objects);
			resObjects.insert(resObjects.end(), temp.begin(), temp.end());
			temp = GetNums(x, y, leftObjects);
			resObjects.insert(resObjects.end(), temp.begin(), temp.end());
		}
		return resObjects;
	}
	if (bottomLeftNode != nullptr) {
		temp = bottomLeftNode->GetObjectsContain(x, y);
		resObjects.insert(resObjects.end(), temp.begin(), temp.end());
		temp = GetNums(x, y, objects);
		resObjects.insert(resObjects.end(), temp.begin(), temp.end());
		temp = GetNums(x, y, leftObjects);
		resObjects.insert(resObjects.end(), temp.begin(), temp.end());
		temp = GetNums(x, y, downObjects);
		resObjects.insert(resObjects.end(), temp.begin(), temp.end());
	}
	return resObjects;
}