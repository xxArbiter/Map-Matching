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
	//�������Ҷ�ӽڵ㣬�������ӽڵ�  
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
	//�����Ҷ�ӽڵ㣬������Ҷ�ӽڵ�  
	if (level == maxLevel)
	{
		objects.push_back(object);
		return;
	}

	//��Ҷ�ӽڵ㣬����²�ڵ���԰����ö�����ݹ鹹���ӽڵ㲢�������,�߹����߲���
	if (IsContain((minX + maxX) / 2, (minY + maxY) / 2, maxX, maxY, object))
	{
		if (!upRightNode) //�����ظ��������ǵ�ԭ���Ľڵ�  
			upRightNode = new QuadTreeNode((minX + maxX) / 2, (minY + maxY) / 2, maxX, maxY, level + 1, maxLevel, UP_RIGHT, this);//���û���ӽڵ�ʹ����ӽڵ㣬parent�ڵ��ǵ�ǰ�ڵ�  
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
	//�²�ڵ㲻����ȫ�����Ķ�������뵽��ǰ��Ҷ�ӽڵ㣬�������ĸ������洢
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
	//�����ǰ�ڵ���ȫ���������ѵ�ǰ�ڵ��Ķ���ŵ��б�ĩβ,������Ҳ��  
	if (IsContain(_minX, _minY, _maxX, _maxY, this))
	{
		resObjects.insert(resObjects.end(), objects.begin(), objects.end());
		//���һ��  
		if (level == maxLevel)
			return resObjects;
	}

	//������²�ڵ�Ͱ��²�ڵ�����Ķ���ӽ���  
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
	//�������ڵ㱻������ɾ������ڵ�Ķ���  
	//����ж���Ҫ�ǶԸ��ڵ������ã������ӽڵ�ʵ�����ϲ㶼�����ж�  
	if (IsContain(_minX, _minY, _maxX, _maxY, this))
	{
		//������ڵ��Ķ���  
		objects.clear();
		//���һ��  
		if (level == maxLevel)
			return;

	}
	//������ӽڵ��ұ������������ӽڵ㣬ע������Ұָ��  
	//��ʵֻҪ�ϲ㱻�����ˣ��²�϶������������뻹��Ľ�  
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
		// ֻ�����������MBR���
		if ((*it)->IsInMBR(x, y))
			resNums.push_back((*it)->GetNum());
		it++;
	}
	return resNums;
}

template <typename T>
std::vector<int> QuadTreeNode<T>::GetObjectsContain(double x, double y) {
	std::vector<int> resObjects, temp;
	// ��ǰ��Ҷ�ڵ�,װ��ڵ������ж�����
	if (upLeftNode == nullptr&&upRightNode == nullptr&&
		bottomLeftNode == nullptr&&bottomRightNode == nullptr)
	{
		temp = GetNums(x, y, objects);
		resObjects.insert(resObjects.end(), temp.begin(), temp.end());
		return resObjects;
	}
	// �����ȼ���Ƿ��ڵ�ǰ�ڵ��£��ټ���²�ڵ�
	if (x >= (minX + maxX) / 2) {
		// ��ǰ�ڵ���
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
		// �����²�ڵ��У���ͬʱ�������ڵ��ٽ������϶������޽ڵ㷵�ؿ�
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