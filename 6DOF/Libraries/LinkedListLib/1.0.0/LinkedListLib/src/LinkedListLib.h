/***************************************************
Copyright (c) 2017 Luis Llamas
(www.luisllamas.es)

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License
 ****************************************************/

#ifndef _LinkedListLib_h
#define _LinkedListLib_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

template<class T>
struct Node
{
	T value;
	Node<T> *next;
};

template <typename T>
class LinkedList {

protected:
	int _size;

	Node<T>* _head;
	Node<T>* _tail;

	Node<T>* _iterator;
	int _iteratorIndex;

public:
	LinkedList();
	~LinkedList();

	int GetSize() const;

	T GetHead();
	T GetTail();
	T GetAt(int index);

	void InsertHead(T);
	void InsertTail(T);
	void InsertAt(int index, T);

	void ReplaceHead(T value);
	void ReplaceTail(T value);
	void ReplaceAt(int index, T value);

	T RemoveHead();
	T RemoveTail();
	T RemoveAt(int index);
	
	void Clear();
	void FromArray(T* fromArray, int n);
	T* ToArray();

private:
	void insertEmpty(T);
	void reset_iterator();
	void move_iterator(int index);
	Node<T>* getNode(int index);
};



template<typename T>
LinkedList<T>::LinkedList()
{
	_size = 0;
	_head = nullptr;
	_tail = nullptr;
}

template<typename T>
LinkedList<T>::~LinkedList()
{
	Node<T>* node;
	while (_head != nullptr)
	{
		node = _head;
		_head = _head->next;
		delete node;
	}
	_tail = nullptr;
	_size = 0;
}

template<typename T>
int LinkedList<T>::GetSize() const
{
	return _size;
}

template<typename T>
void LinkedList<T>::InsertHead(T value)
{
	if (_size == 0)
		return insertEmpty(value);

	Node<T>* node = new Node<T>();
	node->next = _head;
	node->value = value;
	_head = node;

	reset_iterator();
	_size++;
}

template<typename T>
void LinkedList<T>::InsertTail(T value)
{
	if (_size == 0)
		return insertEmpty(value);

	Node<T>* node = new Node<T>();
	node->value = value;
	node->next = nullptr;

	_tail->next = node;
	_tail = node;

	reset_iterator();
	_size++;
}

template<typename T>
void LinkedList<T>::InsertAt(int index, T value)
{
	if (_size == 0)
		return insertEmpty(value);

	Node<T>* node = new Node<T>();
	Node<T>* prev = getNode(index);
	node->next = prev->next;
	prev->next = node;
	node->value = value;
	
	if (_iteratorIndex >= index)
		reset_iterator();
	_size++;
}



template<typename T>
T LinkedList<T>::GetHead()
{
	return _head->value;
}

template<typename T>
T LinkedList<T>::GetTail()
{
	return _tail->value;
}

template<typename T>
T LinkedList<T>::GetAt(int index)
{
	Node<T>* node = getNode(index);
	return node->value;
}


template<typename T>
void LinkedList<T>::ReplaceHead(T value)
{
	_head->value = value;
}

template<typename T>
void LinkedList<T>::ReplaceTail(T value)
{
	_tail->value = value;
}

template<typename T>
void LinkedList<T>::ReplaceAt(int index, T value)
{
	getNode(index)->value = value;
}

template<typename T>
T LinkedList<T>::RemoveHead()
{
	if (_size <= 0)
		return T();

	if (_size > 1) {
		Node<T>* next = _head->next;
		T rst = _head->value;
		delete(_head);
		_head = next;
		_size--;
		return rst;
	}
	else {
		return T();
	}
}

template<typename T>
T LinkedList<T>::RemoveTail()
{
	if (_size <= 0)
		return T();

	if (_size > 1) {
		Node<T>* prev = getNode(_size - 2);
		T rst = prev->next->value;
		delete(prev->next);
		prev->next = nullptr;
		_tail = prev;
		_size--;
		return rst;
	}
	else {
		T rst = _head->value;
		delete(_head);
		_head = nullptr;
		_tail = nullptr;
		_size = 0;
		return rst;
	}
}

template<typename T>
T LinkedList<T>::RemoveAt(int index)
{
	if (_size <= 0)
		return T();

	Node<T>* prev = getNode(index-1);
	Node<T>* node = prev->next;
	prev->next = node->next;
	delete node;
	_size--;
}

template<typename T>
void LinkedList<T>::Clear() 
{
	while (_size > 0)
		RemoveTail(); 
}

template<typename T>
T* LinkedList<T>::ToArray() 
{
	T* rst = new T[_size];

	Node<T>* node = _head;
	for (int iCount = 0; iCount < _size; iCount++)
	{
		rst[iCount] = node->value;
		node = node->next;
	}
	return rst;
}

template<typename T>
void LinkedList<T>::FromArray(T* fromArray, int n)
{
	if (_size > 0)
		Clear();

	for (int iCount = 0; iCount < n; iCount++)
	{
		InsertTail(fromArray[iCount]);
	}
}

template<typename T>
void LinkedList<T>::move_iterator(int index)
{
	if (_iteratorIndex > index)
		reset_iterator();

	while (_iteratorIndex < index && _iteratorIndex < _size)
	{
		_iterator = _iterator->next;
		_iteratorIndex++;
	}
}

template<typename T>
inline Node<T>* LinkedList<T>::getNode(int index)
{
	move_iterator(index);
	return _iterator;
}

template<typename T>
inline void LinkedList<T>::reset_iterator()
{
	_iteratorIndex = 0;
	_iterator = _head;
}

template<typename T>
void LinkedList<T>::insertEmpty(T value) 
{
	Node<T>* node = new Node<T>();
	node->value = value;
	node->next = nullptr;
	_head = node;
	_tail = node;
	_size++;
}

#endif
