/***************************************************
Copyright (c) 2017 Luis Llamas
(www.luisllamas.es)

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License
 ****************************************************/
 
#include "LinkedListLib.h"

// Funcion auxiliar para debug
void printArray(int* x, int length)
{
	for (int iCount = 0; iCount < length; iCount++)
	{
		Serial.print(x[iCount]);
		Serial.print(',');
	}
}

void setup() {
	Serial.begin(9600);
	
	// Crear LinkedList
	LinkedList<int> linkedlist;
	
	// Ejemplos insert
	linkedlist.InsertHead(1);	//1
	linkedlist.InsertTail(5);	//1,5
	linkedlist.InsertAt(0, 2);	//1,2,5
	linkedlist.InsertAt(1, 4);	//1,2,4,5
	linkedlist.InsertAt(1, 3);	//1,2,3,4,5

	// Ejemplos get
	Serial.println(linkedlist.GetHead());
	Serial.println(linkedlist.GetAt(0));
	Serial.println(linkedlist.GetAt(1));
	Serial.println(linkedlist.GetAt(2));
	Serial.println(linkedlist.GetAt(3));
	Serial.println(linkedlist.GetAt(4));
	Serial.println(linkedlist.GetTail());

	// Ejemplos replace
	linkedlist.ReplaceHead(10);
	linkedlist.ReplaceAt(2, 30);
	linkedlist.ReplaceAt(3, 40);
	linkedlist.ReplaceAt(1, 20);
	linkedlist.ReplaceTail(50);

	Serial.println(linkedlist.GetHead());
	Serial.println(linkedlist.GetAt(0));
	Serial.println(linkedlist.GetAt(1));
	Serial.println(linkedlist.GetAt(2));
	Serial.println(linkedlist.GetAt(3));
	Serial.println(linkedlist.GetAt(4));
	Serial.println(linkedlist.GetTail());


	// Ejemplo conversion a array
	int* toArray = linkedlist.ToArray();
	printArray(toArray, linkedlist.GetSize());

	// Ejemplos conversion desde array
	int testArray[] = { 100,200,300,400,500 };
	linkedlist.FromArray(testArray, sizeof(testArray)/sizeof(int));
	int* toArray2 = linkedlist.ToArray();
	printArray(toArray2, linkedlist.GetSize());
}

void loop() {

}
