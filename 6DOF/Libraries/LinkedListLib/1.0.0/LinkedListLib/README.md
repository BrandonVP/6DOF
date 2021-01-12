# Librería Arduino Linkedlist
Librería para Arduino que implementa una versión sencilla de una Single LinkedList, una colección de elementos formada por una serie de nodos que permite añadir y eliminar elementos de forma muy eficiente.

Más información https://www.luisllamas.es/libreria-arduino-linkedlist/

## Instrucciones de uso
Cada nodo contiene un elemento y una referencia al siguiente nodo. De esta forma, es posible añadir o eliminar elementos creando o eliminando nodos, y gestionando las referencias entre ellos. 

La clase dispone de funciones para obtener, añadir, insertar, reemplazar y eliminar elementos de la LinkedList. Obtener un elemento es un proceso relativamente lento respecto a un array simple, porque es necesario recorrer todos los nodos hasta alcanzar el nodo deseado. 

Para mejorar la eficiencia la clase mantiene una referencia al último elemento accedido. Esto evita tener que recorrer la colección desde el principio, lo cual es especialmente útil al acceder repetidas veces al mismo índice. 

Adicionalmente, la clase mantiene una referencia al nodo cabeza (head) y cola (tail) lo que permite implementar de forma eficiente estructuras de tipo LIFO (Pila) y FIFO (Cola). 

También se disponen de métodos para buscar elementos dentro de la LinkedList, invertir los elementos, y para copiar los elementos desde y hacia un array externo.

### Constructor
La clase LinkedList se instancia a través de su constructor.
```c++
// Crea una nueva LinkedList
LinkedList();
```

### Métodos generales
```c++
// Obtiene el tamaño
int GetSize() const;
```

### Acceder elementos
```c++
// Devuelve el primer elemento
T GetHead();

// Devuelve el ultimo elemento
T GetTail();

// Devuelve el elemento en la posicion indicada
T GetAt(int index);
```

### Insertar elementos
```c++
// Inserta un elemento al inicio
void InsertHead(T);

// Insetar un elemento al final
void InsertTail(T);

// Inserta un elemento en la posicion indicada
void InsertAt(int index, T);
```

### Reemplazar elementos
```c++
// Reemplazar el primer elemento
void ReplaceHead(T value);

// Reemplazar el ultimo elemento
void ReplaceTail(T value);

// Reemplazar el elemento en la posicion indicada
void ReplaceAt(int index, T value);
```

### Eliminar elementos
```c++
// Elimina todos los elementos
void Clear();

// Elimina el primer elemento
T RemoveHead();

// Elimina el ultimo elemento
T RemoveTail();

// Elimina el elemento en la posicion indicada
T RemoveAt(int index);
```

### Conversión a arrays
```c++
// Devuelve todos los elementos en un nuevo array
T* ToArray();

// Inicia la LinkedList copiando los elementos desde un array
void FromArray(T* fromArray, int n);
```


## Ejemplos
La librería LinkedList incluye los siguientes ejemplos para ilustrar su uso.
* LinkedList: Ejemplo general de uso de la clase LinkedList.
```c++
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
```
