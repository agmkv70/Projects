/*
 *  QueueListFIXEDARRAY.h
 *
 *  Library implementing a generic, dynamic queue (linked list version).
 *
 *  ---
 *
 *  Copyright (C) 2010  Efstathios Chatzikyriakidis (contact@efxa.org)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  Version 1.0
 *
 *    2010-09-28  Efstathios Chatzikyriakidis  <contact@efxa.org>
 *
 *      - added exit(), blink(): error reporting and handling methods.
 *
 *    2010-09-25  Alexander Brevig  <alexanderbrevig@gmail.com>
 *
 *      - added setPrinter(): indirectly reference a Serial object.
 *
 *    2010-09-20  Efstathios Chatzikyriakidis  <contact@efxa.org>
 *
 *      - initial release of the library.
 *
 *  ---
 *
 *  For the latest version see: http://www.arduino.cc/
 */

// header defining the interface of the source.
#ifndef _QUEUELISTFIXEDARRAY_H
#define _QUEUELISTFIXEDARRAY_H

#ifndef _MAX_FIXEDARRAY_DEFINED
#define _MAX_FIXEDARRAY_DEFINED 15
#endif

// include Arduino basic header.
#include <Arduino.h>

// the definition of the queue class.
template<typename T>
class QueueListFA {
  public:
    const static int MAX_FIXEDARRAY = _MAX_FIXEDARRAY_DEFINED;
    // init the queue (constructor).
    QueueListFA ();

    // clear the queue (destructor).
    ~QueueListFA ();

    // push an item to the queue.
    void push (T* i);

    // pop an item from the queue.
    T* pop ();

    void drop(); //drop current item

    // get an item from the queue.
    T* peek ();

    // check if the queue is empty.
    bool isEmpty () const;

    // get the number of items in the queue.
    int count () const;

    // set the printer of the queue.
    void setPrinter (Print & p);

  private:
    // exit report method in case of error.
    void exit (const char * m) const;

    // led blinking method in case of error.
    void blink () const;

    // the pin number of the on-board led.
    static const int ledPin = 13;

    T fixedArray[MAX_FIXEDARRAY];

    Print * printer; // the printer of the queue.
    int size;        // the size of the queue.
};

// init the queue (constructor).
template<typename T>
QueueListFA<T>::QueueListFA () {
  size = 0;       // set the size of queue to zero.
  printer = NULL; // set the printer of queue to point nowhere.
}

// clear the queue (destructor).
template<typename T>
QueueListFA<T>::~QueueListFA () {
  
  size = 0;       // set the size of queue to zero.
  printer = NULL; // set the printer of queue to point nowhere.
}

// push an item to the queue.
template<typename T>
void QueueListFA<T>::push (T* i) {
  
  // if there is a memory allocation error.
  if (size == MAX_FIXEDARRAY)
    exit ("QUEUE: insufficient fixed array size to push more.");

  memcpy(&(fixedArray[size]),i,sizeof(T));
  // increase the items.
  size++;
}

// pop an item from the queue.
template<typename T>
T* QueueListFA<T>::pop () {
  // check if the queue is empty.
  if (size==0)
    exit ("QUEUE: can't pop item from fixed array: queue is empty.");

  // decrease the items.
  size--;

  // return the item.
  return &(fixedArray[size]);
}

// drop an item from the queue.
template<typename T>
void QueueListFA<T>::drop () {
  // check if the queue is empty.
  if (size==0)
    exit ("QUEUE: can't pop item from fixed array: queue is empty.");

  // decrease the items.
  size--;

  return;
}

// get an item from the queue.
template<typename T>
T* QueueListFA<T>::peek () {
  // check if the queue is empty.
  if (size==0)
    exit ("QUEUE: can't peek item from fixed array: queue is empty.");

  // return the item of the head node.
  return &(fixedArray[size-1]);
}

// check if the queue is empty.
template<typename T>
bool QueueListFA<T>::isEmpty () const {
  return size==0;
}

// get the number of items in the queue.
template<typename T>
int QueueListFA<T>::count () const {
  return size;
}

// set the printer of the queue.
template<typename T>
void QueueListFA<T>::setPrinter (Print & p) {
  printer = &p;
}

// exit report method in case of error.
template<typename T>
void QueueListFA<T>::exit (const char * m) const {
  // print the message if there is a printer.
  if (printer)
    printer->println (m);

  // loop blinking until hardware reset.
  blink ();
}

// led blinking method in case of error.
template<typename T>
void QueueListFA<T>::blink () const {
  // set led pin as output.
  pinMode (ledPin, OUTPUT);

  // continue looping until hardware reset.
  while (true) {
    digitalWrite (ledPin, HIGH); // sets the LED on.
    delay (250);                 // pauses 1/4 of second.
    digitalWrite (ledPin, LOW);  // sets the LED off.
    delay (250);                 // pauses 1/4 of second.
  }

  // solution selected due to lack of exit() and assert().
}

#endif // _QUEUELIST_H
