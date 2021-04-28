# Java
This code is for an Arduino to read temperature data from a TC4 thermocouple board and display the temperature on a 2 x 16 LCD.

It compiles on the arduino compiler

The main directory should contain aJava.pde, button.pde user.h

needs libs 

Standard libs

#include <Wire.h>

#include <cLCD.h>

Custom libs, copy from this git

#include <TypeK.h>

#include <cADC.h>

#include "user.h"

add subdirectories TypeK and cADC to your Arduino/Libraries directory.  Add user.h to the main folder with the pde file
