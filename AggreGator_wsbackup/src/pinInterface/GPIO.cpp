/* Daniel Kelly
   GPIO_Read/Write source code
   This file takes physical pin number as input and changes a GPIO pin on the Odroid-X2 to be "in" or "out"  and then sets the values of the pin. (Either "1" or "0")*/

#include <iostream>
#include <fstream>
using namespace std;

void GPIOWrite(int pin_number, int value)
{
	string gpioLookUp[27] = {"105.7","114.1","87.6","96.4","105.3","87.4","87.3","96.3","105.6","96.7","87.1","96.2","87.2","114.0","87.0","gnd","87.7","105.0","96.1","96.6","105.2","105.5","96.5","114.3","114.2","105.1","105.4"};
	
	int pinNumber = pinNumber;
	int pinNumberIndex = pinNumber - 17;
	int gpioChip = gpioLookUp[pinNumberIndex]; 
	int value = value;
	fstream gpioFile;
	
	gpioFile.open("/sys/class/gpio/gpio" + gpioChip + "/direction")  
	
	if(!gpioFile.is_open)
	{
		cout << "Exporting GPIO" << pin_number << 
