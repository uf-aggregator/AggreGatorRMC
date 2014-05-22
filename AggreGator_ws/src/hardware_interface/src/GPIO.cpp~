/* Daniel Kelly/Abrar Polani
  O-Droid GPIO_Read/Write source code
   This file contains a functions that interface to the O-Droid's GPIO headers*/
#include <sstream>
#include <iostream>
#include <fstream>
#include "stdlib.h"
#include "GPIO.h"
using namespace std;

//Function that writes values to the valid GPIO pins on the 
//O-Droid header, accepts user input for pin number and value
void setGPIOWrite(int pinNumber, int val)
{
	while(true){
	//Check if pin input is valid
	if(pinNumber > 45 || pinNumber < 17 || pinNumber == 32)
	{
		cout << "Invalid pin. Please specify another pin: ";
		cin >> pinNumber;
	}
//Creates an array which maps the inputted pin number to the //gpiochip value to write in the export file
//Valid pins are from 45 to 17, so array goes from 28 to 0.
string gpioLookUp[29] =  {"112","115","93","100","108","91","90","99","111","103","88","98","89","114","87","GND","94",
"105","97","102","107","110","101","117","92","96","116","106","109"};
	
	int pinNumberIndex = pinNumber - 17; //Offsets input number, to be an index in the gpioLookUp array
	
	string gpioExport = gpioLookUp[pinNumberIndex]; //Gets //gpio value to write to export file
	
	int value = val; //sets inputted value
	ofstream gpioFile; //file object to manipulate
	
	cout << "Exporting GPIO" << gpioExport << " which is physical pin " << pinNumber << " on the Odroid X2 header"<< endl;
	
	//Open export file to set which gpio to modify
	gpioFile.open("/sys/class/gpio/export", ios::out);
	if(gpioFile.is_open())
	{
		gpioFile << gpioExport;
		gpioFile.close();
	}
	else	{
		cout  << "Unable to open /sys/class/gpio/export" << endl << "Try another pin: ";
		cin >> pinNumber;
		continue;

	}
	
	//opens relevant direction file to set pin direction
	gpioFile.open(("/sys/class/gpio/gpio" + gpioExport + "/direction").c_str(), ios::out);
	
	if(gpioFile.is_open())
	{
		gpioFile << "out";
		gpioFile.close();
	}
	else
	{
		cout  << "Unable to open /sys/class/gpio/gpio" << gpioExport << "/direction" << endl
		<< "Try another pin: ";
		cin >> pinNumber;
		continue;
	}
	
	//Opens relevant value file to set pin value
	gpioFile.open(("/sys/class/gpio/gpio" + gpioExport + "/value").c_str(), ios::out);
	if(gpioFile.is_open())
	{
		stringstream ss;
		ss << value;
		string val = ss.str();

		gpioFile << val;
		gpioFile.close();
	}
	else
	{
		cout  << "Unable to open /sys/class/gpio" << gpioExport << "/value"<< endl
		<< "try another pin: ";
		cin >> pinNumber;
		continue;
	}
	
	//Unexports pin file, puts it away for clean up purposes
	gpioFile.open("/sys/class/gpio/unexport", ios::out);
	if(gpioFile.is_open())
	{
		
		gpioFile << gpioExport;
		gpioFile.close();
	}
	else
	{
		cout  << "Unable to open /sys/class/gpio/unexport"<< endl
		<< "Try another pin: ";
		cin >> pinNumber;
		continue;
	}
	break;
}
	
}
	

//Function that sets a given GPIO pin to be an input. Used for reading in values.
void setGPIORead(int pinNumber)
{
	while(true){
	//Check if pin input is valid
	if(pinNumber > 45 || pinNumber < 17 || pinNumber == 32)
	{
		cout << "Invalid pin. Please specify another pin: ";
		cin >> pinNumber;
	}
	//Creates an array which maps the inputted pin number to the //gpiochip value to write in the export file
	//Valid pins are from 45 to 17, so array goes from 28 to 0.
	string gpioLookUp[29] =  {"112","115","93","100","108","91","90","99","111","103","88","98","89","114","87","GND","94",
	"105","97","102","107","110","101","117","92","96","116","106","109"};
	
	int pinNumberIndex = pinNumber - 17; //Offsets input number, to be an index in the gpioLookUp array
	
	string gpioExport = gpioLookUp[pinNumberIndex]; //Gets //gpio value to write to export file
	ofstream gpioFile; //file object to manipulate
	
	cout << "Exporting GPIO" << gpioExport << " which is physical pin " << pinNumber << " on the Odroid X2 header"<< endl;
	
	//Open export file to set which gpio to modify
	gpioFile.open("/sys/class/gpio/export", ios::out);
	if(gpioFile.is_open())
	{
		
		gpioFile << gpioExport;
		gpioFile.close();
	}
	else	{
		cout  << "Unable to open /sys/class/gpio/export" << endl << "Try another pin: ";
		continue;

	}
	
	//opens relevant direction file to set pin direction
	gpioFile.open(("/sys/class/gpio/gpio" + gpioExport + "/direction").c_str(), ios::out);
	
	if(gpioFile.is_open())
	{
		
		gpioFile << "in";
		gpioFile.close();
	}
	else
	{
		cout  << "Unable to open /sys/class/gpio/gpio" << gpioExport << "/direction" << endl
		<< "Try another pin: ";
		cin >> pinNumber;
		continue;
	}
	
	
	//Unexports pin file, puts in away for clean up purposes
	gpioFile.open("/sys/class/gpio/unexport", ios::out);
	if(gpioFile.is_open())
	{
	
		gpioFile << gpioExport;
		gpioFile.close();
	}
	else
	{
		cout  << "Unable to open /sys/class/gpio/unexport"<< endl
		<< "Try another pin: ";
		cin >> pinNumber;
		continue;
	}
	break;
}
}
//Function that reads a value from the O-Droid's GPIO pin, pinNumber
 int readGPIO(int pinNumber)
{
	while(true){
	string line;
	string direction;
	int pinValue;
	//Check if pin input is valid
	if(pinNumber > 45 || pinNumber < 17 || pinNumber == 32)
	{
		cout << "Invalid pin. Please specify another pin: ";
		cin >> pinNumber;
	}
	
	//Creates an array which maps the inputted pin number to the //gpiochip value to write in the export file
	//Valid pins are from 45 to 17, so array goes from 28 to 0.
	string gpioLookUp[29] =  {"112","115","93","100","108","91","90","99","111","103","88","98","89","114","87","GND","94",
	"105","97","102","107","110","101","117","92","96","116","106","109"};
	
	int pinNumberIndex = pinNumber - 17; //Offsets input //number, to be an index in the gpioLookUp array
	
	string gpioExport = gpioLookUp[pinNumberIndex]; //Gets //gpio value to write to export file
	fstream gpioFile; //file object to manipulate
	
	cout << "Exporting GPIO" << gpioExport << " which is physical pin " << pinNumber << " on the Odroid X2 header"<< endl;
	
	//Open export file to set which gpio to modify
	gpioFile.open("/sys/class/gpio/export", ios::out);
	if(gpioFile.is_open())
	{
		
		gpioFile << gpioExport;
		gpioFile.close();
	}
	else	
	{
		cout  << "Unable to open /sys/class/gpio/export" << endl << "Try another pin: "
		<< "Try another pin: ";
		cin >> pinNumber;
		continue;

	}
	
	//opens relevant direction file to read pin direction
	gpioFile.open(("/sys/class/gpio/gpio" + gpioExport + "/direction").c_str(), ios::in);
	//Reads what direction the pin is to make sure it is set to "in"
	if(gpioFile.is_open())
	{
		while(getline(gpioFile,line))
		{
			istringstream ss(line);
			
			ss >> direction;
			
		}
		gpioFile.close();
	}
	else
	{
		cout  << "Unable to open /sys/class/gpio/gpio" << gpioExport << "/direction" << endl
		<< "Try another pin: ";
		cin >> pinNumber;
		continue;
	}
	//Don't read from a pin that is set to "out"
	if(direction != "in")
	{
		cout << "Cannot read from a pin set to output! Please set pin direction to IN before reading" << endl
		<< "Try another pin: ";
		cin >> pinNumber;
		continue;
	}
	
	//Opens relevant value file to read pin value
	gpioFile.open(("/sys/class/gpio/gpio" + gpioExport + "/value").c_str(), ios::in);
	if(gpioFile.is_open())
	{
		while(getline(gpioFile,line))
		{
			istringstream ss(line);
			
			ss >> pinValue;
			
		}
		gpioFile.close();
	}
	else
	{
		cout  << "Unable to open /sys/class/gpio" << gpioExport << "/value"<< endl
		<< "Try another pin: ";
		cin >> pinNumber;
		continue;
	}
	
	//Unexports pin file, puts in away for clean up purposes
	gpioFile.open("/sys/class/gpio/unexport", ios::out);
	if(gpioFile.is_open())
	{
		
		gpioFile << gpioExport;
		gpioFile.close();
	}
	else
	{
		cout  << "Unable to open /sys/class/gpio/unexport"<< endl
		<< "Try another pin: ";
		cin >> pinNumber;
		continue;
	}
	
        return pinValue;
	break;
	}
}

//Upon shutdown, reset all GPIO pins to 0
void resetGPIO()
{
	 
	for(int i = 17; i <= 31; i++)
	{
		setGPIOWrite(i,0);
	}
	for(int k = 33; k <= 45; k++)
	{
		setGPIOWrite(k,0);
	}
}	


