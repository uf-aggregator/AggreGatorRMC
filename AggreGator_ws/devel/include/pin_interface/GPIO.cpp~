/* Daniel Kelly
   GPIO_Read/Write source code
   This file takes physical pin number as input and changes a GPIO pin on the Odroid-X2 to be "in" or "out"  and then sets the values of the pin. (Either "1" or "0")*/

#include <iostream>
#include <fstream>
using namespace std;

void GPIOWrite(int pin_number, int value)
{
	string gpioLookUp[27] = 							   					  {"112","115","93","100","108","91","90","99","111","103","88","98","89","114","87","GND","94",
"105","97","102","107","110","101","117","116","106","109"};
	
	int pinNumber = pinNumber;
	int pinNumberIndex = pinNumber - 17;
	string gpioExport = gpioLookUp[pinNumberIndex]; 
	int value = value;
	ofstream gpioFile;
	
	cout << "Exporting GPIO" << gpioExport << "which is physical pin " << pinNumber << " on the Odroid X2 header";
	

	gpioFile.open("/sys/class/gpio/export", ios::out);
	if(gpioFile.is_open())
	{
		gpioFile << gpioExport
		gpioFile.close();
	}
	else
	{
		cout  << "Unable to open /sys/class/gpio/export";
	}

	gpioFile.open("/sys/class/gpio/gpio" + gpioExport + "/direction", ios::out);
	
	if(gpioFile.is_open())
	{
		gpioFile << "out";
		gpioFile.close();
	}
	else
	{
		cout  << "Unable to open /sys/class/gpio/gpio" << gpioExport << "/direction";
	}
	
	gpioFile.open("/sys/class/gpio/" + gpioExport + "/value", ios::out);
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
		cout  << "Unable to open /sys/class/gpio/" << gpioExport << "/value";
	}

	gpioFile.open("/sys/class/gpio/unexport", ios::out);
	if(gpioFile.is_open())
	{
		gpioFile << gpioExport;
		gpioFile.close();
	}
	else
	{
		cout  << "Unable to open /sys/class/gpio/unexport";
	}
	
	delete[] gpioLookUp;
}
	

void GPIOWrite(int pin_number, int value)
{
	string gpioLookUp[27] = 							   					  {"112","115","93","100","108","91","90","99","111","103","88","98","89","114","87","GND","94",
"105","97","102","107","110","101","117","116","106","109"};
	
	int pinNumber = pinNumber;
	int pinNumberIndex = pinNumber - 17;
	string gpioExport = gpioLookUp[pinNumberIndex]; 
	int value = value;
	fstream gpioFile;
	
	cout << "Exporting GPIO" << gpioExport << "which is physical pin " << pinNumber << " on the Odroid X2 header";
	

	gpioFile.open("/sys/class/gpio/export", ios::out);
	if(gpioFile.is_open())
	{
		gpioFile << gpioExport
		gpioFile.close();
	}
	else
	{
		cout  << "Unable to open /sys/class/gpio/export";
	}

	gpioFile.open("/sys/class/gpio/gpio" + gpioExport + "/direction", ios::out);
	
	if(gpioFile.is_open())
	{
		gpioFile << "in";
		gpioFile.close();
	}
	else
	{
		cout  << "Unable to open /sys/class/gpio/gpio" << gpioExport << "/direction";
	}
	
	gpioFile.open("/sys/class/gpio/" + gpioExport + "/value", ios::in);
	if(gpioFile.is_open())
	{
		while(getline (gpioFile,line))
		{
			istringstream ss(line);
			int val;
			ss >> val;
		}
		gpioFile.close();
	}
	else
	{
		cout  << "Unable to open /sys/class/gpio/" << gpioExport << "/value";
	}

	gpioFile.open("/sys/class/gpio/unexport", ios::out);
	if(gpioFile.is_open())
	{
		gpioFile << gpioExport;
		gpioFile.close();
	}
	else
	{
		cout  << "Unable to open /sys/class/gpio/unexport";
	}
	
	delete[] gpioLookUp;
}

