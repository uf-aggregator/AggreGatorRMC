#Aggregator Best Practices && Style Guide

This is the best practices and style guide for the UF Aggregator codebase.

----------------------------------------------------------
###Naming
In order to keep code uniform and easily readable by members, use the following naming conventions,
	* ROS Packages - lowercase_with_underscores
	* ROS Nodes - lowercase_with_underscores_plus_node_at_the_end_node
	* ROS Message Files - [UpperCamelCase](http://c2.com/cgi/wiki?CamelCase)
	* ROS Service Files - same as message.
	* Files
		* C++	- same as ROS Nodes
		* Scripts - [camelCase](http://c2.com/cgi/wiki?CamelCase)
	* Classes - same as ROS Message Files
	* Variables
		* Persistant Variables - same as ROS Packages
			* Like i2c_current_value.
		* Other variables - same as script files.

----------------------------------------------------------
###ROS
In order to lower redundancy and help organize everything, follow these practices when working with ROS,
	* One interface, one package
		* ex.) Camera system is one package. This package does the feed, the transforms, and any computations related to the camera system.
	* Don't make a new message file, if it exists elsewhere
		* You can import message files from other packages in your packages src/CMakeList.txt
	* The same goes for service, action, and C++ header files.

----------------------------------------------------------
###Classes
AggreGator relies on a mixture of functional and object oriented programming. In order to utilize the best of both, follow these practices.
	**_Reminder_** OOP is object-oriented programming and focuses on extending classes and creating classes for every function. Functional programming focuses on writing generally applicable functions and calling them directly where needed.

	* **Functional Programming**
		* This should be kept in its own header and header definition files. These are functions that any node of a particular type will need to use.
		* These should be purely .h files.
		* ex.) Hardware interfacing like I2C functions. Though I2C functions can be wrapped into a class, you may only need exactly one function from the suite, so it's better for memory and processing to just call that directly.
	* **OOP**
		* This should be any system in the robot, i.e. handler for the entire camera system should be OOP. Autonomy should be OOP
		* Classes should not be made for just a few functions. Use OOP when you have a whole suite of functions you need to add or a slew of publishing and
		* ex.) Hardware interfacing like I2C functions. Though I2C functions can be wrapped into a class, you may only need exactly one function from the suite, so it's better for memory and processing to just call that directly. subscribing you need to carry out.

----------------------------------------------------------
###Variables
These are best practices for keeping code manageable and uncluttered through good variable use
	* **No Global Variables**
		* If you can help it. In functional programming, the functions just handle input of a certain kind and output of a certain kind.
		* e.g. Assume whoever is using the function knows it needs, say, a 5 char array.
	* **Reduce, Reuse, Recycle**
		* If you have a series of variables ( > 3) that you need, just make an array.
		* If you need global variables, just encapsulate the method in a class that holds those data members. 

----------------------------------------------------------
###Paths
Just use relative paths whenever possible.
ex.) Pretend you're using /home/catkin_workspace/src/FileINeed.txt for some IO op. The file you're writing the IO code in is at /home/catkin_workspace/src/SomePackage/src/MyCode.cpp.
Rather than specifying the absolute path, use
`../../FileINeed.txt`
In case /home/catkin_workspace is not the path to the workspace.