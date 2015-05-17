#AggreGatorRMC

Welcome to the UF AggreGator GitHub repository!

Keep up to date with non-code shenanigans at [our website](http://www.ufaggregator.com "UF Aggregator Official Homepage")

----------------------------------------------------------

###Setting Up Your Workspace

In order to setup the workspace, first run the usual commands from the directory you want it in,
	
```shell

	# init the workspace
	source /opt/ros/hydro/setup.bash
	mkdir src
	cd src
	catkin_init_workspace

	# build it for the first time
	cd ..
	catkin_make

	# clone the repo in src/
	cd src/
	git clone https://github.com/uf-aggregator/AggreGatorRMC.git

	#initialize the repo with a special build script
	cd ..
	catkin_make --pkg common_files
	catkin_make --pkg common_files

	#repeat the following until it generates without error, normally ~2-3 times
	catkin_make
```

---------------------------------------------------------

###Connecting to the Odroid (AggreGator)

First, make sure the router is setup and that the Odroid is set to connect.
Alternatively, you can hook up an ethernet cable between your machine and the Odroid.

Connect to that router's network, then ssh into the Odroid:

```bash

	ssh odroid@odroid
```

If you can't find the hostname IP for the Odroid, run the following while connected to the Team# WiFi

```bash

	sudo apt-get install arp-scan

```

Then, depending on your preference, run one of the below commands

```bash
	
	sudo arp-scan --interface=wlan0 --localnet 		#meant for wifi
	sudo arp-scan --interface=eth0 --localnet		#meant for LAN

```

---------------------------------------------------------

###Starting the AggreGator

Navigate into the folder with the `.launch` files and start one of them with

```bash

	roslaunch <file>.launch

```

If it says `roslaunch` isn't a command, check that you `source`'d `devel/setup.bash`.


---------------------------------------------------------

###Recovering from Crashes

First, connect to the Odroid via ssh.

Once logged in, run the kill all processes command:

```bash

	killall5
```

Your ssh session will end after this command; log into the ODroid again, and run the ODroidStart script as before. 
roscore on the Odroid will now be running, and you can once again connect to it with Command Nodes.  

---------------------------------------------------------

###System Structure

Granted you have the appropriate permissions, you can view our design doc [here](https://docs.google.com/presentation/d/1R9dvbtlgpm_iyYBrbS1A0h2gCdaTvyt2T7zIN8LaEMw/edit?usp=sharing).