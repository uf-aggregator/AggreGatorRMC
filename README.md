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

###Recovering from Crashes


First, ssh into the Odroid:

```
ssh odroid@odroid
```

Once logged in, run the kill all processes command:
```
killall5
```

Your ssh session will end after this command; log into the ODroid again, and run the ODroidStart script as before. 
roscore on the Odroid will now be running, and you can once again connect to it with Command Nodes.  

----------------------------------------------------------

See our [style guide](https://github.com/uf-aggregator/AggreGatorRMC/tree/master/Getting_Started/Style_Guide "UF Aggregator Style Guide") for our code and naming practices

----------------------------------------------------------

