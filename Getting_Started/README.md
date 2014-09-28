#Getting Started

###0.) Prerequisites

So, you're now part of this repository and (presumably) now part of the AggreGator team. Congratulations. There are a few things you need to do before getting hands-on with the project (outlined below).

First, here are some *prequisites*:
  * Ask a team lead to add you to the Google Drive 
  * Add your contact information to the Contact Info document on the Google Drive
  * Get added to the organization on Trello (optional | software only)
    * for task management purposes
    * the Trello is the same as the whiteboard in the lab

----------------------------------------------------------
###1.) Install Ubuntu
*Recommended v13.04*

There are two ways to go about this, choose one or both
####Using a Virtual Machine (VM) (not recommended)

This is the easier way to get Ubuntu up and running on your computer. However, it is not recommended for several reasons, this includes slower performance, miscellaneous hardware issues, and system dependencies. For coding and testing purposes, this option is fine.

1.) **Downloading Virtualization Software**
Once again, two options. Download and install either.

* [VirtualBox](https://www.virtualbox.org/wiki/Downloads) (Oracle's freeware)

* [VMWare](http://e5.onthehub.com/WebStore/ProductsByMajorVersionList.aspx?ws=c52673b4-58fe-e011-8e6c-f04da23e67f6&vsro=8) (premium, free at onthehub)

2.) **Setting Up the Environment**
TBA - elaborate on creating an Ubuntu VM with appropriate settings for ROS.

3.) **Using the Environment**
Start your machine and you're ready to go.
To get acquainted with Linux terminal commands, please see [this](http://cli.learncodethehardway.org/bash_cheat_sheet.pdf)

####Dual-Booting A Partition (recommended)
This way is more low-level and can include unforseen complications that cannot be addressed generally. However, it is faster and is more stable than a VM.

1.) **Follow These Instructions**
Instructions [here](https://help.ubuntu.com/community/Installation). Ask a team member if you're having trouble. Or Google.

2.) **Liberate**
Now that you have Ubuntu, enjoy the superior OS.

----------------------------------------------------------
###2.) Cloning the Repository

All distros of Ubuntu have git pre-installed. If yours is bizarre or bootleg and doesn't have git, then run the following command

	sudo apt-get install git

1.) **Create a directory to put the repo folder in.**

	mkdir <path where you want directory to be>\<new directory>

mkdir will make this directory if it doesn't already exist there.

2.) **Change Into That Directory.**

	cd <relative path to your directory>

3.) **Clone the Repo with Git**

	git clone https://github.com/uf-aggregator/AggreGatorRMC.git

You have to be a member of a team on GitHub that's added to this repo. Ask [Andrew](https://github.com/FaytxZen) or [Joey](https://github.com/JoeyS7) to be added.

----------------------------------------------------------
###3.) Install ROS
*Current-distro: Hydro*

1.) **Follow These Instructions**

[Installing ROS](http://wiki.ros.org/hydro/Installation/Ubuntu). 

2.) **Install ROS Packages**

ROS has a bunch of handy packages for simplifying robotics. The currently used libraries are in the repo and will be compiled automatically, but if you want to test a new package,

**Individually install these packages**

Using
	
	sudo apt-get install ros-<distro>-<package name>


The following are the package names currently used by us, so just insert them at <package name>

* urg-node 
* usb-cam
* joy
* image-view
* image-transport
* gazebo-ros-pkgs

_NOTE_ There is a bash script in `AggreGatorWS/src/third-party-pkgs` ([link](https://github.com/uf-aggregator/AggreGatorRMC/tree/master/AggreGator_ws/src/third_party_pkgs)). Just run that and it should install all the packages we're using.

----------------------------------------------------------
###4.) Running the System

#####With ROSlaunch

TBD will be using roslaunch

#####Manually

Assuming you've `catkin_make`d the workspace, just do the command

	roscore

And then start up the following packages in listed order:

* [insert here]

----------------------------------------------------------
###5.) Troubleshooting

Some common issues we've encountered and our workarounds.

Click [here](https://docs.google.com/document/d/1OsnD7rFXJDcYGi1VJESj43cr08V7GOLNI-skH8Boo-o/edit?usp=sharing) to see troubleshooting document.

----------------------------------------------------------
###6.) New? Learn More About AggreGator
	
Visit our [site](http://www.ufaggregator.com)

