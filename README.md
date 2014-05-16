NASAboticsCode
==============

All the Code for NASAbotics

===============================================================

**Running the Aggregator**

First, on the Odroid, navigate to the NASAboticsCode folder, and run the ODroid script:
bash ODroidStart.sh

Then, on your own computer, run the Command script:
bash CommandStart.sh

Wait a few seconds, then open a new terminal on your own computer. Run the image_view node:
rosrun image_view image_view image:=rotated/image 

**Recovering from Crashes**
First, ssh into the Odroid:
ssh odroid@odroid

Once logged in, run the kill all processes command:
killall5

Your ssh session will end after this command; log into the ODroid again, and run the ODroidStart script as before. 
roscore on the Odroid will now be running, and you can once again connect to it with Command Nodes.  


Use name conventions from wiki.ros.org/ROS/Patterns/Conventions

**How to use github**

// add some stuff to the repository

git add . //prepares to add everything changed
git status //tells you what you're about to add
git commit -m "appropriate message about yo code"
git push //officially puts it into the repository

// delete some stuff from the repository

git add -u //prepares to delete what you just deleted
git status //tells you what you're about to delete
git push //officially deletes

// take some stuff from the repository

git pull //updates your repository
