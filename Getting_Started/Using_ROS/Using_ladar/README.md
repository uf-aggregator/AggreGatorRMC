#Using_ladar

This is everything you need to know about running our ladar package.

------------------------

###Running _ladar_

To run the ladar node and begin publishing/subscribing data to the /scan topic,

#####Start the urg_node

Information taken from [ROS Wiki](http://wiki.ros.org/hokuyo_node/Tutorials/UsingTheHokuyoNode)

Plug the hokuyo laser rangefinder into the machine. Then check for adequate permissions

        sudo ls -l /dev/ttyACM0
    
If the third octal permissions are not `-rx` then run,

        sudo chmod a+rw /dev/ttyACM0        

It won't hurt to run this just to be sure.

Now run the urg_node (make sure roscore is running),

        rosrun urg_node urg_node
        
After a moment, it should say it is streaming data.


#####Run the node

        rosrun ladar ladar_node