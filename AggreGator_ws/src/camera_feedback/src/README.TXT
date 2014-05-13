Getting Camera Feedback
========================
Haven't tested on Odroid, but if you sudo apt-get install ros-hydro-usb-cam and run 1.) you should be able to subscribe to the topic over wifi and open up regular feed.

PREREQUISITES
-usb_cam package (sudo apt-get install ros-<distro>-usb-cam)
-image_transport plugins (sudo apt-get install ros-<distro>-image-transport-plugins)
	!!Run "rosrun image_transport list_transports"
	If theora is not installed run "rosmake theora_image_transport"

1.) Start up the cam_node to listen for images
	rosrun usb_cam usb_cam_node

2.) OPTION 1:: Open Up Regular Stream
		rosrun image_view image_view image:=usb_cam/image_raw

	OPTION 2::	Open Up Very Compressed Theora Stream
		rosrun image_view image_view image:=usb_cam/image_raw _image_transport:=theora

3.) TODO: Timed low-res video feed
