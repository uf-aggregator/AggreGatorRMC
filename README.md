#NASAboticsCode - Splash Page (WIP)
This is the root director.

Keep up to date with non-code shenanigans at [our website](www.ufaggregator.com "UF Aggregator Official Homepage")

----------------------------------------------------------

###Running the Aggregator

First, on the Odroid, navigate to the NASAboticsCode folder, and run the ODroid script:
```
bash ODroidStart.sh
```

Then, on your own computer, run the Command script:

```
bash CommandStart.sh
```

Wait a few seconds, then open a new terminal on your own computer. Run the image_view node:

```
rosrun image_view image_view image:=rotated/image 
```
----------------------------------------------------------

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

See our [style guide](www.github.com/fnivek/NASAboticsCode/Getting_Started/Style_Guide/ "UF Aggregator Style Guide") for our code and naming practices

----------------------------------------------------------

