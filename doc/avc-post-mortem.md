## Some problems I worked through upon arriving

* Setting up wifi on watbotnet and router.
* Getting reverse to work correctly.
* Bot wasn’t tracking odometry. Turned out to be the sun flooding the qrd1114 sensors on the encoders. Attached cardboard to block out light.

## How it went down

Round 1
	Bot doesn’t start. ESC light was green, never synced with Arduino. Turned out to be signal wire came loose from Arduino. Amazingly, the bot next to me failed to start as well!

Round 2
	Bot started, veered left, blocked the Lockheed team’s bot. Lots of fun.

Round 3
	Bot looked ok to go. Flipped switch. Nothing. Turns out the bot had lost the wifi connection so the terminal on which the launch file had executed was lost. Next time, start it as a systemd service.
	
## Some things I learned the past two days

* Most everyone fails spectacularly their first year attending.

* Many people were using Lidar. Many also pointed out that Lidars have trouble in the sun.

* Many teams were using vision sensing.

* Raspberry pi is underpowered for ROS.

* The Arduino Nano does not have enough memory for all but the most rudimentary ROS node implementations.

* I originally thought having a bunch of small Arduino-based nodes would help with individual node efficiency and code management/separation. This was a fallacy, as the number of physical nodes made the layout of the bot’s guts more complicated. Also, managing which nodes had which versions of code uploaded was difficult to track.

* The ROS navigation stack, while supposedly very capable, has many bells and whistles that make it a beast to tune and get working properly. It might be easier to develop your own navigation stack that, while not as generally capable as ROS, would get the job done in the AVC domain. The idea is; if it fails, we are more likely to know why it failed having developed it ourselves.

* Most everyone has trouble making that first turn.

* Sometimes, the simpler algorithm, implementation, or approach was the most effective. Complexity doesn’t necessarily mean better performance.

* Always show up! Don’t be scared or embarrassed to run a bot that fails miserably. The knowledge/lessons gained from attending were far greater than the last several months of late-night development. Key is to iterate quickly and run the bot as much as possible in a real scenario (it doesn’t have to match the avc environment, it just needs to be thorough enough to challenge the regular operation of the bot).

## Some memorable quotes

* Some admitted to me the first year they went, they did not even line up their bot.
* One gentleman said my bot was a lot for one person to create.
* A little girl said she liked the sonar array and how it looked!

## Suggestions for next development cycle

* Try to do a dry run as early as possible to help flush out simple issues. Don’t wait until the last week to start test driving your vehicle.
* Do a dry run of taking your vehicle outside the regular test environment. I.e. take it to a parking lot and go through the motions of setting up your vehicle and attempting to run it through a pattern.


## Some immediate changes I’d like to make

	Simpler ESC without the weird reverse control.
	Avc github project restructure to make workflow easier.
	Better shocks.
	straight/true steering.
	Proper battery holder.
	Finer encoder counts.
	Filter to smooth out velocity calculation
	Play with ROS ekf to see if the extraneous y velocity can be reduced.
	Test how many messages you can send from the IMU to the PI.
	Test the maximum baud of the rosserial library.
	Test the GPS and nmea driver of ROS to see what happens.

## Strategies for bot next year

Button up the vehicle more so important wires can’t come loose during operation.

Add RealSense vision processing.

Pare down the number of Arduinos to one or two if possible. Use a Mega perhaps or Due.
Ditch the PI for a beefier processor (i.e Nvidia Jetson).

Ditch planning stack of ROS, but keep the messaging structure (at least for development). The idea is ROS provides some great visualization, logging, and playback capabilities. These can help develop the non-vision sensors.
