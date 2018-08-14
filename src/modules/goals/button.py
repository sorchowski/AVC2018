#shamelessly copied from https://raspberrypihq.com/use-a-push-button-with-raspberry-pi-gpio/
# and https://thepihut.com/blogs/raspberry-pi-tutorials/27968772-turning-on-an-led-with-your-raspberry-pis-gpio-pins

#pinout diagram for raspberry pi 3: https://www.jameco.com/Jameco/workshop/circuitnotes/raspberry-pi-circuit-note.html


import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library
def button_callback(channel):
    print("Button was pushed!")
GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
#GPIO.setup(??,GPIO.OUT) # for led indicator
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
GPIO.add_event_detect(10,GPIO.RISING,callback=button_callback) # Setup event on pin 10 rising edge
message = input("Press enter to quit\n\n") # Run until someone presses enter


#LED Control TODO
GPIO.output(18,GPIO.HIGH)
time.sleep(1)
print "LED off"
GPIO.output(18,GPIO.LOW)


GPIO.cleanup() # Clean up