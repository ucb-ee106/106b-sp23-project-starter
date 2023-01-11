#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import sys

#Import the desired message type from the /msg directory of
#the desired package.

# EDIT *********************************************************
from PACKAGE.msg import MESSAGE
# **************************************************************

#Define the method which contains the main functionality of the node.
def controller(turtle_name):
  '''
  Controls the turtle denoted by turtle_name via user keyboard input

  turtle_name - the first command line argument, which is the name of the turtle to control
  '''

  #Run this program as a new node in the ROS computation graph 
  #called /controller.
  rospy.init_node('controller', anonymous=True)

  #Create an instance of the rospy.Publisher object which we can 
  #use to publish messages to a topic.
  # EDIT *******************************************************
  pub = rospy.Publisher('TOPIC', MESSAGE TYPE, queue_size=10)
  # ************************************************************
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():

    # YOUR CODE HERE *******************************************
    
    user_input = input('Helpful user prompt')



    message = 
    # **********************************************************


    # Publish our string to the topic you define
    pub.publish(message)
    
    # Use our rate object to sleep until it is time to publish again
    r.sleep()
      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the controller method
  try:
    controller(sys.argv[1])
  except rospy.ROSInterruptException: pass
