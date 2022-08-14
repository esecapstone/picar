# picar
ESE 498/499 Capstone Pi-Car Project Official Github Repository

# Dependencies
  
  ## Hardware: 
    
    RC Car with Traxxas XL-5 electronic speed control (ESC) and servo motor for steering
    
    Raspberry Pi 4 (at least 4GB memory, 32GB SD card storage)
    
    Raspberry Pi Camera Module v2 [See note below]
    
    HC-SR04 Ultrasonic sensor (for distance measurement)
    
    PCA9685 16-Channel 12-Bit PWM Driver
    
  
  
  Note: Pi Camera v2 is preferred; v1 might be usable but is not fully tested/calibrated to the control system.
    Note that v1 has a different sensor resolution and field of view compared to v2. 
    Using v1 will require simple changes to the pi-cam node as well as the calibrator.
    Control algorithm may also need to be altered if using v1. 
  
  
  
  ## Software:
    
    Raspberry Pi OS: Raspbian Buster
    
    Robot Operating System (ROS) Noetic
    
    OpenCV
    
    Python 3
    
# Warnings (Please Read)

  This car and our supplied software IS NOT a TOY. Use utmost caution when working with this vehicle
  
  Read the datasheet of all components you are working with 
  
  Fully insulate the thick power wires on the XL-5 ESC if you are soldering connections to them. These wires can deliver up to 100 Amps peak/15 Amps continous current which can be dangerous. 
  
  There is a start-up sequence implemented in the PWM node which will command the XL-5 ESC to turn the rear wheels at a very high speed.
  
  During this motor start-up sequence, make sure the car is secure with its rear wheels and rear axles elevated from the ground and clear from skin/wire contact. 
  
  Make sure the path is clear before launching the vehicle. 
  
  Insulate the pi from any metal surfaces it may come into contact with to prevent short-circuits. 



# Repository Rules and How to Use GitHub

To prevent catkin_make issues or pushing/pulling bad code, don't clone this repository directly to your catkin workspace: 
Instead, clone this repository to your home directory or elsewhere.
Copy packages to the catkin workspace when needed (remember to source and catkin_make after copying to catkin workspace!)

In your home directory (You only ever need to do this once): 

**git clone https://github.com/esecapstone/picar**



  # How to Push Code to GitHub

  Make sure your code works locally as a ROS node, never push non-working code.
  
  Always do this step below before you change anything to your local or online github repository

  **git pull**

  Delete the package in your local repo that you want to replace 

  Copy your entire ros package for the node (with CMakeLists.txt and package.xml) to your local repo to replace the old one.

  **git add ros_package_name**

  **git commit -m 'your message here'**

  **git push**



# Special Thanks
Prof. Dorothy Wang

Alejandro Acevedo Guillot

Hugh Tillman James Jr

Danny Andreev

Classmates and Alums
