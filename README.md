# WELCOME TO THREE SOCKS PONG

################################################################################
##################### WELCOME TO THREE SOCKS PONG ##############################
################################################################################

--------------------------------------------------------------------------------
-------------------------------Table of Contents--------------------------------
--------------------------------------------------------------------------------
 
 * Introduction
 * Required Files
 * Required Modules
 * How to Run
 * How to Play
 * References


################################################################################
############################### Introduction ###################################
################################################################################

    Three Socks Pong is a pong game that uses the kinect motion sensor to let two 
players face off in a competitive game of Pong! The game itself has two different
gamemodes: gamemode 1 mimics classic pong since both of the paddles are fixed to 
one side of the screen, each player controls an indivial paddle and the goal is to 
not let the ball pass your paddle. Gamemode 2, works in a similar way, expect the 
paddles are not fixed to the x-axis, so you can face off in a 2d match of pong
rather than a  vertical only experience. 

################################################################################
############################### Required Files #################################
################################################################################

ultimateSock.py
utils_PyKinectV2.py
Start screen.png

################################################################################
############################### Required Modules #################################
################################################################################

Kinect for Windows SDK 2.0
pyKinect2
pygame
numpy
comtypes
openCV

################################################################################
############################### How to run #####################################
################################################################################

**MUST HAVE A KINECT TO PLAY**
(0) Download Kinect for Windows and plug the Kinect into your computer
(1) Download all files listed in required files into a parent directory
(2) Install all required modules at the level of parent directory (we recommend using pip for this)
(3) Run the ultimateSock.py file to start the game.

################################################################################
################################  How to Play ##################################
################################################################################

1)The game automatically defaults to the Traditional gamemode but after a player
loses you have the ability to change between the Traditional and Fun gamemodes. 

2) Start by positioning both players on oposite sides of the screen with their 
right hand extended out and their palm facing foward

3) Choose a gamemode by having the player on the corresponding side of the screen
clap to initate the game that is on that side

4) Once the player claps the gamemode will begin, if you are on gamemode one you 
do not have to keep your hand fixed, as it tracks only vertical movements so you 
can move left and right but the paddle will only move up and down

5) If gamemode 2 is selected beware of having any player too close to the middle 
as the speed of the ball in gamemode 2 is a lot faster so you want to give each 
player time to react. 

6) Last pieces of advice:
Make sure your right hand is extended at all times when playing the game (the
tracking software can get very sensitive)



################################################################################
############################### References #####################################
################################################################################

    This game was built in the pyKinect2 platform, which connects Kinect input with 
Python code. Our implementation of pyKinect2 was heavily based on the example
made by David Navarro Saiz (GOAT) at https://github.com/DavidNavarroSaiz/DemoKinect/tree/6481d8cda7ea3d82a8940ed4e919fa5c87d65c74
(Basic example implementing PyKinect2, drawing skeleton from Kinect input).
The start and end of Saiz's code is denoted within ultimateSOck.py and utils_PyKinectV2.py
We made minor adjustments within his code to update deprecated class functions 
and target the values we want. Beyond that, the code we did was in addition to 
his basic frame.
    We also owe a thanks to our 112 professors Pat Virtue and David Kosbie for
showing us Pong.





