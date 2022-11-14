import pyautogui
import cv2
import numpy as np
import utils_PyKinectV2 as utils
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectV2
from pykinect2 import PyKinectRuntime


  

'''
The following code was taken from David Navarro Saiz at
https://github.com/DavidNavarroSaiz/DemoKinect/tree/6481d8cda7ea3d82a8940ed4e919fa5c87d65c74
(Basic example implementing PyKinect2, drawing skeleton from Kinect input)
We updated deprecated class functions and targeting the values we want
(Coordinates of Right hand joints)
'''


#############################
### Kinect runtime object ###
#############################
kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Body | 
                                         PyKinectV2.FrameSourceTypes_BodyIndex |
                                         PyKinectV2.FrameSourceTypes_Color |
                                         PyKinectV2.FrameSourceTypes_Depth |
                                         PyKinectV2.FrameSourceTypes_Infrared)

depth_width, depth_height = kinect.depth_frame_desc.Width, kinect.depth_frame_desc.Height # Default: 512, 424
color_width, color_height = kinect.color_frame_desc.Width, kinect.color_frame_desc.Height # Default: 1920, 1080

# while True:
#     ##############################
#     ### Get images from camera ###
#     ##############################
#     if kinect.has_new_body_frame() and \
#        kinect.has_new_body_index_frame() and \
#        kinect.has_new_color_frame() and \
#        kinect.has_new_depth_frame() and \
#        kinect.has_new_infrared_frame():

#         body_frame       = kinect.get_last_body_frame()
#         body_index_frame = kinect.get_last_body_index_frame()
#         color_frame      = kinect.get_last_color_frame()
#         depth_frame      = kinect.get_last_depth_frame()
#         infrared_frame   = kinect.get_last_infrared_frame()
        
#         #########################################
#         ### Reshape from 1D frame to 2D image ###
#         #########################################
#         body_index_img   = body_index_frame.reshape(((depth_height, depth_width))).astype(np.uint8) 
#         color_img        = color_frame.reshape(((color_height, color_width, 4))).astype(np.uint8)
#         depth_img        = depth_frame.reshape(((depth_height, depth_width))).astype(np.uint16) 
#         infrared_img     = infrared_frame.reshape(((depth_height, depth_width))).astype(np.uint16)

#         ###############################################
#         ### Useful functions in utils_PyKinectV2.py ###
#         ###############################################
#         align_color_img = utils.get_align_color_image(kinect, color_img)
#         align_color_img = utils.draw_bodyframe(body_frame, kinect, align_color_img) # Overlay body joints on align_color_img
#         body_index_img  = utils.color_body_index(kinect, body_index_img) # Add color to body_index_img

#         ######################################
#         ### Display 2D images using OpenCV ###
#         ######################################
#         color_img_resize = cv2.resize(color_img, (0,0), fx=0.5, fy=0.5) # Resize (1080, 1920, 4) into half (540, 960, 4)
#         depth_colormap   = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=255/1500), cv2.COLORMAP_JET) # Scale to display from 0 mm to 1500 mm
#         infrared_img     = cv2.convertScaleAbs(infrared_img, alpha=255/65535) # Scale from uint16 to uint8
        
#         # cv2.imshow('body index', body_index_img)                    # (424, 512)
#         # cv2.imshow('color', color_img_resize)                       # (540, 960, 4)
#         cv2.imshow('align color with body joints', align_color_img) # (424, 512)
#         # cv2.imshow('depth', depth_colormap)                         # (424, 512)
#         # cv2.imshow('infrared', infrared_img)                        # (424, 512)


#     key = cv2.waitKey(30)
#     if key==27: # Press esc to break the loop
#         break

# kinect.close()
# cv2.destroyAllWindows()

'''
David Navarro Saiz code ends here.
'''
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import ctypes
import _ctypes
import pygame
import sys

if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread


myfont = pygame.font.SysFont("monospace", 40)

# colors for drawing different bodies 
SKELETON_COLORS = [pygame.color.THECOLORS["red"], 
                  pygame.color.THECOLORS["blue"], 
                  pygame.color.THECOLORS["green"], 
                  pygame.color.THECOLORS["orange"], 
                  pygame.color.THECOLORS["purple"], 
                  pygame.color.THECOLORS["yellow"], 
                  pygame.color.THECOLORS["violet"]]

def distance(x1, y1, x2, y2):
    return ((x1-x2)**2+(y1-y2)**2)**.5

class BodyGameRuntime(object):
    def __init__(self):
        pygame.init()

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1), 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)

        pygame.display.set_caption("Kinect for Windows v2 Body Game")

        # Loop until the user clicks the close button.
        self._done = False

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Kinect runtime object, we want only color and body frames 
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)

        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
        self._frame_surface = pygame.Surface((self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)
        self.height = self._kinect.color_frame_desc.Height
        self.width = self._kinect.color_frame_desc.Width

        # here we will store skeleton data 
        self._bodies = None
        self.restart(False)
    
    def restart(self, gameMode1):
        self.ballPos = self._kinect.color_frame_desc.Width/2, self._kinect.color_frame_desc.Height/2
        self.ballDx = -10
        self.ballDy = 22
        self.barWidth = 50
        self.barHeight = 200
        self.rect1X = 0
        self.rect1Y = 0
        self.rect2X = self.width - self.barWidth
        self.rect2Y = 0
        self.gameOver = False
        self.ballR = 20
        self.p1won = 'None'
        self.isGameMode1 = gameMode1

    def drawPongBall(self):
        pygame.draw.circle(self._frame_surface, 'green', self.ballPos, self.ballR)
        pygame.draw.rect(self._frame_surface, 'orange', pygame.Rect(self.rect1X,self.rect1Y-(self.barHeight/2), self.barWidth, self.barHeight ))
        pygame.draw.rect(self._frame_surface, 'blue', pygame.Rect(self.rect2X,self.rect2Y-(self.barHeight/2), self.barWidth, self.barHeight ))

    def drawHandPos(self):
        pygame.draw.circle(self._frame_surface, 'white', (self.rect1X,self.rect1Y), 50)
        pygame.draw.circle(self._frame_surface, 'white', (self.rect2X,self.rect2Y), 50)

    def draw_body_bone(self, joints, jointPoints, color, joint0, joint1, bodyIndex):
        joint0State = joints[joint0].TrackingState;
        joint1State = joints[joint1].TrackingState;

        # both joints are not tracked
        if (joint0State == PyKinectV2.TrackingState_NotTracked) or (joint1State == PyKinectV2.TrackingState_NotTracked): 
            return

        # both joints are not *really* tracked
        if (joint0State == PyKinectV2.TrackingState_Inferred) and (joint1State == PyKinectV2.TrackingState_Inferred):
            return

        # ok, at least one is good 
        start = (jointPoints[joint0].x, jointPoints[joint0].y)
        end = (jointPoints[joint1].x, jointPoints[joint1].y)

        try:
            if joint0==PyKinectV2.JointType_HandRight:
                x , y = start
                # Check which player it is
                midLine = self.width/2
                if 0<=x<midLine:
                    if self.isGameMode1:
                        self.rect1X = x
                    self.rect1Y = y
                elif midLine<=x<=self.width:
                    if self.isGameMode1:
                        self.rect2X = x
                    self.rect2Y = y
                 
                
            #pygame.draw.circle(self._frame_surface, color, end, 8)
        except: # need to catch it due to possible invalid positions (with inf)
            pass


    def draw_body(self, joints, jointPoints, bodyIndex, color):        
        # Right Arm    
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight, bodyIndex);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandRight, PyKinectV2.JointType_HandTipRight, bodyIndex);

        # Left Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft, bodyIndex);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandTipLeft, bodyIndex);


    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    ##############################
    ### Get images from camera ###
    ##############################
    if kinect.has_new_body_frame() and \
       kinect.has_new_body_index_frame() and \
       kinect.has_new_color_frame() and \
       kinect.has_new_depth_frame() and \
       kinect.has_new_infrared_frame():

        body_frame       = kinect.get_last_body_frame()
        body_index_frame = kinect.get_last_body_index_frame()
        color_frame      = kinect.get_last_color_frame()
        depth_frame      = kinect.get_last_depth_frame()
        infrared_frame   = kinect.get_last_infrared_frame()
        
        #########################################
        ### Reshape from 1D frame to 2D image ###
        #########################################
        body_index_img   = body_index_frame.reshape(((depth_height, depth_width))).astype(np.uint8) 
        color_img        = color_frame.reshape(((color_height, color_width, 4))).astype(np.uint8)
        depth_img        = depth_frame.reshape(((depth_height, depth_width))).astype(np.uint16) 
        infrared_img     = infrared_frame.reshape(((depth_height, depth_width))).astype(np.uint16)

        ###############################################
        ### Useful functions in utils_PyKinectV2.py ###
        ###############################################
        align_color_img = utils.get_align_color_image(kinect, color_img)
        align_color_img = utils.draw_bodyframe(body_frame, kinect, align_color_img) # Overlay body joints on align_color_img
        body_index_img  = utils.color_body_index(kinect, body_index_img) # Add color to body_index_img

        ######################################
        ### Display 2D images using OpenCV ###
        ######################################
        color_img_resize = cv2.resize(color_img, (0,0), fx=0.5, fy=0.5) # Resize (1080, 1920, 4) into half (540, 960, 4)
        depth_colormap   = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=255/1500), cv2.COLORMAP_JET) # Scale to display from 0 mm to 1500 mm
        infrared_img     = cv2.convertScaleAbs(infrared_img, alpha=255/65535) # Scale from uint16 to uint8
        
        # cv2.imshow('body index', body_index_img)                    # (424, 512)
        # cv2.imshow('color', color_img_resize)                       # (540, 960, 4)
        # cv2.imshow('align color with body joints', align_color_img) # (424, 512)
        # cv2.imshow('depth', depth_colormap)                         # (424, 512)
        # cv2.imshow('infrared', infrared_img)                        # (424, 512)
    

    def updateBallPos(self):
        if not self.gameOver:
            x, y= self.ballPos 
            x += self.ballDx
            y += self.ballDy
            barY0 = self.rect1Y-100
            barY1 = self.rect1Y+self.barHeight -100
            bar2Y0 = self.rect2Y-100
            bar2Y1 = self.rect2Y+self.barHeight -100
            # Bounce if vertically off screen
            if 20>y or y>self.height-100:
                self.ballDy = -self.ballDy
                y += self.ballDy
            
            # Check if ball passes the left player, p1
            if x-self.ballR<self.rect1X +self.barWidth:
                if (barY0)<=y<=(barY1):
                    x = self.rect1X + self.barWidth + self.ballR
                    dToMiddleY = y - (barY0+ barY1)/2
                    dampeningFactor = 3 # smaller = more extreme bounces
                    self.ballDy = dToMiddleY / dampeningFactor
                    self.ballDx = -self.ballDx
                    y -= self.ballDy
                    self.ballPos = x,y
                else:   
                    return self.gameIsOver(False)

            # Check if ball passes the right player, p2
            if x+self.ballR>self.rect2X:
                if (bar2Y0)<=y<=(bar2Y1):
                    x = self.rect2X - self.ballR
                    dToMiddleY = y - (bar2Y0+ bar2Y1)/2
                    dampeningFactor = 3 # smaller = more extreme bounces
                    self.ballDy = dToMiddleY / dampeningFactor
                    self.ballDx = -self.ballDx
                    y -= self.ballDy
                    self.ballPos = x,y
                else:
                    return self.gameIsOver(True)
            self.ballPos = x,y

    def gameIsOver(self, player1Won):
        self.p1won = player1Won
        self.isGameMode1 = True
        pygame.time.Clock().tick
        self.gameOver = True

    def userClapped(self):
        if self._bodies is not None: 
            for i in range(0, self._kinect.max_body_count):
                body = self._bodies.bodies[i]
                if not body.is_tracked: 
                    continue 
                
                joints = body.joints 
                # convert joint coordinates to color space 
                joint_points = self._kinect.body_joints_to_color_space(joints)
                clapped, clapPoint = self.checkIfClapping(joints, joint_points)
                if clapped:
                    return True, clapPoint
        return False, None
    
    def checkIfClapping(self, joints, joint_points):
        sensitivity = 80
        rightHand = joint_points[PyKinectV2.JointType_HandRight]
        rightX, rightY = rightHand.x, rightHand.y
        leftHand = joint_points[PyKinectV2.JointType_HandLeft]
        leftX, leftY = leftHand.x, leftHand.y
        if distance(rightX, rightY, leftX, leftY)<= sensitivity:
            return True, (rightX, rightY)
        return False, None

    def run(self):
        SET_POS = pygame.USEREVENT + 1
        if not self.gameOver:
            pygame.time.set_timer(SET_POS, 30)
        # -------- Main Program Loop -----------
        while not self._done:
            # --- Main event loop
            for event in pygame.event.get(): # User did something 
                if event.type == pygame.QUIT: # If user clicked close
                    self._done = True # Flag that we are done so we exit this loop

                elif event.type == pygame.VIDEORESIZE: # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'], 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)

                elif event.type == SET_POS:
                    self.updateBallPos()
                    
            # --- Game logic should go here
            

            # --- Getting frames and drawing  
            # --- Woohoo! We've got a color frame! Let's fill out back buffer surface with frame's data 
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self._frame_surface)
                frame = None

            # --- Cool! We have a body frame, so can get skeletons
            if self._kinect.has_new_body_frame(): 
                self._bodies = self._kinect.get_last_body_frame()

            # --- draw skeletons to _frame_surface
            if self._bodies is not None: 
                for i in range(0, self._kinect.max_body_count):
                    body = self._bodies.bodies[i]
                    if not body.is_tracked: 
                        continue 
                    
                    joints = body.joints 
                    # convert joint coordinates to color space 
                    joint_points = self._kinect.body_joints_to_color_space(joints)
                    self.draw_body(joints, joint_points, i, SKELETON_COLORS[i])

            if not self.isGameMode1:
                pygame.draw.rect(self._frame_surface, (0,0,0), pygame.Rect(0,0, self.width,self.height ))

            self.drawPongBall()
            if self.gameOver:
                #pygame.draw.rect(self._frame_surface, (41,51,92), pygame.Rect(0,0, self.width,self.height ))
                imp = pygame.image.load("Start screen.png").convert()
 
                # Using blit to copy content from one surface to other
                self._frame_surface.blit(imp, (0, 0))


                if self.p1won != 'None':
                    winner = 'Orange' if self.p1won else 'Blue'
                    label = myfont.render(f'Game over! The {winner} Player Won!', 100, 'white')
                    text_rect = label.get_rect(center=(self.width/2, self.height/2+self.height/3))
                    self._frame_surface.blit(label, text_rect)

                label2 = myfont.render(f'Stand far apart. Keep your hands apart during play. Clap to restart.', 100, 'white')
                text_rect2 = label2.get_rect(center=(self.width/2, self.height/2+self.height/3+50))
                self._frame_surface.blit(label2, text_rect2)

                self.drawHandPos()

                clapped, clapPoint = self.userClapped()
                if clapped:
                    x, y = clapPoint
                    gameMode1 = True
                    if 0<x<self.width/3:
                        gameMode1 = False
                        self.restart(gameMode1)
                    elif 2*self.width/3<x<self.width:
                        gameMode1 = True
                        self.restart(gameMode1)

                    
            # --- copy back buffer surface pixels to the screen, resize it if needed and keep aspect ratio
            # --- (screen size may be different from Kinect's color frame size) 
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height));
            self._screen.blit(surface_to_draw, (0,0))
            surface_to_draw = None
            pygame.display.update()

            # --- Go ahead and update the screen with what we've drawn.
            pygame.display.flip()

            # --- Limit to 60 frames per second
            self._clock.tick(144)

        # Close our Kinect sensor, close the window and quit.
        self._kinect.close()
        pygame.quit()


__main__ = "Kinect v2 Body Game"
game = BodyGameRuntime();
game.run();