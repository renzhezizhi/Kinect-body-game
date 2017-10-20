# -*- coding: utf-8 -*-
"""

@author: lqy
"""

from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
import ctypes
import pygame
import time,cv2,math
import numpy as np

import myfunction as mf

fps = 30
SKELETON_COLORS = [pygame.color.THECOLORS["red"], 
                  pygame.color.THECOLORS["blue"], 
                  pygame.color.THECOLORS["green"], 
                  pygame.color.THECOLORS["orange"], 
                  pygame.color.THECOLORS["purple"], 
                  pygame.color.THECOLORS["yellow"], 
                  pygame.color.THECOLORS["violet"]]
                  
class BodyGameRuntime(object):
    def __init__(self):
        pygame.init()

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()
        
        # Set the width and height of the screen [width, height]
        self._infoObject = pygame.display.Info()
        self._screen = pygame.display.set_mode((self._infoObject.current_w >> 1, self._infoObject.current_h >> 1), 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)

        pygame.display.set_caption("Kinect Body detection")

        # Loop until the user clicks the close button.
        self._done = False
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body|PyKinectV2.FrameSourceTypes_Depth|PyKinectV2.FrameSourceTypes_BodyIndex)    
        # back buffer surface for getting Kinect color frames, 32bit color, width and height equal to the Kinect color frame size
        self._frame_surface = pygame.Surface((self._kinect.color_frame_desc.Width, self._kinect.color_frame_desc.Height), 0, 32)
        # here we will store skeleton data 
        self._bodies = None
        
        self._text_timer = [30,30,30,30]  # timers, [push, draw, backup1, backup2]
        
        
        
        time.sleep(3)


        if self._kinect.has_new_color_frame():
            print ('extracting all information....')
        else:
            print 'failed to extract.....'
        
                
#========================= original part =======================================
    def draw_body_bone(self, joints, jointPoints, color, joint0, joint1):
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
            pygame.draw.line(self._frame_surface, color, start, end, 8)
            #lines(Surface, color, closed, pointlist, width=1)
        except: # need to catch it due to possible invalid positions (with inf)
            pass

    def draw_body(self, joints, jointPoints, color):
        # Torso
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Head, PyKinectV2.JointType_Neck);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_Neck, PyKinectV2.JointType_SpineShoulder);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_SpineMid);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineMid, PyKinectV2.JointType_SpineBase);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineShoulder, PyKinectV2.JointType_ShoulderLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_SpineBase, PyKinectV2.JointType_HipLeft);
        
        # Right Arm    
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderRight, PyKinectV2.JointType_ElbowRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowRight, PyKinectV2.JointType_WristRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristRight, PyKinectV2.JointType_HandRight);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandRight, PyKinectV2.JointType_HandTipRight);

        # Left Arm
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ShoulderLeft, PyKinectV2.JointType_ElbowLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_ElbowLeft, PyKinectV2.JointType_WristLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_WristLeft, PyKinectV2.JointType_HandLeft);
        self.draw_body_bone(joints, jointPoints, color, PyKinectV2.JointType_HandLeft, PyKinectV2.JointType_HandTipLeft);


    # draw the color frame to pygame surface
    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()
     
    # add text onto surface
    # time_no means the No. of timer for different gestureto use (0 for push, 1 for draw)
    def typetext(self,string,pos,color = (255,255,0),fontsize=60,bold=False, timer_no = -1):
        myfont = pygame.font.SysFont("Arial", fontsize,bold)
        label = myfont.render(string, 1, color)
        if ( timer_no < 0):
            self._frame_surface.blit(label, pos)
        else:
            if(self._text_timer[timer_no] > 0):
               self._frame_surface.blit(label, pos)
               self._text_timer[timer_no] -= 1
    
    # draw the window frame inside the color frame, transparency=0.0~1.0
    def updateWindowInKinect(self,color_frame_array,window_frame,transparency):
        DE_RATIO=2
        # window_frame=window_frame_0[50:400,100:412,:] # some part of the frame from video
        window_frame_size_orig=window_frame.shape

        winodw_wtoh_ratio=window_frame_size_orig[1]*1.0/window_frame_size_orig[0]
        window_frame_size=(int(1080/DE_RATIO*winodw_wtoh_ratio),int(1080/DE_RATIO))    # [w,h]
        window_frame=cv2.resize(window_frame,window_frame_size)
        left_top=[1920-window_frame_size[0],1080-window_frame_size[1]] # position of windowed frame in color_frame [x,y]

        color_frame_array[left_top[1]:,left_top[0]:,:3]=window_frame*transparency+color_frame_array[left_top[1]:,left_top[0]:,:3]*(1-transparency)
     
    # control the window video to jump forward or backwark or freeze
    def controlWindowFrame(self,color_frame_array,window_video,window_frame_loc,control_code,last_window_frame,transparency = 1):
        if control_code==0: # do nothing
            retval,window_frame=window_video.read()
        elif control_code==1:   # forward or backward
            window_video.set(cv2.cv.CV_CAP_PROP_POS_MSEC,window_frame_loc/30.0*1000)
            retval,window_frame=window_video.read()
        elif control_code==2:   #freeze
            if len(last_window_frame)>0: 
                window_frame=last_window_frame
            else:   # it is the first frame at the beginning of video play
                retval,window_frame=window_video.read()
        elif control_code==3:   #start over
            window_video.set(cv2.cv.CV_CAP_PROP_POS_MSEC,window_frame_loc/30.0*1000)
            retval,window_frame=window_video.read()
        
        # update the window frame
        self.updateWindowInKinect(color_frame_array,window_frame,transparency)  
        
        return window_frame #the last window frame
        
    # for draw gesture
    def drawGestureDetection(self, body, joints_color, hand_record_x, miss_count, draw_flag):
        hand_state    = body.hand_left_state
        hand_position = joints_color[7]  # left hand
        MISS_THRESH = 5
        ACTIVE_THRESH = int(abs(joints_color[1].y-joints_color[0].y)*1.5) # equals to the distance between
                                                                        # spine shoulder and mid
        
        if(hand_record_x > -1 ):   # recorded before
            if(hand_state == HandState_Lasso):
                if(abs(hand_position.x-hand_record_x) > ACTIVE_THRESH):
                    draw_flag = not draw_flag
                    hand_record_x = -1
                    miss_count  = 0
                    # if draw_flag becomes true, then need 30 seconds to show system text
                    if(draw_flag):
                        self._text_timer[2] = 30 # for system text to show for 30 frames
                        
            else:
                miss_count += 1
            if( miss_count > MISS_THRESH):
                hand_record_x = -1
                miss_count = 0
        elif(hand_record_x == -1 and hand_state == HandState_Lasso):   # a new record appear
            hand_record_x=hand_position.x
                
        return draw_flag, hand_record_x, miss_count
        
    # for point and choose gesture
    def handPointControl(self, surface_draw, body, joints_color, gesture_record,rect_list, text_list,\
            window_frame_control_code, window_frame_loc, nFrames):
        control_state=0 # 0=nothing happens; 1=one option is actived;
                        # 2= one option is 'pressed' but not released
        last_no=len(rect_list)-1
        hand_state=body.hand_left_state
        origin = np.array([joints_color[5].x,joints_color[5].y])          # left elbow joint as origin
        point_wrist  = np.array([joints_color[6].x,joints_color[6].y])    # left wrist joint
        point_top    = np.array([rect_list[0][0]+rect_list[0][2],rect_list[0][1]])
        point_bottom = np.array([rect_list[last_no][0]+rect_list[last_no][2],\
                               rect_list[last_no][1]+rect_list[last_no][3]])
                               
        angle_top=mf.angle_between(point_top-origin,(-1,0))
        angle_bottom=mf.angle_between(point_bottom-origin,(-1,0))
        angle_between=angle_top-angle_bottom
        
        angle_pointing=mf.angle_between(point_wrist-origin,(-1,0))
        
        angle_cut_point=[angle_top]
        for i in xrange(1, last_no+1):
            angle_cut_point.append(angle_top-i*1.0/(last_no+1)*angle_between)
        angle_cut_point.append(angle_bottom)
        color_box=[]
        color_text=[]
        pointed_no=-1   # which option is pointed 0~last_no
        
        # extend the outer bounder
        angle_cut_point[0]=math.pi/2-0.5
        angle_cut_point[last_no+1]=-math.pi/2+0.5
        
        for i in xrange(0,last_no+1):
            if(angle_cut_point[i]>angle_pointing>angle_cut_point[i+1]):
                pointed_no=i
                break
        
        for i in xrange(len(rect_list)):
            color_box.append((170,170,150))
            color_text.append((80,80,150))
            
        # the open-closed-open recognition part
        if (hand_state != HandState_NotTracked):
            # after open-closed is trigered
            if (gesture_record[0]==HandState_Open and hand_state==HandState_Closed):
                gesture_record[0]=hand_state
                gesture_record[1]=1 # the open-closed part is trigered
                gesture_record[2]=gesture_record[2]+1
                gesture_record[3]=pointed_no
                control_state=2
                #print 'trigred'
            # avoid ambiguse when close hand (Kinect may not recognize hand state)
            elif(gesture_record[0]==HandState_Open and hand_state==HandState_Unknown):
                gesture_record[0]=HandState_Open
                
            # open-closed has been trige
            elif(gesture_record[0]==HandState_Closed and gesture_record[1]==1): 
                if(hand_state==HandState_Open):
                    if(gesture_record[2]>=5 and gesture_record[3]==pointed_no and pointed_no>-1):
                        control_state=1 # there is an activation of option
                        gesture_record[0] = -1
                        gesture_record[1] = 0
                        gesture_record[2] = 0
                        gesture_record[3] = -1
                        print 'option ',pointed_no+1,' is activated!'
                    else:
                        gesture_record[0] = -1
                        gesture_record[1] = 0
                        gesture_record[2] = 0
                        gesture_record[3] = -1    # reset
                        
                # trigered but not released
                elif(hand_state==HandState_Closed):
                    control_state=2
                    gesture_record[2]=gesture_record[2]+1    # increase the count
            # first frame after reset or beginning
            elif(hand_state != HandState_NotTracked):
                gesture_record[0]=hand_state
        
        # change the color of the trigered or pointed option
        if(0 <= pointed_no <= last_no):
            if(control_state==2):   # triger and pending
                color_box[pointed_no]=(70,70,50)
                color_text[pointed_no]=(180,180,180)
            else:                   # pointed the option
                color_box[pointed_no]=(220,255,200)
                
        # draw all the option and write the label text
        for i in xrange(0,4):
            pygame.draw.rect(surface_draw,color_box[i],rect_list[i])
            self.typetext(text_list[i],(rect_list[i][0]+10,rect_list[i][1]+20),color_text[i])
        
        # control part
        if(pointed_no > -1 and control_state == 1):
            if(window_frame_control_code != 2):  # when it is not paused
                if(pointed_no == 0):    # 1st option
                    if(window_frame_control_code == 0):
                        window_frame_control_code = 2   # pause
                elif(pointed_no ==1):
                    window_frame_control_code = 3   # restart
                    window_frame_loc = 0
                elif(pointed_no ==2):
                    window_frame_control_code = 1   # forward
                    window_frame_loc += 20*3
                    if(window_frame_loc>nFrames-1):
                        window_frame_loc=nFrames-1
                elif(pointed_no ==3):
                    window_frame_control_code = 1   # backward
                    window_frame_loc -= 20*3
                    if(window_frame_loc< 0):
                        window_frame_loc = 0
            elif(pointed_no == 0):      # only when it is paused and start again
                window_frame_control_code = 0   # start
                
            if (control_state != 1):    # when actived, output the activated option number, otherwise, dismiss
                pointed_no = -1
        return pointed_no, window_frame_control_code, window_frame_loc
        
    # detection of pushing gesture of right hand
    def pushGestureDetection(self,body,push_start_flag,palm_posi_record,push_delay_count):
        push_flag = False
        if push_delay_count > 0:
            push_delay_count -= 1
        else:
            joints = body.joints
            # detect beginning of pushing
            temp_diff_z = joints[11].Position.z - palm_posi_record[0]
            if( temp_diff_z < -0.01 and body.hand_right_state == HandState_Open \
                and joints[1].Position.z-joints[11].Position.z > 0.1 ): # the hand is in front of hand
                palm_posi_record[1] = palm_posi_record[0]
                push_start_flag = True
            else:
                push_start_flag = False
                
            # update record
            palm_posi_record[0] = joints[11].Position.z  # right hand
            
            # threshold for range of movement
            if( joints[11].Position.z - palm_posi_record[1] < -0.03 and push_start_flag == True):
                palm_posi_record = [-99.0,-99.0]
                push_start_flag = False
                push_flag = True
                push_delay_count = 15
                print 'push!'
            
        return push_start_flag, push_flag, push_delay_count
    
    def run(self):
        #--------- initial -------       
        global video
        
        start_time = 0
        
        #-for key pressing
        wait_key_count=3
        
        # window video inside the color frame
        file_path='video/3.mp4'
        
        window_video=cv2.VideoCapture(file_path)
        if not window_video.isOpened():
            print 'open video FAILED!'
            
        nFrames = int(window_video.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT)) 
        fps = window_video.get(cv2.cv.CV_CAP_PROP_FPS)
        window_frame_loc=0
        window_frame_control_code=2 # 0=nothing; 1=forward or backward; 2=Pause
        last_window_frame=[]
        
        # gesture control
        draw_flag=False
        hand_record_x = -1
        gesture_miss_time_count=0   # time(frame) count for unvalidated gesture;
                                    # if larger than threshold, then dismiss the 
                                    # information in hand_record (become -1)
        # choose option
        gesture_record = [-1,0,0,-1]# [pre_hand state,pre_triger_state,closed count,pre pointed box]
                                    # pre_hand_state: the hand state of previous frame;
                                    # pre_triger_state: whether the fist part of
                                    #     open-closed-open process has been done;
                                    # the frame count as hand is closed;
                                    # the box was pointed when the open-closed
                                    # state was trigered
        
        # option to point
        pointed_no=-1
        rect_list=[(0,100,400,100),(0,350,400,100),(0,600,400,100),(0,850,400,100)]
        text_option_list=['start/pause','restart','3s forward','3s backward']
        
        ### the new system initialization code
        start_system_flag = False # flag of enter the gesture control system (GCS)
        lock_system_flag = False # flag of lock pattern appear after start the GCS
        push_start_flag = False # flag of start of push gesture
        push_delay_count = 0 # the delay before detect push again when deteced one
        video_section_flag = False # flag of starts video playing
        
        lock_circle_list = [(760,240),(960,240),(1160,240),\
                            (760,440),(960,440),(1160,440),\
                            (760,640),(960,640),(1160,640)]
        chosen_circle_no_list = []
        unlock_circle_pattern = [0,1,4,7] # the pattern (order) of circles in order to unlock, password in other word
        palm_posi_record = [-99,-99]    # [z position of pre_frame, z position when pushing starts]
        
        
        show_skeleton_flag = False
        # -------- Main Program Loop -----------
        while not self._done:
            
            start_time = time.time()
            
            #--key pressing--
            if(wait_key_count<3):
                wait_key_count+=1
            if(pygame.key.get_focused() and wait_key_count>=3):
                press=pygame.key.get_pressed()
                wait_key_count=0
                if press[pygame.K_ESCAPE]==1:
                    self._done=True 
                    #print self.cntno
                    
                if press[pygame.K_s] == 1:
                    # enter the gesture control system
                    start_system_flag = True
                    lock_system_flag = True
                
                if press[pygame.K_h] == 1:
                    show_skeleton_flag = True #press h to show skeleton
                    
                if press[pygame.K_r] == 1: # press r to reset everything
                    window_frame_loc=0
                    window_frame_control_code=2 # 0=nothing; 1=forward or backward; 2=Pause
                    last_window_frame=[]
                    
                    # gesture control
                    draw_flag=False
                    hand_record_x = -1
                    gesture_miss_time_count=0   
                    # choose option
                    gesture_record = [-1,0,0,-1]
                    
                    # option to point
                    pointed_no=-1
                    
                    ### the new system initialization code
                    start_system_flag = False 
                    lock_system_flag = False 
                    push_start_flag = False 
                    push_delay_count = 0 
                    video_section_flag = False 
                    palm_posi_record = [-99,-99]
                    show_skeleton_flag = False
                    
            #--key pressing over--
            
            
            # --- Main event loop
            for event in pygame.event.get(): # User did something
                if event.type == pygame.QUIT: # If user clicked close
                    self._done = True # Flag that we are done so we exit this loop

                elif event.type == pygame.VIDEORESIZE: # window resized
                    self._screen = pygame.display.set_mode(event.dict['size'], 
                                               pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE, 32)
                            
            
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                
                # reshape the color frame array to a 1080 x 1920 x 3 image
                # its original from is a one dimension array which has length 1080*1920*4
                frame1 = frame.reshape(1080,1920,4)[:,:,:3]
                
                # --- the video playing section -----
                if(start_system_flag == True and video_section_flag == True):
                    
                    # insert a window frame in the original color frame and control it 
                    last_window_frame=self.controlWindowFrame(frame1,window_video,\
                        window_frame_loc,window_frame_control_code,last_window_frame, 0.9)                
                    if(window_frame_control_code!=2):
                        window_frame_control_code=0
                    
                    self.draw_color_frame(frame, self._frame_surface)
                    self.typetext('Video Play Part',(800,60))
                    self.typetext('Unlocked!',(900,120),timer_no=3)
                    # the draw gesture recognition
                    draw_flag,hand_record_x,gesture_miss_time_count = self.drawGestureDetection( \
                            body,joint_points,hand_record_x,gesture_miss_time_count,draw_flag)
                    # when the menu appears, use arm to choose
                    if(draw_flag):
                        pointed_no,window_frame_control_code,window_frame_loc=self.handPointControl(\
                        self._frame_surface,body,joint_points,gesture_record,rect_list,text_option_list,\
                        window_frame_control_code,window_frame_loc,nFrames)
                        
                        self.typetext('Menu on the left',(800,120),timer_no=2)
                        
                    # if the window frame video is played to the end,
                    # repeat it from the start
                    if(window_frame_control_code!=2):
                        window_frame_loc+=1
                    if(window_frame_loc>=nFrames):
                        window_frame_loc=0
                        window_video.set(cv2.cv.CV_CAP_PROP_POS_MSEC,0)
                else:
                    self.draw_color_frame(frame, self._frame_surface)                         
                
                
            
            #--- get skeleton information ---
            no_one_tracked = True
            if self._kinect.has_new_body_frame(): 
                self._bodies = self._kinect.get_last_body_frame()                         
            
            # find if there is anyone being tracked
            for body_i in xrange(0,6):
                no_one_tracked = no_one_tracked and (not self._bodies.bodies[body_i].is_tracked)
                
            if no_one_tracked:
                self.typetext('No human be detected ',(100,100))
            else:
                closest_ID=-1
                cSS_dist=20 #closest SpineShoulder distance
                
                # find the cloest person to the camera
                for i in range(0, self._kinect.max_body_count):                    
                    body = self._bodies.bodies[i]
                    
                    if not body.is_tracked: 
                        #self.typetext('No human be detected ',(100,100))
                        continue
                    if body.joints[20].Position.z<=cSS_dist:
                        closest_ID=i
                        cSS_dist=body.joints[20].Position.z
                        
                if (closest_ID!=-1):
                    body = self._bodies.bodies[closest_ID]
                    joints = body.joints
                    
                    # transfer the 3D locations of joints to color space and depth space
                    joint_points = self._kinect.body_joints_to_color_space(joints)
                    
                    if (show_skeleton_flag):
                        # assign color to the circle on hand 
                        # the color denote the state of hand
                        color_1=[0,0,0]
                        if(body.hand_left_state == HandState_Lasso):  # Lasso
                            color_1=[255,0,0]
                        elif(body.hand_left_state == HandState_Closed):
                            color_1=[0,255,0]
                        elif(body.hand_left_state == HandState_Open):
                            color_1=[0,0,255]
                        elif(body.hand_left_state == HandState_Unknown):
                            color_1=[255,255,255]
                        
                        # draw circle on the hand to indicate hand state
                        if(0<=joint_points[7].x<1920 and 0<=joint_points[7].y<1080):
                            pygame.draw.circle(self._frame_surface, color_1,\
                                [int(joint_points[7].x),int(joint_points[7].y)],50,15)
                        if(0<=joint_points[11].x<1920 and 0<=joint_points[11].y<1080):
                            pygame.draw.circle(self._frame_surface, color_1,\
                                [int(joint_points[11].x),int(joint_points[11].y)],50,15)
                        
                        # draw bones of body
                        self.draw_body(joints, joint_points, SKELETON_COLORS[i])               
                    
                    # ---lock interface and unlock procedure----
                    if(start_system_flag == True and lock_system_flag == True):
                        # draw the lock pattern (consists of 9 filled circles) when enter the lock interface
                        for ci in xrange(0,9):
                            pygame.draw.circle(self._frame_surface, (255,0,255),lock_circle_list[ci],50,0)
                        
                        # detect push gesture
                        push_start_flag, push_flag, push_delay_count = self.pushGestureDetection(\
                                    body,push_start_flag,palm_posi_record,push_delay_count)
                        
                        if(push_flag):
                            for ci in xrange(0,9):
                                if((lock_circle_list[ci][0]- joint_points[11].x)**2+(lock_circle_list[ci][1]- joint_points[11].y)**2<3600):
                                    if(len(chosen_circle_no_list) == 0): # ignore rest part of long distance push
                                        chosen_circle_no_list.append(ci)
                        
                        # the draw gesture recognition, if detected, erase chose circle info
                        draw_flag,hand_record_x,gesture_miss_time_count = self.drawGestureDetection( \
                                body,joint_points,hand_record_x,gesture_miss_time_count,draw_flag)
                        if (draw_flag):
                            chosen_circle_no_list = []
                            # reset all the data related to draw gesture
                            draw_flag=False
                            hand_record_x = -1
                            gesture_miss_time_count=0   
                        
                        # color the chosen circle
                        for ci in chosen_circle_no_list:
                            pygame.draw.circle(self._frame_surface, (0,255,255),lock_circle_list[ci],50,0)
                        
                        
                        # draw lines for pattern
                        if(len(chosen_circle_no_list)>0):
                            
                            # add new point of circle when hands pass throgh it
                            for ci in xrange(0,9):
                                if((lock_circle_list[ci][0]- joint_points[11].x)**2+(lock_circle_list[ci][1]- joint_points[11].y)**2<3600):
                                    if(ci != chosen_circle_no_list[-1]):
                                        chosen_circle_no_list.append(ci)
                                    
                            # draw lines between last chosen circle and hand
                            pygame.draw.line(self._frame_surface, (0,255,255),lock_circle_list[chosen_circle_no_list[-1]],(joint_points[11].x,joint_points[11].y),40)
                            
                            # draw lines between the chosen circles
                            if(len(chosen_circle_no_list)>1):
                                for ki in xrange(0,len(chosen_circle_no_list)-1):
                                    pygame.draw.line(self._frame_surface, (0,255,255),lock_circle_list[chosen_circle_no_list[ki]],lock_circle_list[chosen_circle_no_list[ki+1]],40)
                            
                            # compare the pattern you draw and the pattern needed for unlock
                            if(len(chosen_circle_no_list) >= len(unlock_circle_pattern)):
                                temp_sum = 0
                                for pati in xrange(0,len(unlock_circle_pattern)):
                                    temp_sum += abs(unlock_circle_pattern[pati]-chosen_circle_no_list[pati])
                                if( temp_sum == 0 ):
                                    lock_system_flag = False
                                    chosen_circle_no_list = []
                                    video_section_flag = True
                                    print 'unlocked!'
                                    self._text_timer[3] = 30 # for system text to show for 30 frames, use push timer
                                    
            
            h_to_w = float(self._frame_surface.get_height()) / self._frame_surface.get_width()
            target_height = int(h_to_w * self._screen.get_width())
            
                
            surface_to_draw = pygame.transform.scale(self._frame_surface, (self._screen.get_width(), target_height));
            
            self._screen.blit(surface_to_draw, (0,0))
            surface_to_draw = None
            pygame.display.update()
                       
            
            # --- Limit to 30 frames per second
            self._clock.tick(fps)
#            self.typetext('Fps: %d' % int(1/(time.time()-start_time)),(1050,900))
        # -------- Main Program Loop End-----------
        self._kinect.close()
        pygame.quit()
        


__main__ = "Kinect v2 Body Game"

game = BodyGameRuntime();

game.run();
