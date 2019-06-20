from PIL import Image
from PIL import ImageTk
import tkinter as tki
from tkinter import Toplevel, Scale
import numpy as np
import threading
import datetime
import cv2
import os
import time
import platform


# image from tello is [720,960,3]
# 720 is vertical piexl
# 960 is horizontal piexl
# x means horizontal
# y means vertical

class TelloUI:
    """Wrapper class to enable the GUI."""

    def __init__(self, tello, outputpath):
        """
        Initial all the element of the GUI,support by Tkinter

        :param tello: class interacts with the Tello drone.

        Raises:
            RuntimeError: If the Tello rejects the attempt to enter command mode.
        """

        self.tello = tello  # videostream device
        self.outputPath = outputpath  # the path that save pictures created by clicking the takeSnapshot button
        self.frame = None  # frame read from h264decoder and used for pose recognition 
        self.thread = None  # thread of the Tkinter mainloop
        self.stopEvent = None

        # control variables
        self.distance = 0.1  # default distance for 'move' cmd
        self.degree = 30  # default degree for 'cw' or 'ccw' cmd
        self.target_angle = 0 #target angle when rotate
        # state variables
        self.state = {'take_off': 0, 'fly_up': 1, 'find_ball': 2,
                      'roll_to_ball': 3, 'fly_straight': 4, 'fly_finish': 5}
        self.state_value = 0
        self.state_to_call = [self.take_off, self.fly_up, self.find_ball,
                              self.roll_to_ball, self.fly_straight, self.fly_finish]
        self.take_off_time = 0
        self.is_takeoff = False
        self.is_cmd_send = False
        self.show_image = True


        self.ball_x = -1
        self.ball_y = -1
        self.distance = -1
        # img record
        self.is_img_record = False
        self.img_num = 0
        # image variables
        self.middle_x = 960 / 2
        self.middle_y = 720 / 2

        # if the flag is TRUE,the auto-takeoff thread will stop waiting for the response from tello
        self.quit_waiting_flag = False

        # initialize the root window and image panel
        self.root = tki.Tk()
        self.panel = None

        # create buttons
        self.btn_snapshot = tki.Button(self.root, text="Snapshot!",
                                       command=self.takeSnapshot)
        self.btn_snapshot.pack(side="bottom", fill="both",
                               expand="yes", padx=10, pady=5)

        self.btn_pause = tki.Button(self.root, text="Pause", relief="raised", command=self.pauseVideo)
        self.btn_pause.pack(side="bottom", fill="both",
                            expand="yes", padx=10, pady=5)

        self.btn_landing = tki.Button(
            self.root, text="Open Command Panel", relief="raised", command=self.openCmdWindow)
        self.btn_landing.pack(side="bottom", fill="both",
                              expand="yes", padx=10, pady=5)
        self.is_program_end = False
        # start a thread that constantly pools the video sensor for
        # the most recently read frame
        self.stopEvent = threading.Event()
        self.thread = threading.Thread(target=self.videoLoop, args=())
        self.thread.start()



        # set a key event
        self.root.bind('<Key>', self.onKey)
        # set a callback to handle when the window is closed
        self.root.wm_title("TELLO Controller")
        self.root.wm_protocol("WM_DELETE_WINDOW", self.onClose)

        # the sending_command will send command to tello every 5 seconds
        self.sending_command_thread = threading.Thread(target=self._sendingCommand)

    def videoLoop(self):
        """
        The mainloop thread of Tkinter
        Raises:
            RuntimeError: To get around a RunTime error that Tkinter throws due to threading.
        """
        try:
            # start the thread that get GUI image and drwa skeleton
            time.sleep(0.5)
            self.sending_command_thread.start()
            while (not self.stopEvent.is_set()) and (not self.is_program_end):
                system = platform.system()
               # print(self.is_program_end)
                # read the frame for GUI show
                self.frame = self.tello.get_frame_read()
                if self.frame is None or self.frame.size == 0:
                    continue

                    # transfer the format from frame to image
                image = Image.fromarray(self.frame)
                self.commandProcess(image)
                # we found compatibility problem between Tkinter,PIL and Macos,and it will
                # sometimes result the very long preriod of the "ImageTk.PhotoImage" function,
                # so for Macos,we start a new thread to execute the _updateGUIImage function.
                if system == "Windows" or system == "Linux":
                    self._updateGUIImage(image)

                else:
                    thread_tmp = threading.Thread(target=self._updateGUIImage, args=(image,))
                    thread_tmp.start()
                    time.sleep(0.03)
        except RuntimeError:
            print("[INFO] caught a RuntimeError")

    def _updateGUIImage(self, image):
        """
        Main operation to initial the object of image,and update the GUI panel
        """
        image = ImageTk.PhotoImage(image)
        # if the panel none ,we need to initial it
        if self.panel is None:
            self.panel = tki.Label(image=image)
            self.panel.image = image
            self.panel.pack(side="left", padx=10, pady=10)
        # otherwise, simply update the panel
        else:
            self.panel.configure(image=image)
            self.panel.image = image

    def _sendingCommand(self):
        """
        start a while loop that sends 'command' to tello every 5 second
        """
        while not self.is_program_end:
            self.tello.send_command_with_return('command')
            bat = self.tello.get_battery()
            if bat <= 10:
                print('low power')
                self.is_program_end = True
                break
            time.sleep(5)

        self.onClose()

    def _setQuitWaitingFlag(self):
        """
        set the variable as TRUE,it will stop computer waiting for response from tello
        """
        self.quit_waiting_flag = True

    def openCmdWindow(self):
        """
        open the cmd window and initial all the button and text
        """
        panel = Toplevel(self.root)
        panel.wm_title("Command Panel")

        # create text input entry
        text0 = tki.Label(panel,
                          text='This Controller map keyboard inputs to Tello control commands\n'
                               'Adjust the trackbar to reset distance and degree parameter',
                          font='Helvetica 10 bold'
                          )
        text0.pack(side='top')

        text1 = tki.Label(panel, text=
        'W - Move Tello Up\t\t\tArrow Up - Move Tello Forward\n'
        'S - Move Tello Down\t\t\tArrow Down - Move Tello Backward\n'
        'A - Rotate Tello Counter-Clockwise\tArrow Left - Move Tello Left\n'
        'D - Rotate Tello Clockwise\t\tArrow Right - Move Tello Right',
                          justify="left")
        text1.pack(side="top")

        self.btn_landing = tki.Button(
            panel, text="Land", relief="raised", command=self.telloLanding)
        self.btn_landing.pack(side="bottom", fill="both",
                              expand="yes", padx=10, pady=5)

        self.btn_takeoff = tki.Button(
            panel, text="Takeoff", relief="raised", command=self.telloTakeOff)
        self.btn_takeoff.pack(side="bottom", fill="both",
                              expand="yes", padx=10, pady=5)

        # binding arrow keys to drone control
        self.tmp_f = tki.Frame(panel, width=100, height=2)
        self.tmp_f.bind('<KeyPress-w>', self.on_keypress_w)
        self.tmp_f.bind('<KeyPress-s>', self.on_keypress_s)
        self.tmp_f.bind('<KeyPress-a>', self.on_keypress_a)
        self.tmp_f.bind('<KeyPress-d>', self.on_keypress_d)
        self.tmp_f.bind('<KeyPress-Up>', self.on_keypress_up)
        self.tmp_f.bind('<KeyPress-Down>', self.on_keypress_down)
        self.tmp_f.bind('<KeyPress-Left>', self.on_keypress_left)
        self.tmp_f.bind('<KeyPress-Right>', self.on_keypress_right)
        self.tmp_f.pack(side="bottom")
        self.tmp_f.focus_set()

        self.btn_landing = tki.Button(
            panel, text="Flip", relief="raised", command=self.openFlipWindow)
        self.btn_landing.pack(side="bottom", fill="both",
                              expand="yes", padx=10, pady=5)

        self.distance_bar = Scale(panel, from_=0.02, to=5, tickinterval=0.01, digits=3, label='Distance(m)',
                                  resolution=0.01)
        self.distance_bar.set(0.2)
        self.distance_bar.pack(side="left")

        self.btn_distance = tki.Button(panel, text="Reset Distance", relief="raised",
                                       command=self.updateDistancebar,
                                       )
        self.btn_distance.pack(side="left", fill="both",
                               expand="yes", padx=10, pady=5)

        self.degree_bar = Scale(panel, from_=1, to=360, tickinterval=10, label='Degree')
        self.degree_bar.set(30)
        self.degree_bar.pack(side="right")

        self.btn_distance = tki.Button(panel, text="Reset Degree", relief="raised", command=self.updateDegreebar)
        self.btn_distance.pack(side="right", fill="both",
                               expand="yes", padx=10, pady=5)

    def openFlipWindow(self):
        """
        open the flip window and initial all the button and text
        """

        panel = Toplevel(self.root)
        panel.wm_title("Gesture Recognition")

        self.btn_flipl = tki.Button(
            panel, text="Flip Left", relief="raised", command=self.telloFlip_l)
        self.btn_flipl.pack(side="bottom", fill="both",
                            expand="yes", padx=10, pady=5)

        self.btn_flipr = tki.Button(
            panel, text="Flip Right", relief="raised", command=self.telloFlip_r)
        self.btn_flipr.pack(side="bottom", fill="both",
                            expand="yes", padx=10, pady=5)

        self.btn_flipf = tki.Button(
            panel, text="Flip Forward", relief="raised", command=self.telloFlip_f)
        self.btn_flipf.pack(side="bottom", fill="both",
                            expand="yes", padx=10, pady=5)

        self.btn_flipb = tki.Button(
            panel, text="Flip Backward", relief="raised", command=self.telloFlip_b)
        self.btn_flipb.pack(side="bottom", fill="both",
                            expand="yes", padx=10, pady=5)

    def takeSnapshot(self):
        """
        save the current frame of the video as a jpg file and put it into outputpath
        """

        # grab the current timestamp and use it to construct the filename
        ts = datetime.datetime.now()
        filename = "{}.jpg".format(ts.strftime("%Y-%m-%d_%H-%M-%S"))

        p = os.path.sep.join((self.outputPath, filename))

        # save the file
        cv2.imwrite(p, cv2.cvtColor(self.frame, cv2.COLOR_RGB2BGR))
        print("[INFO] saved {}".format(filename))

    def pauseVideo(self):
        """
        Toggle the freeze/unfreze of video
        """
        if self.btn_pause.config('relief')[-1] == 'sunken':
            self.btn_pause.config(relief="raised")
            self.tello.video_freeze(False)
        else:
            self.btn_pause.config(relief="sunken")
            self.tello.video_freeze(True)

    def telloTakeOff(self):
        return self.tello.takeoff()

    def telloLanding(self):
        return self.tello.land()

    def telloFlip_l(self):
        return self.tello.flip('l')

    def telloFlip_r(self):
        return self.tello.flip('r')

    def telloFlip_f(self):
        return self.tello.flip('f')

    def telloFlip_b(self):
        return self.tello.flip('b')

    def telloCW(self, degree):
        return self.tello.rotate_cw(degree)

    def telloCCW(self, degree):
        return self.tello.rotate_ccw(degree)

    def telloMoveForward(self, distance):
        return self.tello.move_forward(distance)

    def telloMoveBackward(self, distance):
        return self.tello.move_backward(distance)

    def telloMoveLeft(self, distance):
        return self.tello.move_left(distance)

    def telloMoveRight(self, distance):
        return self.tello.move_right(distance)

    def telloUp(self, dist):
        return self.tello.move_up(dist)

    def telloDown(self, dist):
        return self.tello.move_down(dist)

    def updateTrackBar(self):
        self.my_tello_hand.setThr(self.hand_thr_bar.get())

    def updateDistancebar(self):
        self.distance = self.distance_bar.get()
        print('reset distance to %.1f' % self.distance)

    def updateDegreebar(self):
        self.degree = self.degree_bar.get()
        print('reset distance to %d' % self.degree)

    def on_keypress_w(self, event):
        print("up %d m" % self.distance)
        self.telloUp(self.distance)

    def on_keypress_s(self, event):
        print("down %d m" % self.distance)
        self.telloDown(self.distance)

    def on_keypress_a(self, event):
        print("ccw %d degree" % self.degree)
        self.tello.rotate_ccw(self.degree)

    def on_keypress_d(self, event):
        print("cw %d degree" % self.degree)
        self.tello.rotate_cw(self.degree)

    def on_keypress_up(self, event):
        print( "forward %d m" % self.distance)
        self.telloMoveForward(self.distance)

    def on_keypress_down(self, event):
        print("backward %d m" % self.distance)
        self.telloMoveBackward(self.distance)

    def on_keypress_left(self, event):
        print("left %d m" % self.distance)
        self.telloMoveLeft(self.distance)

    def on_keypress_right(self, event):
        print("right %d m" % self.distance)
        self.telloMoveRight(self.distance)

    def on_keypress_enter(self, event):
        if self.frame is not None:
            self.registerFace()
        self.tmp_f.focus_set()

    def onClose(self):
        """
        set the stop event, cleanup the camera, and allow the rest of

        the quit process to continue
        """
        print("[INFO] closing...")
        self.tello.land()
        self.stopEvent.set()
        del self.tello
        self.root.quit()

    def call_back(self):
        self.state_to_call[self.state_value]()

    def record_image(self, image):
        if self.is_img_record:
            img_name = str(self.img_num)
            img_name = img_name.zfill(6) + '.jpg'
            p = os.path.sep.join((self.outputPath, img_name))
            self.img_num += 1
            # save the file
            cv2.imwrite(p, cv2.cvtColor(self.frame, cv2.COLOR_RGB2BGR))

    def commandProcess(self, image):
        '''
        run for judge image and send command
        :param image: image from tello in h264
        :return: none
        '''
        #check if has enough power

        self.tello.prase_state()
        # process image to find the ball position in pielx
        # and real distance in meter
        self.ball_x, self.ball_y, self.distance = self.imageProcess(image)
        #record image
        self.record_image(image)

        ##state machine of action
        self.call_back()

    def imageProcess(self, image):
        lmin = 0#30;
        lmax = 162#100;

        amin = 150#150;
        amax = 255#200;

        bmin = 118#130;
        bmax = 255#170;

        redLower = np.array([lmin, amin, bmin])
        redUpper = np.array([lmax, amax, bmax])

        img = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)
        # print ('image x = %s, y = %s' %(img.shape[0],img.shape[1]))
        frame_lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        red = cv2.inRange(frame_lab, redLower, redUpper)
        red = cv2.erode(red, None, iterations=8)
        # red = cv2.dilate(red, None, iterations=10)

        ball_x, ball_y, distance = self.findBall(red, frame_lab)
        if self.show_image:
            cv2.imshow("OpenCV", img)
            cv2.imshow('red in range', red)
            cv2.waitKey(30)
        return ball_x, ball_y, distance

    def findBall(self, red, img):

        radius2distance = 67.5#59.4

        contours_ball, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        res = np.zeros_like(img)
        ball_x = ball_y = radius = dist_real = -1

        if len(contours_ball) > 0:
            contours_ball.sort(key=lambda x: len(x))
            cnt = contours_ball[-1]
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            if y > 100:
                ball_x, ball_y = x, y
                last_ball_x = x
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(res, center, radius, (0, 0, 255), -1)
            if radius > 0:
                dist_real = radius2distance / (1.0 * radius)
            else:
                dist_real = radius
        if self.show_image:
            cv2.imshow("c", res)

            print("ball_x=%s \n ball_y = %s \n ball_rp=%s \n real_r = %s" % (ball_x, ball_y, radius, dist_real))

        return ball_x, ball_y, dist_real

    def take_off(self):
        # self.tello.land()
        if (self.is_takeoff == False):
            self.tello.takeoff()
            self.take_off_time = time.time()
            self.is_takeoff = True
        elif (self.tello.get_response() == 'ok') and (time.time() - self.take_off_time >= 2):
            self.state_value = self.state['fly_up']
            # self.tello.land()
            print('transfer 2 fly up')
        print('take off')

    def fly_up(self):
        #0.85m
        target_height = 0.85
        height = self.tello.get_tof()
        delt_height = target_height - height
        print(height)

            #self.state_value = self.state['find_ball']


        if self.is_cmd_send == False:
            self.tello.move_up(target_height)
            self.is_cmd_send = True
        if (self.tello.get_response() == 'ok')and(self.is_cmd_send == True):
            self.is_cmd_send = False
        if delt_height <= 0.03:
            #self.tello.land()
            self.state_value = self.state['find_ball']
        print('fly_up')

    def find_ball(self):
        #print(self.tello.get_yaw())
        if self.ball_x >0 and self.ball_y > 0 and self.distance > 0:
            self.state_value = self.state['roll_to_ball']

        if self.is_cmd_send == False:
            #self.tello.land()
            self.tello.rotate_ccw(45)
            self.is_cmd_send = True

        print('find_ball')

    def roll_to_ball(self):
       # print(self.tello.get_yaw())
        delt_roll = self.middle_x - self.ball_x
        print ('delt_roll %s'%delt_roll)

        if abs(delt_roll) <= 10:
            self.tello.stop()
            self.target_angle = self.tello.get_yaw()
            self.is_cmd_send = False
            self.state_value = self.state['fly_straight']
        elif self.is_cmd_send == False:
            #self.tello.land()
            if(delt_roll>0):
                self.tello.rotate_cw(10)
            else:
                self.tello.rotate_ccw(10)
            self.is_cmd_send = True
        elif (self.tello.get_response() == 'ok')and(self.is_cmd_send == True):
            self.is_cmd_send = False

        print('roll_to_ball')

    def fly_straight(self):
        if abs(delt_roll) > 20:
            self.state_value = self.state['roll_to_ball']
            self.is_cmd_send = False
        elif self.distance <=0.5:
            self.state_value = self.state['fly_finish']
            self.is_cmd_send = False

        elif self.is_cmd_send == False :
            fly_dist = self.distance
            if fly_dist >= 5:
                fly_dist = 5
            #if(self.distance <= 1.5)
            self.tello.move_forward(fly_dist)

            self.is_cmd_send = True
        elif (self.tello.get_response() == 'ok')and(self.is_cmd_send == True):
            self.is_cmd_send = False
        print('fly_stright')

    def fly_finish(self):
        if self.is_cmd_send == False:
            self.tello.land()
            self.is_cmd_send = True
        print('fly_finish')

        # self.state = {'take_off': 0, 'fly_up': 1, 'find_ball': 2,
        #             'roll_to_ball': 3, 'fly_straight': 4, 'fly_finish': 5}

    def onKey(self, event):
        # press q to stop the programme
        if event.char == 'q':
            self.onClose()
            self.is_program_end = True
        if event.char == 's':
            self.is_img_record = True
            print('record img start')
        if event.char == 'p':
            print('record img pause')
            self.is_img_record = False