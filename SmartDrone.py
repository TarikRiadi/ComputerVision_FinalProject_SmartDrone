from djitellopy import Tello
import cv2
import pygame
from pygame.locals import *
import numpy as np
import time
import os
from tkinter import filedialog

# Speed of the drone
S = 60
# Frames per second of the pygame window display
FPS = 30 #Limit to 10fps for final test.

font = cv2.FONT_HERSHEY_COMPLEX #Font to write in image.

#Define color ranges in HSV
lower =  {'red':(0,170,70),   'red2':(160,130,50),  'green':(45,86,37),   'blue':(101,50,40),   'yellow':(16,70,100)} #'blue':(101,50,40)
upper =  {'red':(10,255,255), 'red2':(180,255,255), 'green':(70,250,194), 'blue':(129,255,255), 'yellow':(30,255,255)} # 'blue':(129,255,255)
#Define color for text in BGR
colors = {'red':(0,0,255),    'red2':(0,0,255),     'green':(0,255,0),    'blue':(255,0,0),     'yellow':(0,255,255)}


class TelloDrone(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations
            - W and S: Up and down.
    """

    def __init__(self):
        # Init pygame
        pygame.init()

        # Creat pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()

        # Drone States
        self.battery = 100
        self.altitude = 0

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10

        # VA Figures Detect Sates
        self.find = False
        self.circle = False
        self.triangle = False
        self.rectangle = False
        self.first_instruction = False

        # Buff Figures detected
        self.figures_detected = list()
        self.last_figure = ["","",""]
        self.last_area = None
        self.last_time = 0
        self.flight_time = 0

        # radio control state
        self.send_rc_control = False #Control Manual.

        # drone control
        self.vision_control = False
        self.control_view = False

        # create update timer
        pygame.time.set_timer(USEREVENT + 1, 50)

        # DIR images save
        #self.DIR = filedialog.askdirectory() #Asks for images' directory.
        #print(self.DIR)

        self.id_image = 0 # id counter image

        self.frame = None # last_frame

    def run(self):

        if not self.tello.connect():
            print("Tello not connected")
            return

        if not self.tello.set_speed(self.speed):
            print("Not set speed to lowest possible")
            return

        # In case streaming is on. This happens when we quit this program without the escape key.
        if not self.tello.streamoff():
            print("Could not stop video stream")
            return

        if not self.tello.streamon():
            print("Could not start video stream")
            return

        frame_read = self.tello.get_frame_read()

        should_stop = False
        while not should_stop:

            for event in pygame.event.get():
                if event.type == USEREVENT + 1:
                    if self.vision_control == False:
                        self.update()
                elif event.type == QUIT:
                    should_stop = True
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        should_stop = True
                        print(self.tello.get_battery())
                    else:
                        if self.vision_control == False:
                            self.keydown(event.key)
                        else:
                            pass
                elif event.type == KEYUP:
                    if self.vision_control == False:
                        self.keyup(event.key)
                    else:
                        self.keyup_vision_control(event.key)
                else:
                    pass


            if frame_read.stopped:
                frame_read.stop()
                break

            self.screen.fill([0, 0, 0])

            self.frame = frame_read.frame
            self.searching()

            if self.vision_control == True: #Que se maneje según lo que ve.
                if self.control_view == True: #Que reemplace (si True) last figure.
                    self.mostFrequency()
                self.figureAction()

            self.update()
            self.display_frame()
            #self.save_images(self.frame) #saving image
        # Call it always before finishing. I deallocate resources.
        self.tello.end()

    def display_frame(self):
        frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
        frame = np.rot90(frame)
        frame = np.flipud(frame)
        frame = pygame.surfarray.make_surface(frame)
        self.screen.blit(frame, (0, 0))
        pygame.display.update()
        time.sleep(1 / FPS)


    def figureAction(self):
        if self.last_figure[0:2] == ["Triangle","green"]:
            print(self.last_figure)

        elif self.last_figure[0:2] == ["Triangle","blue"]:
            print(self.last_figure)
            #time.sleep(1)
        elif self.last_figure[0:2] == ["Triangle","red"] or self.last_figure[0:2] == ["Triangle","red2"]:
            print(self.last_figure)
            tm = self.tello.get_flight_time()
            tm = self.tello.get_flight_time()
            print(tm)
            buff = tm[:-3]
            if buff.isnumeric():
                self.flight_time = int(buff)
            if self.control_view == True:
                print("First",self.flight_time)
                self.last_time = self.flight_time
                self.control_view = False
                self.tello.set_speed(10)
                time.sleep(0.5)
                self.tello.flip_back()
            elif (self.flight_time > self.last_time + 3) and self.first_instruction == False:
                print("Second",self.flight_time)
                self.last_time = self.flight_time
                self.first_instruction = True
                time.sleep(0.5)
                self.tello.move_left(15)
            elif self.flight_time > self.last_time + 3:
                print("Third",self.flight_time)
                self.last_time = self.flight_time
                self.control_view = True
                self.first_instruction = False
                time.sleep(0.5)
                self.tello.land()

        elif self.last_figure[0:2] == ["Square","yellow"]:
            print(self.last_figure)
        elif self.last_figure[0:2] == ["Circle","green"]:
            print(self.last_figure)
        elif self.last_figure[0:2] == ["Circle","blue"]:
            print(self.last_figure)
            tm = self.tello.get_flight_time()
            tm = self.tello.get_flight_time()
            print(tm)
            buff = tm[:-3]
            if buff.isnumeric():
                self.flight_time = int(buff)
            if self.control_view == True:
                print("First",self.flight_time)
                self.last_time = self.flight_time
                self.control_view = False
                self.tello.set_speed(10)
                time.sleep(0.5)
                #self.tello.rotate_clockwise(180)
                self.tello.rotate_counter_clockwise(90)
            elif (self.flight_time > self.last_time + 3) and self.first_instruction == False:
                print("Second",self.flight_time)
                self.last_time = self.flight_time
                self.first_instruction = True
                time.sleep(0.5)
                #self.tello.move_forward(30)
                self.tello.move_left(47)
            elif self.flight_time > self.last_time + 3:
                print("Third",self.flight_time)
                self.last_time = self.flight_time
                self.control_view = True
                self.first_instruction = False
                self.tello.set_speed(50)
                time.sleep(0.5)
                self.tello.move_forward(212)
                #self.tello.land()
        #elif self.last_figure[0:2] == ["Triangle","red2"]:
        #    print(self.last_figure)
        elif self.last_figure[0:2] == ["Triangle","yellow"]:
            print(self.last_figure)
            #time.sleep(1)
        elif self.last_figure[0:2] == ["Square","green"]:
            print(self.last_figure)
            tm = self.tello.get_flight_time()
            tm = self.tello.get_flight_time()
            print(tm)
            buff = tm[:-3]
            if buff.isnumeric():
                self.flight_time = int(buff)
            if self.control_view == True:
                print("First",self.flight_time)
                self.last_time = self.flight_time
                self.control_view = False
                time.sleep(0.5)
                self.tello.move_back(20)
            elif (self.flight_time > self.last_time + 3) and self.first_instruction == False:
                print("Second",self.flight_time)
                self.last_time = self.flight_time
                self.first_instruction = True
                time.sleep(0.5)
                #self.tello.move_forward(45)
                self.tello.rotate_counter_clockwise(90)
            elif self.flight_time > self.last_time + 3:
                print("Third",self.flight_time)
                self.last_time = self.flight_time
                self.control_view = True
                self.first_instruction = False
                self.tello.set_speed(10)
                time.sleep(0.5)
                self.tello.move_forward(20)
        elif self.last_figure[0:2] == ["Square","blue"]:
            print(self.last_figure)
        elif self.last_figure[0:2] == ["Square","red"] or self.last_figure[0:2] == ["Square","red2"]:
            print(self.last_figure)

        elif self.last_figure[0:2] == ["Circle","red"]:
            print(self.last_figure)
        elif self.last_figure[0:2] == ["Circle","red2"]:
            print(self.last_figure)
        elif self.last_figure[0:2] == ["Circle","yellow"]:
            print(self.last_figure)
            tm = self.tello.get_flight_time()
            tm = self.tello.get_flight_time()
            print(tm)
            buff = tm[:-3]
            if buff.isnumeric():
                self.flight_time = int(buff)

            if self.control_view == True:
                print("First",self.flight_time)
                self.last_time = self.flight_time
                self.control_view = False
                self.tello.set_speed(50)
                time.sleep(0.5)
                #self.tello.rotate_clockwise(180)
                self.tello.move_left(212) #Good
            elif (self.flight_time > self.last_time + 3) and self.first_instruction == False:
                print("Second",self.flight_time)
                self.last_time = self.flight_time
                self.first_instruction = True
                self.tello.set_speed(10)
                time.sleep(0.5)
                #self.tello.move_left(210)
                #self.tello.rotate_clockwise(180) #Good
                #self.tello.move_forward(10)
            elif self.flight_time > self.last_time + 3:
                print("Third",self.flight_time)
                self.last_time = self.flight_time
                self.control_view = True
                self.first_instruction = False
                self.tello.set_speed(10)
                time.sleep(0.5)
                #self.tello.land()
                #self.tello.move_left(210)
        else:
            print("Nothing!!!")


    def searching(self):
        length = 10 # buff length
        if len(self.figures_detected) < length:
            self.figures_detected.append(self.colorFigures())
        else:
            self.figures_detected.pop(0)
            self.figures_detected.append(self.colorFigures())

    def colorFigures(self):
        figures = list()
        frame = cv2.GaussianBlur(self.frame,(15,15),0)
        #frame = cv2.convertScaleAbs(self.frame,alpha=0,beta=10) #change contrast and brightness.
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) #Transform colorspace from BGR to HSV.

        #Threshold the original image
        for key, value in upper.items():
            mask = self.filter(key, hsv)
            figures = self.circleDetect(mask, key, figures) #Circle detection.
            contours = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
            for cnt in contours:
                figure = self.figureDetect(cnt, key)
                if figure[0] == "Triangle":
                    figures.append(["Triangle", key, figure[1]])
                elif figure[0] == "Square":
                    figures.append(["Square", key, figure[1]])
                else:
                    pass

        return figures

    def circleDetect(self, mask, key, figures):
        rows = mask.shape[0]
        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, rows,
                                   param1=200, param2=16, #200,15
                                   minRadius=50, maxRadius=100) #50,80
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                radius = i[2]
                area = 2*np.pi*np.square(radius)
                if 30000 < area and area < 60000:
                    # circle center
                    cv2.circle(self.frame, center, 1, colors[key], 3) #Draw the center.
                    # circle outline
                    cv2.circle(self.frame, center, radius, colors[key], 2) #Draw the circle.
                    cv2.putText(self.frame, "Circle " + key, center, font, 0.7, colors[key], 2)
                    figures.append(["Circle", key, area])
        return figures

    def filter(self, key, hsv):
        kernel = np.ones((3,3), np.uint8) #Kernel of 1s in uint8.

        mask = cv2.inRange(hsv,lower[key],upper[key]) #Find objects that match the color range defined.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask

    def figureDetect(self, frame, key):
        figure = ["", None]
        area = cv2.contourArea(frame)
        approx = cv2.approxPolyDP(frame, 0.03*cv2.arcLength(frame, True), True)
        x = approx.ravel()[0] #Determine horizontal position.
        y = approx.ravel()[1] #Determine vertical postition.
        if 5000 < area and area < 25000: #Only detects objects larger than #Probemos con [6,000 - 20,000]?
            #print('Area: ', area)
            #Draw the contour on top of the frame and write the shape detected.
            if len(approx) == 3:
                figure = ["Triangle", area]
                cv2.drawContours(self.frame, [approx], 0, colors[key], 2)
                cv2.putText(self.frame, "Triangle", (x,y), font, 0.7, colors[key], 2)
            elif len(approx) == 4:
                #cv2.drawContours(self.frame, [approx], 0, colors[key], 2)
                (x,y,w,h) = cv2.boundingRect(approx)
                ar = w/float(h)
                if ar >= 0.95 and ar <= 1.05:
                    cv2.putText(self.frame, "Square", (x,y), font, 0.7, colors[key], 2)
                    cv2.drawContours(self.frame, [approx], 0, colors[key], 2)
                    figure = ["Square", area]
        return figure


    def mostFrequency(self):
        List = list()
        for row in self.figures_detected:
            counter = 0
            num = []
            for figure in row:
                curr_frequency = row.count(figure)
                if (curr_frequency > counter):
                    counter = curr_frequency
                    num = figure
            List.append(num)
        counter = 0
        fig = []
        for figure in List:
            curr_frequency = List.count(figure)
            if (curr_frequency > counter):
                counter = curr_frequency
                fig = figure
        self.last_figure = fig
        return fig

    def keyup_vision_control(self, key):
        if key == pygame.K_t:  # takeoff
            self.tello.takeoff()
            self.send_rc_control = True
        elif key == pygame.K_l:  # land
            self.tello.land()
            self.send_rc_control = False
        elif key == pygame.K_c:
            if self.vision_control == False:
                self.vision_control = True
                self.control_view = True
                self.send_rc_control = False
            else:
                self.vision_control = False
                self.control_view = False
                self.send_rc_control = True


    def keydown(self, key):
        """ Update velocities based on key pressed
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP:  # set forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_DOWN:  # set backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_LEFT:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # set right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # set up velocity
            self.up_down_velocity = S
        elif key == pygame.K_s:  # set down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_a:  # set yaw clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # set yaw counter clockwise velocity
            self.yaw_velocity = S


    def keyup(self, key):
        """ Update velocities based on key released
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_t:  # takeoff
            self.tello.takeoff()
            self.send_rc_control = True
        elif key == pygame.K_l:  # land
            self.tello.land()
            self.send_rc_control = False
        elif key == pygame.K_c:
            if self.vision_control == False:
                self.vision_control = True
                self.control_view = True
                self.send_rc_control = False
            else:
                self.vision_control = False
                self.control_view = False
                self.send_rc_control = True

    def update(self):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,
                                       self.yaw_velocity)


    def save_images(self, frame):
        dir = self.DIR + '/image_' + str(self.id_image).zfill(5) + '.jpg'
        #print(dir)
        cv2.imwrite(dir,frame)
        self.id_image += 1

def main():
    drone = TelloDrone()

    # run drone
    drone.run()


if __name__ == '__main__':
    main()
