import cv2
import gym
import math
import rospy
import roslaunch
import time
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from gym import utils, spaces
from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import Image
from time import sleep

from gym.utils import seeding

import os

# class Gazebo_Linefollow_Env(gazebo_env.GazeboEnv):

#     def __init__(self):
#         # Launch the simulation with the given launchfile name
#         LAUNCH_FILE = '/home/fizzer/enph353_gym-gazebo-noetic/gym_gazebo/envs/ros_ws/src/linefollow_ros/launch/linefollow_world.launch'
#         gazebo_env.GazeboEnv.__init__(self, LAUNCH_FILE)
#         self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#         self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
#         self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
#         self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world',
#                                               Empty)

#         self.action_space = spaces.Discrete(3)  # F,L,R
#         self.reward_range = (-np.inf, np.inf)
#         self.episode_history = []

#         self._seed()

#         self.bridge = CvBridge()
#         self.timeout = 0  # Used to keep track of images with no line detected


#     def process_image(self, data):
#         '''
#             @brief Coverts data into a opencv image and displays it
#             @param data : Image data from ROS

#             @retval (state, done)
#         '''
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             print(e)

#         # cv2.imshow("raw", cv_image)

#         NUM_BINS = 3
#         state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
#         done = False

#         # TODO: Analyze the cv_image and compute the state array and
#         # episode termination condition.
#         #
#         # The state array is a list of 10 elements indicating where in the
#         # image the line is:
#         # i.e.
#         #    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0] indicates line is on the left
#         #    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0] indicates line is in the center
#         #
#         # The episode termination condition should be triggered when the line
#         # is not detected for more than 30 frames. In this case set the done
#         # variable to True.
#         #
#         # You can use the self.timeout variable to keep track of which frames
#         # have no line detected.

#         return state, done

#     def _seed(self, seed=None):
#         self.np_random, seed = seeding.np_random(seed)
#         return [seed]

#     def step(self, action):
#         rospy.wait_for_service('/gazebo/unpause_physics')
#         try:
#             self.unpause()
#         except (rospy.ServiceException) as e:
#             print ("/gazebo/unpause_physics service call failed")

#         self.episode_history.append(action)

#         vel_cmd = Twist()

#         if action == 0:  # FORWARD
#             vel_cmd.linear.x = 0.4
#             vel_cmd.angular.z = 0.0
#         elif action == 1:  # LEFT
#             vel_cmd.linear.x = 0.0
#             vel_cmd.angular.z = 0.5
#         elif action == 2:  # RIGHT
#             vel_cmd.linear.x = 0.0
#             vel_cmd.angular.z = -0.5

#         self.vel_pub.publish(vel_cmd)

#         data = None
#         while data is None:
#             try:
#                 data = rospy.wait_for_message('/pi_camera/image_raw', Image,
#                                               timeout=5)
#             except:
#                 pass

#         rospy.wait_for_service('/gazebo/pause_physics')
#         try:
#             # resp_pause = pause.call()
#             self.pause()
#         except (rospy.ServiceException) as e:
#             print ("/gazebo/pause_physics service call failed")

#         state, done = self.process_image(data)

#         # Set the rewards for your action
#         if not done:
#             if action == 0:  # FORWARD
#                 reward = 4
#             elif action == 1:  # LEFT
#                 reward = 2
#             else:
#                 reward = 2  # RIGHT
#         else:
#             reward = -200

#         return state, reward, done, {}

#     def reset(self):

#         print("Episode history: {}".format(self.episode_history))
#         self.episode_history = []
#         print("Resetting simulation...")
#         # Resets the state of the environment and returns an initial
#         # observation.
#         rospy.wait_for_service('/gazebo/reset_simulation')
#         try:
#             # reset_proxy.call()
#             self.reset_proxy()
#         except (rospy.ServiceException) as e:
#             print ("/gazebo/reset_simulation service call failed")

#         # Unpause simulation to make observation
#         rospy.wait_for_service('/gazebo/unpause_physics')
#         try:
#             # resp_pause = pause.call()
#             self.unpause()
#         except (rospy.ServiceException) as e:
#             print ("/gazebo/unpause_physics service call failed")

#         # read image data
#         data = None
#         while data is None:
#             try:
#                 data = rospy.wait_for_message('/pi_camera/image_raw',
#                                               Image, timeout=5)
#             except:
#                 pass

#         rospy.wait_for_service('/gazebo/pause_physics')
#         try:
#             # resp_pause = pause.call()
#             self.pause()
#         except (rospy.ServiceException) as e:
#             print ("/gazebo/pause_physics service call failed")

#         self.timeout = 0
#         state, done = self.process_image(data)

#         return state

class Gazebo_Linefollow_Env(gazebo_env.GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        LAUNCH_FILE = '/home/fizzer/enph353_gym-gazebo-noetic/gym_gazebo/envs/ros_ws/src/linefollow_ros/launch/linefollow_world.launch'
        gazebo_env.GazeboEnv.__init__(self, LAUNCH_FILE)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world',
                                              Empty)

        self.action_space = spaces.Discrete(3)  # F,L,R
        self.reward_range = (-np.inf, np.inf)
        self.episode_history = []
        self._seed()

        self.bridge = CvBridge()
        self.timeout = 0  # Used to keep track of images with no line detected


    def process_image(self, data):
        '''
            @brief Coverts data into a opencv image and displays it
            @param data : Image data from ROS

            @retval (state, done)
        '''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("raw", cv_image)

        NUM_BINS = 10
        state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        done = False
        height = cv_image.shape[0]
        width = cv_image.shape[1]
        line_detected = True


        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Blurr Background
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Bin Map
        thresh = 127 ## update to change based on HSB and/or current conditions
        ret, frame_bin = cv2.threshold(blurred, thresh, 255, 0)

        row_of_interest = height - 20  # Line 50 pixels from the bottom
        line_data = frame_bin[row_of_interest, 0:width]  # Extract the specific row
        right_found = False
        left_found = False

        leftmost = 0
        rightmost = 0

        for x in range(len(line_data)):
            # Get the leftmost and rightmost points
            if not left_found and line_data[x] == 0:
                leftmost = x
                left_found = True
            if not right_found and line_data[width - 2 - x] == 0:
                rightmost = width - 1 - x
                right_found = True
        if right_found == False or left_found == False:
            self.timeout += 1
            line_detected = False
            if self.timeout >= 30:
                done = True
        
        if line_detected:
            # Calculate the midpoint
            midpoint = (leftmost + rightmost) / 2

            # Put into bin
            bin_size = width / NUM_BINS
            assigned_bin = math.floor(midpoint / bin_size) # floor to stay within range of 0-9
            state[assigned_bin] = 1

        return state, done, line_detected

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        self.episode_history.append(action)

        vel_cmd = Twist()

        if action == 0:  # FORWARD
            vel_cmd.linear.x = 0.5
            vel_cmd.angular.z = 0.0
        elif action == 1:  # LEFT
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 0.4
        elif action == 2:  # RIGHT
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = -0.4

        self.vel_pub.publish(vel_cmd)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/pi_camera/image_raw', Image,
                                              timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            # resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state, done, line_found = self.process_image(data)

        # Set the rewards for your action
        if not done:
            # position = np.argmax(state)
            if action == 0:
                reward = 2
            elif action == 1:  # LEFT
                reward = 1
            else:
                reward = 1
            if line_found:
                reward += 1
        else:
            reward = -10
            self.reset()

        return state, reward, done, {}

    def reset(self):

        print("Episode history: {}".format(self.episode_history))
        self.episode_history = []
        print("Resetting simulation...")
        # Resets the state of the environment and returns an initial
        # observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            # reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            # resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        # read image data
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/pi_camera/image_raw',
                                              Image, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            # resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        self.timeout = 0
        state, done, line_found = self.process_image(data)

        return state
