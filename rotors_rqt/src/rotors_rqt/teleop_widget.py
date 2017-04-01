#!/usr/bin/env python
import os
import rospkg
import rospy
import select
import sys
import termios
import tty
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QFormLayout
from python_qt_binding.QtCore import QTimer, Slot
from mav_msgs.msg import Actuators

class TeleopWidget(QWidget):
  # String constants
  STR_ACTUATORS_SUB_TOPIC = 'gazebo/command/motor_speed'

  def __init__(self, parent):
    # Init QWidget
    super(TeleopWidget, self).__init__(parent)
    self.setObjectName('TeleopWidget')
    
    # Load UI
    ui_file = os.path.join(rospkg.RosPack().get_path('rqt_rotors'), 'resource', 'teleop_widget.ui')
    loadUi(ui_file, self)

    # Set the initial slider positions
    self.slider_aileron.setSliderPosition(50)
    self.slider_elevator.setSliderPosition(50)
    self.slider_rudder.setSliderPosition(50)
    self.slider_throttle.setSliderPosition(0)

    # Initialize ROS subscribers
    self.actuators_sub = rospy.Subscriber(self.STR_ACTUATORS_SUB_TOPIC, Actuators, self.actuators_callback, queue_size=1)

  def actuators_callback(self, msg):
    self.slider_aileron.setSliderPosition(50 + 50 * msg.normalized[0])
    self.slider_elevator.setSliderPosition(50 + 50 * msg.normalized[1])
    self.slider_rudder.setSliderPosition(50 + 50 * msg.normalized[2])
    self.slider_throttle.setSliderPosition(100 * msg.normalized[5])
