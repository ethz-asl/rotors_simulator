#!/usr/bin/env python
import os
import rospy
import rospkg
import numpy as np
import time
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandLong
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import StreamRate
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtGui import QFormLayout
from python_qt_binding.QtCore import QTimer, Slot
from rotors_comm.srv import RecordRosbag
from std_msgs.msg import Bool
from std_srvs.srv import Empty

class MavModeWidget(QWidget):
  # MAV mode flags
  MAV_MODE_FLAG_SAFETY_ARMED = 128
  MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64
  MAV_MODE_FLAG_HIL_ENABLED = 32
  MAV_MODE_FLAG_STABILIZE_ENABLED = 16
  MAV_MODE_FLAG_GUIDED_ENABLED = 8
  MAV_MODE_FLAG_AUTO_ENABLED = 4
  MAV_MODE_FLAG_TEST_ENABLED = 2
  MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

  # MAV state dictionary
  mav_state = {0: 'Uninitialized',
  1: 'Booting up',
  2: 'Calibrating',
  3: 'Standby',
  4: 'Active',
  5: 'Critical',
  6: 'Emergency',
  7: 'Poweroff'}

  # String constants
  STR_UNKNOWN = 'N/A'

  STR_MAVROS_ARM_SERVICE_NAME = '/mavros/cmd/arming'
  STR_MAVROS_COMMAND_LONG_SERVICE_NAME = '/mavros/cmd/command'
  STR_MAVROS_SET_MODE_SERVICE_NAME = '/mavros/set_mode'
  STR_MAVROS_SET_STREAM_RATE_SERVICE_NAME = '/mavros/set_stream_rate'
  STR_RECORD_ROSBAG_SERVICE_NAME = 'record_rosbag'
  STR_RESET_MODEL_SERVICE_NAME = 'reset_model'

  STR_SYS_STATUS_SUB_TOPIC = '/mavros/state'
  STR_START_RECONSTRUCTION_PUB_TOPIC = 'start_reconstruction'

  STR_BUTTON_LABEL_RECORD_BAG = 'Record Bag'
  STR_BUTTON_LABEL_STOP_RECORDING_BAG = 'Stop Recording'

  def __init__(self, parent):
    # Init QWidget
    super(MavModeWidget, self).__init__(parent)
    self.setObjectName('MavModeWidget')
    
    # Load UI
    ui_file = os.path.join(rospkg.RosPack().get_path('rqt_rotors'), 'resource', 'mav_mode_widget.ui')
    loadUi(ui_file, self)

    # Set the initial parameters of UI elements
    self.button_set_hil_mode.setEnabled(False)
    self.button_arm.setEnabled(False)
    self.button_reboot_autopilot.setEnabled(False)
    self.text_state.setText(self.STR_UNKNOWN)
    self.clear_mav_mode()

    # Initialize class variables
    self.last_heartbeat_time = time.time()
    self.mav_mode = 65
    self.mav_status = 255
    self.armed = False
    self.connected = False
    self.guided = False
    self.hil_enabled = False
    self.is_recording = False
    self.reconstruction_started = False
    
    # Set the functions that are called when signals are emitted
    self.button_set_hil_mode.pressed.connect(self.on_set_hil_mode_button_pressed)
    self.button_arm.pressed.connect(self.on_arm_button_pressed)
    self.button_reboot_autopilot.pressed.connect(self.on_reboot_autopilot_button_pressed)
    self.button_record_rosbag.pressed.connect(self.on_record_rosbag_button_pressed)
    self.button_reset_model.pressed.connect(self.on_reset_model_button_pressed)
    self.button_reconstruct.pressed.connect(self.on_reconstruct_button_pressed)
    self.button_set_position_rate.pressed.connect(self.on_set_position_rate_button_pressed)
    self.button_set_imu_rate.pressed.connect(self.on_set_imu_rate_button_pressed)

    # Create ROS service proxies
    self.arm = rospy.ServiceProxy(self.STR_MAVROS_ARM_SERVICE_NAME, CommandBool)
    self.record_rosbag = rospy.ServiceProxy(self.STR_RECORD_ROSBAG_SERVICE_NAME, RecordRosbag)
    self.reset_model = rospy.ServiceProxy(self.STR_RESET_MODEL_SERVICE_NAME, Empty)
    self.send_command_long = rospy.ServiceProxy(self.STR_MAVROS_COMMAND_LONG_SERVICE_NAME, CommandLong)
    self.set_mode = rospy.ServiceProxy(self.STR_MAVROS_SET_MODE_SERVICE_NAME, SetMode)
    self.set_stream_rate = rospy.ServiceProxy(self.STR_MAVROS_SET_STREAM_RATE_SERVICE_NAME, StreamRate)
    
    # Initialize ROS subscribers and publishers
    self.sys_status_sub = rospy.Subscriber(self.STR_SYS_STATUS_SUB_TOPIC, State, self.sys_status_callback, queue_size=1)
    self.start_reconstruction_pub = rospy.Publisher(self.STR_START_RECONSTRUCTION_PUB_TOPIC, Bool, queue_size=1)

  def on_set_hil_mode_button_pressed(self):
    new_mode = self.mav_mode | self.MAV_MODE_FLAG_HIL_ENABLED
    self.hil_enabled = True
    self.mav_mode = new_mode
    self.set_mode(new_mode, '')
    self.text_mode_hil.setText(self.mav_mode_text(self.hil_enabled))

  def on_arm_button_pressed(self):
    self.arm(True)

  def on_reboot_autopilot_button_pressed(self):
    self.send_command_long(False, 246, 1, 1, 0, 0, 0, 0, 0, 0)

  def on_record_rosbag_button_pressed(self):
    if not(self.is_recording):
      self.is_recording = True
      self.record_rosbag(self.is_recording)
      self.button_record_rosbag.setText(self.STR_BUTTON_LABEL_STOP_RECORDING_BAG)
    else:
      self.is_recording = False
      self.record_rosbag(self.is_recording)
      self.button_record_rosbag.setText(self.STR_BUTTON_LABEL_RECORD_BAG)

  def on_reset_model_button_pressed(self):
    self.reset_model()

  def on_reconstruct_button_pressed(self):
    if not(self.reconstruction_started):
      self.start_reconstruction_pub.publish(True)
    self.reconstruction_started = True
    self.button_reconstruct.setEnabled(False)

  def on_set_position_rate_button_pressed(self):
    self.set_stream_rate(33, 20, True)

  def on_set_imu_rate_button_pressed(self):
    self.set_stream_rate(105, 20, True)
    
  def sys_status_callback(self, msg):
    if (not self.connected and msg.connected):
      self.button_set_hil_mode.setEnabled(True)
      self.button_arm.setEnabled(True)
      self.button_reboot_autopilot.setEnabled(True)
      self.connected = True
      self.last_heartbeat_time = time.time()
      self.text_mode_safety_armed.setText(self.mav_mode_text(msg.armed))
      self.text_mode_guided.setText(self.mav_mode_text(msg.guided))
      return

    if (((time.time() - self.last_heartbeat_time) >= 2.0) and self.hil_enabled):
      new_mode = self.mav_mode | self.MAV_MODE_FLAG_HIL_ENABLED
      self.set_mode(new_mode, '')

    if (self.armed != msg.armed):
      self.armed = msg.armed
      self.text_mode_safety_armed.setText(self.mav_mode_text(self.armed))
      self.button_arm.setEnabled(not(self.armed))
      self.mav_mode = self.mav_mode | self.MAV_MODE_FLAG_SAFETY_ARMED

    if (self.guided != msg.guided):
      self.guided = msg.guided
      self.text_mode_guided.setText(self.mav_mode_text(self.guided))

    self.last_heartbeat_time = time.time()

  def clear_mav_mode(self):
    count = self.mav_mode_layout.rowCount()
    for i in range(count):
      self.mav_mode_layout.itemAt(i, QFormLayout.FieldRole).widget().setText(self.STR_UNKNOWN)

  def mav_mode_text(self, mode_enabled):
    return 'ON' if mode_enabled else 'OFF'
