#!/usr/bin/env python
import os
import rospy
import rospkg

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandLong
from mavros_msgs.srv import SetMode

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding import QtCore
from python_qt_binding.QtCore import QTimer, Slot
from python_qt_binding.QtGui import QWidget, QFormLayout

import time

class HilPlugin(Plugin):
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

    # Constants
    STR_ON = 'ON'
    STR_OFF = 'OFF'
    STR_UNKNOWN = 'N/A'

    STR_MAVROS_ARM_SERVICE_NAME = '/mavros/cmd/arming'
    STR_MAVROS_COMMAND_LONG_SERVICE_NAME = '/mavros/cmd/command'
    STR_MAVROS_SET_MODE_SERVICE_NAME = '/mavros/set_mode'

    STR_SYS_STATUS_SUB_TOPIC = '/mavros/state'

    TIMEOUT_HIL_HEARTBEAT = 2.0

    def __init__(self, context):
        super(HilPlugin, self).__init__(context)
        self.setObjectName('HilPlugin')

        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_rotors'), 'resource', 'HilPlugin.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('HilPluginUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        context.add_widget(self._widget)

        # Set the initial parameters of UI elements
        self._widget.button_set_hil_mode.setEnabled(False)
        self._widget.button_arm.setEnabled(False)
        self._widget.button_reboot_autopilot.setEnabled(False)
        self._widget.text_state.setText(self.STR_UNKNOWN)
        self.clear_mav_mode()

        # Initialize class variables
        self.last_heartbeat_time = time.time()
        self.mav_mode = 65
        self.mav_status = 255
        self.armed = False
        self.connected = False
        self.guided = False
        self.hil_enabled = False
        
        # Set the functions that are called when signals are emitted
        self._widget.button_set_hil_mode.pressed.connect(self.on_set_hil_mode_button_pressed)
        self._widget.button_arm.pressed.connect(self.on_arm_button_pressed)
        self._widget.button_reboot_autopilot.pressed.connect(self.on_reboot_autopilot_button_pressed)

        # Create ROS service proxies
        self.arm = rospy.ServiceProxy(self.STR_MAVROS_ARM_SERVICE_NAME, CommandBool)
        self.send_command_long = rospy.ServiceProxy(self.STR_MAVROS_COMMAND_LONG_SERVICE_NAME, CommandLong)
        self.set_mode = rospy.ServiceProxy(self.STR_MAVROS_SET_MODE_SERVICE_NAME, SetMode)
        
        # Initialize ROS subscribers and publishers
        self.sys_status_sub = rospy.Subscriber(self.STR_SYS_STATUS_SUB_TOPIC, State, self.sys_status_callback, queue_size=1)

    def on_set_hil_mode_button_pressed(self):
        new_mode = self.mav_mode | self.MAV_MODE_FLAG_HIL_ENABLED
        self.hil_enabled = True
        self.mav_mode = new_mode
        self.set_mode(new_mode, '')
        self._widget.text_mode_hil.setText(self.mav_mode_text(self.hil_enabled))

    def on_arm_button_pressed(self):
        self.arm(True)

    def on_reboot_autopilot_button_pressed(self):
        self.send_command_long(False, 246, 1, 1, 0, 0, 0, 0, 0, 0)
    
    def sys_status_callback(self, msg):
        if (not self.connected and msg.connected):
            self._widget.button_set_hil_mode.setEnabled(True)
            self._widget.button_arm.setEnabled(True)
            self._widget.button_reboot_autopilot.setEnabled(True)
            self.connected = True
            self.last_heartbeat_time = time.time()
            self._widget.text_mode_safety_armed.setText(self.mav_mode_text(msg.armed))
            self._widget.text_mode_guided.setText(self.mav_mode_text(msg.guided))
            return

        if (((time.time() - self.last_heartbeat_time) >= self.TIMEOUT_HIL_HEARTBEAT) and self.hil_enabled):
            new_mode = self.mav_mode | self.MAV_MODE_FLAG_HIL_ENABLED
            self.set_mode(new_mode, '')

        if (self.armed != msg.armed):
            self.armed = msg.armed
            self._widget.text_mode_safety_armed.setText(self.mav_mode_text(self.armed))
            self._widget.button_arm.setEnabled(not(self.armed))
            self.mav_mode = self.mav_mode | self.MAV_MODE_FLAG_SAFETY_ARMED

        if (self.guided != msg.guided):
            self.guided = msg.guided
            self._widget.text_mode_guided.setText(self.mav_mode_text(self.guided))

        self.last_heartbeat_time = time.time()

    def clear_mav_mode(self):
        count = self._widget.mav_mode_layout.rowCount()
        for i in range(count):
            self._widget.mav_mode_layout.itemAt(i, QFormLayout.FieldRole).widget().setText(self.STR_UNKNOWN)

    def mav_mode_text(self, mode_enabled):
        return self.STR_ON if mode_enabled else self.STR_OFF

    def shutdown_plugin(self):
        if self.sys_status_sub is not None:
            self.sys_status_sub.unregister()
