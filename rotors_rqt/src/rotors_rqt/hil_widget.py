#!/usr/bin/env python
from python_qt_binding import loadUi
from python_qt_binding import QtGui
from python_qt_binding import QtCore

from .mav_mode_widget import MavModeWidget
from .teleop_widget import TeleopWidget

class HilWidget(QtGui.QWidget):

	def __init__(self):
		super(HilWidget, self).__init__()
		self.setObjectName('HilWidget')

		self.row_1 = QtGui.QHBoxLayout()

		main_layout = QtGui.QGridLayout()
		main_layout.addLayout(self.row_1, 0, 0, QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop)

		#self.row_1.setAlignment(QtCore.Qt.AlignTop)
		#main_layout.setAlignment(QtCore.Qt.AlignTop)

		self.setLayout(main_layout)

		self.row_1.addWidget(MavModeWidget(self))
		self.row_1.addWidget(TeleopWidget(self))
