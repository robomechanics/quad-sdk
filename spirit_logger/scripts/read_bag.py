#!/usr/bin/python

# This is the main file being called for logging through commandline. It calls mouse interface and read bag
# for UI operations and processing the bag files
 
from __future__ import print_function
import time
import sys
import bag_reader
import numpy as np
from PyQt5 import QtCore, QtWidgets
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

import mouse_interface

data = {}
topics_selected = []
topics_has_read = []
variables_selected = []

# This class is responsible for extracting message types and fields within each message 
class Example(QtWidgets.QWidget):

    def __init__(self):
        super(Example, self).__init__()
        self.initUI()

    def initUI(self):
        has_type = []
        frame_left = {}
        vbox_left = {}
        label_left = {}
        topic_type_dict = bag_reader.read_bag(sys.argv[1])

        for msg_type in topic_type_dict.values():
            if bag_reader.msg_types.count(msg_type) and not has_type.count(msg_type):
                has_type.append(msg_type)

        splitter_left = QtWidgets.QSplitter(QtCore.Qt.Vertical)

        # Separates the variables to different categories in order to populate the screen 
        for msg_type in has_type:
            frame_left[msg_type] = QtWidgets.QFrame(self)
            frame_left[msg_type].setFrameShape(QtWidgets.QFrame.StyledPanel)
            vbox_left[msg_type] = QtWidgets.QVBoxLayout(self)
            label_left[msg_type] = QtWidgets.QLabel(self)
            label_left[msg_type].setText(msg_type)
            label_left[msg_type].adjustSize()
            vbox_left[msg_type].addWidget(label_left[msg_type])
            for topic in topic_type_dict.keys():
              if topic_type_dict[topic] == msg_type:
                  cb = QtWidgets.QCheckBox(topic, self)
                  cb.stateChanged.connect(self.addTopic)
                  vbox_left[msg_type].addWidget(cb)

            vbox_left[msg_type].addStretch(1)
            frame_left[msg_type].setLayout(vbox_left[msg_type])
            splitter_left.addWidget(frame_left[msg_type])

        vbox_left_var = QtWidgets.QVBoxLayout(self)
        label_left_var = QtWidgets.QLabel(self)
        label_left_var.setText('Variables')
        label_left_var.adjustSize()
        vbox_left_var.addWidget(label_left_var)

        frame_left_var = QtWidgets.QFrame(self)
        frame_left_var.setFrameShape(QtWidgets.QFrame.StyledPanel)

        for var in bag_reader.var_types:
            cb = QtWidgets.QCheckBox(var, self)
            cb.stateChanged.connect(self.addVar)
            vbox_left_var.addWidget(cb)
        vbox_left_var.addStretch(0)
        frame_left_var.setLayout(vbox_left_var)
        splitter_left.addWidget(frame_left_var)

        frame_left_control = QtWidgets.QFrame(self)
        frame_left_control.setFrameShape(QtWidgets.QFrame.StyledPanel)

        vbox_left_control = QtWidgets.QVBoxLayout(self)
        load_btn = QtWidgets.QPushButton('Load', self)
        clear_btn = QtWidgets.QPushButton('Clear', self)
        save_btn = QtWidgets.QPushButton('Save Figure', self)
        load_btn.clicked.connect(self.buttonClicked)
        clear_btn.clicked.connect(self.buttonClearClicked)
        save_btn.clicked.connect(self.buttonSaveClicked)
        vbox_left_control.addWidget(load_btn)
        vbox_left_control.addWidget(clear_btn)
        vbox_left_control.addWidget(save_btn)
        vbox_left_control.addStretch(1)
        frame_left_control.setLayout(vbox_left_control)
        splitter_left.addWidget(frame_left_control)


        frame_right = QtWidgets.QFrame(self)
        frame_right.setFrameShape(QtWidgets.QFrame.StyledPanel)


        splitter_v = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        splitter_v.addWidget(splitter_left)
        splitter_v.addWidget(frame_right)

        hbox = QtWidgets.QHBoxLayout(self)
        hbox.addWidget(splitter_v)

        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        vbox_right = QtWidgets.QVBoxLayout(self)
        vbox_right.addWidget(self.canvas)
        frame_right.setLayout(vbox_right)


        self.setLayout(hbox)
        self.resize(1000, 800)
        self.center()

        self.show()

    # Returns a list of topics that are actually filled with values among all listed topics 
    def topics_to_read(self):
        topics_to_read = []
        for topic in topics_selected:
            if not topics_has_read.count(topic):
                topics_has_read.append(topic)
                if not topics_to_read.count(topic):
                   topics_to_read.append(topic)
        return topics_to_read

    def plot(self):
        new_data = bag_reader.read_msg(self.topics_to_read())
        for key in new_data.keys():
            if key not in data:
                data[key] = new_data[key]

        mouse_interface.plot_data(data, variables_selected, topics_selected)
        self.canvas.draw()

    def addTopic(self, state):
        topic = str(self.sender().text())
        if state == QtCore.Qt.Checked:
            if not topics_selected.count(topic):
                topics_selected.append(topic)
        else:
            if topics_selected.count(topic):
                topics_selected.remove(topic)

    def addVar(self, state):
        var = str(self.sender().text())
        if state == QtCore.Qt.Checked:
            if not variables_selected.count(var):
                variables_selected.append(var)
        else:
            if variables_selected.count(var):
                variables_selected.remove(var)

    def buttonClearClicked(self):
        mouse_interface.clear()

    def buttonSaveClicked(self):
        mouse_interface.save()

    def buttonClicked(self):
        self.plot()

    def keyPressEvent(self, e):

        if e.key() == QtCore.Qt.Key_Escape:
            self.close()
        elif e.key() == QtCore.Qt.Key_Shift:
            mouse_interface.shift_hold = True
        else:
            mouse_interface.shift_hold = False

    def keyReleaseEvent(self, e):
        if e.key() == QtCore.Qt.Key_Shift:
            mouse_interface.shift_hold = False

    def center(self):
        qr = self.frameGeometry()
        cp = QtWidgets.QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

def main():
    # Instantiates a UI object and checks for valid user input
    app = QtWidgets.QApplication(sys.argv)
    if(len(sys.argv) < 2):
        print('Usage:',sys.argv[0],' <bag_file>')
        sys.exit(1)
    ex = Example()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
