import datetime
import time
from collections import deque
from multiprocessing import Queue
import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from flight_controller import *

# length of GUI
l = 800

# width of GUI
w = 500

# receiver created with receiver channels initialized
receiver = Receiver(17, 27, 22, 18, 23)
# IMU created with MPU_6050, alpha initialized
imu = IMU(0x68, 0.98)
# motors created with motors initialized
motor = Motor(10, 9, 25, 8)
# initialize the flight controller
fc = FlightController(np.array([0.1, 0.2, 0.3]), np.array([0.4, 0.5, 0.6]),
                      np.array([0.7, 0.8, 0.9]), receiver, imu, motor)

# TODO:
# potential sensor/graph plot for 'wm' variable in FlightController.py
# buttons to have drone perform certain maneuvers (need further guidance)
# transition to layouts, get rid of absolute positioning


class DroneGUI(QDialog):
    global l, w

    def __init__(self, parent=None):
        super(DroneGUI, self).__init__(parent)

        # create tabs
        self.create_tabs()

        # set the size policy
        self.set_size_policy()

        # palette background setting, style
        self.create_palette()

        # adds the arm, kill, unarm buttons
        self.create_arm_kill_buttons()

        # adds the PID insert text boxes
        self.create_PID_insert()

        # adds the shortcut labels
        self.create_shortcut_labels()

        # adds the log
        self.create_log()

        # stream for the command line output to fill the log
        sys.stdout = Stream(newText=self.onUpdateText)

        # adds the data monitor
        self.create_data_monitor()

        # installs the event filter for 'space bar' and 'a' keys
        qApp.installEventFilter(self)

        # creates a button to deliver a live plot graph
        self.create_plot_button()

        # create the flight motion labels
        self.create_flight_motion_labels()

        # shows the current information of the drone
        self.drone_information()

    # filters space bar to allow it to be a killswitch only
    # filters 'a' to allow it to arm the drone only
    # filters shift + 'q' to allow it to close the app
    def eventFilter(self, obj, event):
        if event.type() == QEvent.KeyPress:
            if event.key() == Qt.Key_Space:
                if self.arm_button.isChecked():
                    print("KILLSWITCH ACTIVATED")
                    self.arm_button.toggle()
                    self.arm_button.setEnabled(False)
                    self.undo_killswitch_button.setEnabled(True)
                    self.undo_killswitch_button.setStyleSheet("background-color: yellow; color:black")
                    self.arm_button.setStyleSheet("background-color: Gray")
                    self.killswitch_button.setEnabled(False)
                    self.killswitch_button.setStyleSheet("background-color: Darkred; color:black")
                    self.curr_status.setStyleSheet("background-color: Red")
                    self.curr_status.setText("Inactive")
                    # fc.receiver.ARM = False
                    return True
                else:
                    return True
            if event.key() == Qt.Key_A:
                if not(self.arm_button.isChecked()):
                    print("Initialize Arming Process")
                    self.arm_button.setStyleSheet("background-color: Green")
                    self.undo_killswitch_button.setEnabled(False)
                    self.killswitch_button.setEnabled(True)
                    self.killswitch_button.setStyleSheet("background-color: red")
                    self.undo_killswitch_button.setStyleSheet("background-color:rgb(53,53,53);")
                    self.arm_button.setEnabled(False)
                    self.arm_button.setChecked(True)
                    self.curr_status.setStyleSheet("background-color: Green")
                    self.curr_status.setText("Active")
                    # Since we're skipping the unlock arm button,
                    # fc.receiver.ARM = True
                    # fc.run()
                    return True
                else:
                    return True
            if event.key() == Qt.Key_Q:
                DroneGUI.close(self)
        return super(DroneGUI, self).eventFilter(obj, event)

    def set_size_policy(self):
        self.sizePolicy = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.sizePolicy.setHorizontalStretch(0)
        self.sizePolicy.setVerticalStretch(0)
        self.sizePolicy.setHeightForWidth(self.sizePolicy.hasHeightForWidth())
        self.setSizePolicy(self.sizePolicy)
        self.setMaximumSize(l, w)
        self.setMinimumSize(l, w)

    def create_tabs(self):
        self.tabWidget = QTabWidget(self)
        self.tabWidget.setGeometry(QRect(0, 150, l-260,w-100 ))
        self.tabWidget.setMaximumSize(QSize(l, w-100))
        self.tabWidget.setTabPosition(QTabWidget.North)
        self.tabWidget.setTabShape(QTabWidget.Rounded)
        self.tabWidget.setElideMode(Qt.ElideRight)
        self.tabWidget.setUsesScrollButtons(False)
        self.tabWidget.setDocumentMode(True)
        self.tabWidget.setTabsClosable(False)
        self.tabWidget.setMovable(False)
        self.tabWidget.setTabBarAutoHide(False)
        self.data_tab = QWidget()
        self.settings_tab = QWidget()
        self.flight_pattern_tab = QWidget()
        self.CV_tab = QWidget()
        self.tabWidget.addTab(self.settings_tab, "Settings")
        self.tabWidget.addTab(self.data_tab, "Data")
        self.tabWidget.addTab(self.flight_pattern_tab, "Flight Pattern")
        self.tabWidget.addTab(self.CV_tab, "CV")
        self.tabWidget.show()

    def create_palette(self):
        self.setWindowTitle("ARC: Flight Control")
        app.setStyle("Fusion")
        font = QFont("Helvetica")
        app.setFont(font)
        dark_palette = QPalette()
        dark_palette.setColor(QPalette.Window, QColor(53, 53, 53))
        dark_palette.setColor(QPalette.WindowText, Qt.white)
        dark_palette.setColor(QPalette.Base, QColor(25, 25, 25))
        dark_palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
        dark_palette.setColor(QPalette.ToolTipBase, Qt.white)
        dark_palette.setColor(QPalette.ToolTipText, Qt.white)
        dark_palette.setColor(QPalette.Text, Qt.white)
        dark_palette.setColor(QPalette.Button, QColor(53, 53, 53))
        dark_palette.setColor(QPalette.ButtonText, Qt.white)
        dark_palette.setColor(QPalette.BrightText, Qt.red)
        dark_palette.setColor(QPalette.Link, QColor(42, 130, 218))
        dark_palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
        dark_palette.setColor(QPalette.HighlightedText, Qt.black)
        app.setPalette(dark_palette)

    # create buttons to arm, and kill, and un-kill the drone
    def create_arm_kill_buttons(self):
        # button to arm the drone
        self.arm_button = QPushButton('ARM', self)
        self.arm_button.setDefault(False)
        self.arm_button.setAutoDefault(False)
        self.arm_button.setStyleSheet("background-color: green")
        self.arm_button.move(0, 0)
        self.arm_button.resize(70, 43)
        self.arm_button.setFont(QFont("Helvetica", 17.5))
        self.arm_button.setCheckable(True)
        self.arm_button.setEnabled(True)
        self.arm_button.clicked.connect(self.arm_drone)

        # button to kill the drone
        self.killswitch_button = QPushButton('KILLSWITCH', self)
        self.killswitch_button.move(l-110, 0)
        self.killswitch_button.setDefault(False)
        self.killswitch_button.setAutoDefault(False)
        self.killswitch_button.setFont(QFont("Helvetica", 17.5))
        self.killswitch_button.resize(110, 60)
        self.killswitch_button.clicked.connect(self.kill_motor)
        self.killswitch_button.setStyleSheet("background-color: red")
        self.killswitch_button.setEnabled(False)

        # button to undo kill switch
        self.undo_killswitch_button = QPushButton('Unlock ARM', self)
        self.undo_killswitch_button.setDefault(False)
        self.undo_killswitch_button.setAutoDefault(False)
        self.undo_killswitch_button.move(l-110, 62)
        self.undo_killswitch_button.setFont(QFont("Helvetica", 12))
        self.undo_killswitch_button.resize(75, 30)
        self.undo_killswitch_button.clicked.connect(self.undo_killswitch)
        self.undo_killswitch_button.setStyleSheet("background-color:rgb(53,53,53);")
        self.undo_killswitch_button.setEnabled(False)

    # killswitch for the drone
    def kill_motor(self):
        if self.arm_button.isChecked():
            print("KILLSWITCH ACTIVATED")
            self.arm_button.toggle()
            self.arm_button.setEnabled(False)
            self.undo_killswitch_button.setEnabled(True)
            self.undo_killswitch_button.setStyleSheet("background-color: yellow; color:black")
            self.killswitch_button.setEnabled(False)
            self.killswitch_button.setStyleSheet("background-color: Darkred; color:black")
            self.arm_button.setStyleSheet("background-color: Gray")
            self.curr_status.setStyleSheet("background-color: Red")
            self.curr_status.setText("Inactive")
            # fc.receiver.ARM = False

    # allows the drone to be armed again
    def undo_killswitch(self):
        print("Arm button unlocked")
        self.arm_button.setEnabled(True)
        self.undo_killswitch_button.setEnabled(False)
        self.killswitch_button.setStyleSheet("background-color: red")
        self.undo_killswitch_button.setStyleSheet("background-color: Gold")
        self.arm_button.setStyleSheet("background-color: Green")
        # fc.receiver.ARM = True

    # arms the drone
    def arm_drone(self):
        print("Initialize Arming Process...")
        self.undo_killswitch_button.setEnabled(False)
        self.killswitch_button.setEnabled(True)
        self.undo_killswitch_button.setStyleSheet("background-color:rgb(53,53,53);")
        self.arm_button.setEnabled(False)
        self.curr_status.setStyleSheet("background-color: Green")
        self.curr_status.setText("Active")
        # fc.run()

    # displays current drone details
    def drone_information(self):

        # PID
        self.state_label = QLabel("Drone Status", self.settings_tab)
        self.state_label.move(0, w-300)
        self.state_label.show()
        self.state_label.setFrameShape(QFrame.Panel)
        self.state_label.setFrameShadow(QFrame.Sunken)
        self.state_label.resize(80,18)
        self.curr_status = QPushButton("Inactive", self.settings_tab)
        self.curr_status.setDefault(False)
        self.curr_status.setEnabled(False)
        self.curr_status.setStyleSheet("background-color: Red")
        self.curr_status.move(90, w-300)
        self.curr_status.resize(60,18)
        self.curr_P = QLineEdit(self.settings_tab)
        self.curr_P.setReadOnly(True)
        self.curr_P_label = QLabel(self.settings_tab)
        self.curr_P_label.move(1, w-275)
        self.curr_P_label.setText("Proportional:")
        self.curr_P_label.show()
        self.curr_P.setText(str(fc.Kp))  # " ", str(fc.Ki), " ", str(fc.Kd))
        self.curr_P.move(78, w-275)
        self.curr_P.resize(90,12)
        self.curr_P.show()
        self.curr_I_label = QLabel(self.settings_tab)
        self.curr_I_label.move(1, w - 262)
        self.curr_I_label.setText("       Integral:")
        self.curr_I_label.show()
        self.curr_I = QLineEdit(self.settings_tab)
        self.curr_I.setReadOnly(True)
        self.curr_I.setText(str(fc.Ki))  # " ", str(fc.Ki), " ", str(fc.Kd))
        self.curr_I.move(78, w - 262)
        self.curr_I.resize(90,12)
        self.curr_I.show()
        self.curr_D_label = QLabel(self.settings_tab)
        self.curr_D_label.move(1, w - 249)
        self.curr_D_label.setText("   Derivative:")
        self.curr_D_label.show()
        self.curr_D = QLineEdit(self.settings_tab)
        self.curr_D.setReadOnly(True)
        self.curr_D.setText(str(fc.Kd))  # " ", str(fc.Ki), " ", str(fc.Kd))
        self.curr_D.move(78, w - 249)
        self.curr_D.resize(90, 12)
        self.curr_D.show()

        # Motors
        self.curr_motor = QLabel(self.settings_tab)
        self.curr_motor.move(200, w - 275)
        self.curr_motor.setText("Motors:")
        self.curr_motor_1 = QLabel(self.settings_tab)
        self.curr_motor_1.move(270, w - 275)
        self.curr_motor_1a = QLabel("1:",self.settings_tab)
        self.curr_motor_1a.move(250, w - 275)
        self.curr_motor_1.setText(str(fc.motor.MOTOR1))
        self.curr_motor_2 = QLabel(self.settings_tab)
        self.curr_motor_2.move(270, w - 260)
        self.curr_motor_2.setText(str(fc.motor.MOTOR2))
        self.curr_motor_2a = QLabel("2:", self.settings_tab)
        self.curr_motor_2a.move(250, w - 260)
        self.curr_motor_3 = QLabel(self.settings_tab)
        self.curr_motor_3.move(270, w - 245)
        self.curr_motor_3.setText(str(fc.motor.MOTOR3))
        self.curr_motor_3a = QLabel("3:", self.settings_tab)
        self.curr_motor_3a.move(250, w - 245)
        self.curr_motor_4 = QLabel(self.settings_tab)
        self.curr_motor_4.move(270, w - 230)
        self.curr_motor_4.setText(str(fc.motor.MOTOR4))
        self.curr_motor_4a = QLabel("4:", self.settings_tab)
        self.curr_motor_4a.move(250, w - 230)

        # Receiver channels
        self.curr_rcvr_chan = QLabel(self.settings_tab)
        self.curr_rcvr_chan.setText("Receiver Channels:")
        self.curr_rcvr_chan.move(100, w - 225)
        self.curr_chan_1 = QLabel(self.settings_tab)
        self.curr_chan_1.move(100, w - 210)
        self.curr_chan_1.setText(str(fc.receiver.RECEIVER_CH1))
        self.curr_chan_2 = QLabel(self.settings_tab)
        self.curr_chan_2.move(120, w - 210)
        self.curr_chan_2.setText(str(fc.receiver.RECEIVER_CH2))
        self.curr_chan_3 = QLabel(self.settings_tab)
        self.curr_chan_3.move(140, w - 210)
        self.curr_chan_3.setText(str(fc.receiver.RECEIVER_CH3))
        self.curr_chan_4 = QLabel(self.settings_tab)
        self.curr_chan_4.move(160, w - 210)
        self.curr_chan_4.setText(str(fc.receiver.RECEIVER_CH4))
        self.curr_chan_5 = QLabel(self.settings_tab)
        self.curr_chan_5.move(180, w - 210)
        self.curr_chan_5.setText(str(fc.receiver.RECEIVER_CH5))

        # Alpha
        self.alpha_label = QLabel(self.settings_tab)
        self.alpha_label.setText("Alpha:")
        self.alpha_label.move(20, w-225)
        self.alpha_value = QLabel(self.settings_tab)
        self.alpha_value.move(20, w - 210)
        self.alpha_value.setText(str(fc.imu.alpha))



    # gets the current updated PID values
    def get_PID_value(self):
        try:
            fc.Kp = np.array([float(self.Kp0_textbox.text()), float(self.Kp1_textbox.text()), float(self.Kp2_textbox.text())])
            fc.Ki = np.array([float(self.Ki0_textbox.text()), float(self.Ki1_textbox.text()), float(self.Ki2_textbox.text())])
            fc.Kd = np.array([float(self.Kd0_textbox.text()), float(self.Kd1_textbox.text()), float(self.Kd2_textbox.text())])
        except(ValueError):
            print("Error: Must input values into each coordinate.")
        else:
            print()
            print("DTG: ", datetime.datetime.now())
            print(" - Updated Proportional Gain:      ", "                  Roll: ", self.Kp0_textbox.text(),
                  " Pitch: ", self.Kp1_textbox.text(),
                  " Yaw: ", self.Kp2_textbox.text())
            print(" - Updated Integral Gain:          ", "                      Roll: ", self.Ki0_textbox.text(),
                  " Pitch: ", self.Ki1_textbox.text(),
                  " Yaw: ", self.Ki2_textbox.text())
            print(" - Updated Proportional Gain:      ", "                  Roll: ", self.Kd0_textbox.text(),
                  " Pitch: ", self.Kd1_textbox.text(),
                  " Yaw: ", self.Kd2_textbox.text())
            print()

    # creates the PID text boxes, button for updating PID values
    def create_PID_insert(self):

        # proportional text
        self.onlyDouble = QDoubleValidator()
        self.Kp0_textbox = QLineEdit(self.settings_tab)
        self.Kp0_textbox.clearFocus()
        self.Kp0_textbox.setValidator(self.onlyDouble)
        self.Kp0_textbox.resize(50, 23)
        self.Kp0_textbox.setText(str(fc.Kp[0]))
        self.Kp1_textbox = QLineEdit(self.settings_tab)
        self.Kp1_textbox.setValidator(self.onlyDouble)
        self.Kp1_textbox.resize(50, 23)
        self.Kp1_textbox.move(50,0)
        self.Kp1_textbox.setText(str(fc.Kp[1]))
        self.Kp2_textbox = QLineEdit(self.settings_tab)
        self.Kp2_textbox.setValidator(self.onlyDouble)
        self.Kp2_textbox.resize(50, 23)
        self.Kp2_textbox.move(100, 0)
        self.Kp2_textbox.setText(str(fc.Kp[2]))
        # proportional label
        self.Kp_label = QLabel(self.settings_tab)
        self.Kp_label.setText('Proportional')
        self.Kp_label.move(150, 0)
        self.Kp_label.resize(85, 23)
        self.Kp_label.setFrameShape(QFrame.Panel)
        self.Kp_label.setFrameShadow(QFrame.Sunken)
        self.Kp_label.setLineWidth(3)
        self.Kp_label.setStyleSheet("background-color:rgb(53,53,53);")

        # integral text
        self.Ki0_textbox = QLineEdit(self.settings_tab)
        self.Ki0_textbox.move(0, 27)
        self.Ki0_textbox.resize(50, 23)
        self.Ki0_textbox.setValidator(self.onlyDouble)
        self.Ki0_textbox.setText(str(fc.Ki[0]))
        self.Ki1_textbox = QLineEdit(self.settings_tab)
        self.Ki1_textbox.move(50, 27)
        self.Ki1_textbox.resize(50, 23)
        self.Ki1_textbox.setValidator(self.onlyDouble)
        self.Ki1_textbox.setText(str(fc.Ki[1]))
        self.Ki2_textbox = QLineEdit(self.settings_tab)
        self.Ki2_textbox.move(100, 27)
        self.Ki2_textbox.resize(50, 23)
        self.Ki2_textbox.setValidator(self.onlyDouble)
        self.Ki2_textbox.setText(str(fc.Ki[2]))
        # integral label
        self.Ki_label = QLabel(self.settings_tab)
        self.Ki_label.setText('Integral')
        self.Ki_label.move(150, 27)
        self.Ki_label.resize(85, 23)
        self.Ki_label.setFrameShape(QFrame.Panel)
        self.Ki_label.setFrameShadow(QFrame.Sunken)
        self.Ki_label.setLineWidth(3)
        self.Ki_label.setStyleSheet("background-color:rgb(53,53,53);")

        # derivative text
        self.Kd0_textbox = QLineEdit(self.settings_tab)
        self.Kd0_textbox.move(0, 54)
        self.Kd0_textbox.resize(50, 23)
        self.Kd0_textbox.setValidator(self.onlyDouble)
        self.Kd0_textbox.setText(str(fc.Kd[0]))
        self.Kd1_textbox = QLineEdit(self.settings_tab)
        self.Kd1_textbox.move(50, 54)
        self.Kd1_textbox.resize(50, 23)
        self.Kd1_textbox.setValidator(self.onlyDouble)
        self.Kd1_textbox.setText(str(fc.Kd[1]))
        self.Kd2_textbox = QLineEdit(self.settings_tab)
        self.Kd2_textbox.move(100, 54)
        self.Kd2_textbox.resize(50, 23)
        self.Kd2_textbox.setValidator(self.onlyDouble)
        self.Kd2_textbox.setText(str(fc.Kd[2]))
        # derivative label
        self.Kd_label = QLabel(self.settings_tab)
        self.Kd_label.resize(85, 23)
        self.Kd_label.setText('Derivative')
        self.Kd_label.move(150, 54)
        self.Kd_label.setFrameShape(QFrame.Panel)
        self.Kd_label.setFrameShadow(QFrame.Sunken)
        self.Kd_label.setLineWidth(3)
        self.Kd_label.setStyleSheet("background-color:rgb(53,53,53);")

        # button to insert new PID values
        self.insert_PID_values = QPushButton("Insert PID Gains", self.settings_tab)
        self.insert_PID_values.setStyleSheet("background-color: purple")
        self.insert_PID_values.move(150, 80)
        self.insert_PID_values.resize(85, 25)
        self.insert_PID_values.setFont(QFont("Helvetica", 11.5))
        self.insert_PID_values.setCheckable(True)
        self.insert_PID_values.setEnabled(True)
        self.insert_PID_values.clicked.connect(self.get_PID_value)

        # label for Roll, Pitch, Yaw
        self.RPY = QLabel(self.settings_tab)
        self.RPY.move(0,80)
        self.RPY.setText(' Roll        Pitch         Yaw  ')
        self.RPY.setFrameShape(QFrame.Panel)
        self.RPY.setFrameShadow(QFrame.Sunken)
        self.RPY.setLineWidth(3)
        self.RPY.setStyleSheet("background-color:rgb(53,53,53);")

    # creates shortcut labels
    def create_shortcut_labels(self):
        # label for key shortcuts
        self.key_a_shortcut = QLabel(self)
        self.key_a_shortcut.move(l-123,100)
        self.key_a_shortcut.setText("Press: 'a' to arm,")
        self.key_spacebar_shortcut = QLabel(self)
        self.key_spacebar_shortcut.move(l-158, 116)
        self.key_spacebar_shortcut.setText("          'space bar' to killswitch")

    def onUpdateText(self, text):
        cursor = self.log.textCursor()
        cursor.movePosition(QTextCursor.End)
        cursor.insertText(text)
        self.log.setTextCursor(cursor)
        self.log.ensureCursorVisible()

    def __del__(self):
        sys.stdout = sys.__stdout__

    # creates a log for the command line output
    def create_log(self):
        self.log = QTextEdit(self)
        self.log.move(l-260, w-325)
        self.log.resize(260,325)
        self.log.moveCursor(QTextCursor.Start)
        self.log.ensureCursorVisible()
        self.log.setLineWrapMode(QTextEdit.FixedPixelWidth)
        self.log.setReadOnly(True)
        self.log.setLineWrapColumnOrWidth(250)
        try:
            sys.stdout = Stream(newText=self.onUpdateText)
        except AttributeError as error:
            print(error)

    # creates data monitor
    def create_data_monitor(self):
        # make QTimer
        self.qTimer = QTimer(self)

        # label to title the data monitor
        self.sensor_label_title = QLabel("sensor_label", self.data_tab)
        self.sensor_label_title.move(l-360,10)
        self.sensor_label_title.setFrameShape(QFrame.Panel)
        self.sensor_label_title.setFrameShadow(QFrame.Sunken)
        self.sensor_label_title.setStyleSheet("background-color:rgb(53,53,53);")

        # label to hold the data
        self.sensor_label = QLabel("sensor_value", self.data_tab)
        self.sensor_label.move(l-340,25)
        self.sensor_label.resize(50,12)

        # set interval to update every 500 ms
        self.qTimer.setInterval(500)

        # connect timeout signal to signal handler
        self.qTimer.timeout.connect(self.get_sensor_value)

        self.qTimer.start()

        # dummy variable to demonstrate the monitor
        self.i = np.pi

    # gets the sensor's value
    def get_sensor_value(self):
        self.i += 0.000001

        self.curr_P.setText(str(fc.Kp))
        self.curr_I.setText(str(fc.Ki))
        self.curr_D.setText(str(fc.Kd))

        # get's the sensor value
        self.sensor_label.setText(str(self.i))

    # creates the plot graph button
    def create_plot_button(self):
        self.plot_button = QPushButton("Show Plot Graph", self.data_tab)
        self.plot_button.clicked.connect(self.create_plot)
        self.plot_button.move(0,20)
        self.plot_button.setStyleSheet("Black")
        # time stamp for the graph's delta time initialization
        self.timestamp = time.time()

    # function to start the timer for QTimer
    def start_timer(self):
        if self.timer.isActive():
            # sets the Y Range for the graph
            self.pw.setYRange(-1, 1)
            print("Graph is already updating at ", self.timer.interval(), " ms between data retrievals")
        else:
            self.timer.start()
            # sets the Y Range for the graph
            self.pw.setYRange(-1, 1)

    # create a live plot graph
    def create_plot(self):
        self.pw = pg.PlotWidget(self.data_tab)
        self.pw.showGrid(x=True,y=True)
        self.pw.setTitle('Demo uses sin function and Kp[0] values')
        self.pw.move(0,20)
        self.pw.resize(400,282)
        self.pw.show()
        self.pw.setLabel('left', 'insert_value')
        self.pw.setLabel('bottom', 'Time', units='s')
        self.pw.setAntialiasing(True)
        # sets the Y Range for the graph
        self.pw.setYRange(-1,1)
        self.timer = pg.QtCore.QTimer(self)

        self.stop_plot_button = QPushButton("Stop Graph Update", self.data_tab)
        self.stop_plot_button.clicked.connect(self.timer.stop)
        self.stop_plot_button.move(0, 0)
        self.stop_plot_button.show()

        self.start_plot_button = QPushButton("Start Graph Update", self.data_tab)
        self.start_plot_button.clicked.connect(self.start_timer)
        self.start_plot_button.move(290,0)
        self.start_plot_button.show()

        # buffer size for the data
        self.buffer = 100
        # queue to get the current delta time and values
        self.queue = Queue(self.buffer)
        # deque containing the values
        self.values_1 = deque([], maxlen=self.buffer)
        self.values_2 = deque([], maxlen=self.buffer)
        # deque containing the delta times
        self.times = deque([], maxlen=self.buffer)

        def update():
            # current delta time
            t = time.time() - self.timestamp

            # value(s) that we want to track
            v1 = np.sin(t)
            v2 = fc.Kp[0]

            # put the data into queue
            self.queue.put([t,v1,v2])

            # get the data from the queue. Will wait until item is available
            data = self.queue.get(True, None)

            # append data in each deque
            self.times.append(data[0])
            self.values_1.append(data[1])
            self.values_2.append(data[2])


            # draw the incoming data
            self.pw.clear()
            self.pw.plot(x=list(self.times)[-self.buffer:], y=list(self.values_1)[-self.buffer:],pen='r')
            self.pw.plot(x=list(self.times)[-self.buffer:], y=list(self.values_2)[-self.buffer:],pen='b')

        self.timer.timeout.connect(update)
        # length between updates (in ms)
        self.timer.start(2)

    # creates labels for the flight motion patterns
    def create_flight_motion_labels(self):
        self.flight_motion_label = QLabel(self.flight_pattern_tab)
        self.flight_motion_label.setFrameShape(QFrame.StyledPanel)
        self.flight_motion_label.setFrameShadow(QFrame.Raised)
        self.flight_motion_label.move(0,20)
        self.flight_motion_label.setText("Flight Motion Pattern")

        self.square_pattern = QPushButton("Square", self.flight_pattern_tab)
        self.square_pattern.move(0,35)
        self.square_pattern.clicked.connect(self.do_square_pattern)

    # conducts the square pattern
    def do_square_pattern(self):
        print("Completing Square Flight Motion Pattern...")
        print("[Currently Under Development]")


# a class to read the command line output stream
class Stream(QObject):
    newText = pyqtSignal(str)

    def write(self, text):
        self.newText.emit(str(text))

    def flush(self):
        pass


if __name__ == '__main__':
    import sys

    app = QApplication(sys.argv)
    gallery = DroneGUI()
    gallery.setGeometry(0, 0, l, w)
    gallery.show()
    print("Cornell Aerial Robotics 2019")
    print("Press 'q' to exit")
    sys.exit(app.exec_())
