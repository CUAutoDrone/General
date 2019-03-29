try:
    import time
    from collections import deque
    from multiprocessing import Queue
    import numpy as np
    import pyqtgraph as pg
    import paramiko

    from PyQt5.QtCore import QObject, pyqtSignal, QRect, QSize, Qt, QTimer, QTime, QEvent
    from PyQt5.QtGui import QFont, QPalette, QColor, QDoubleValidator, QTextCursor, QPixmap, QIcon, QWindow
    from PyQt5.QtWidgets import QApplication, QPushButton, QLabel, QDialog, QTabWidget, \
        QWidget, QSizePolicy, QLineEdit, QFrame, QTextEdit, qApp
    from flight_controller import *
except ImportError as e:
    print("Import Error has occurred. Make sure you have all the packages and modules installed.")
    print("Visit https://github.com/CornellAerialRobotics/General/blob/master/flight_controller/"
          "GUI/README.md for requirements")
    print()
    print(e)
    exit()


# version number of the GUI
version_number = "0.1.2"

# length of GUI
l = 960

# width of GUI
w = 640

# receiver created with receiver channels gpio pin numbers initialized
receiver = Receiver(17, 27, 22, 18, 23)
# IMU created with MPU6050 address, alpha initialized
imu = IMU(0x68, 0.98)
# motors created with motor gpio pin numbers initialized
motor = Motor(10, 9, 25, 8, 400.0)
# initialize the flight controller
fc = FlightController(np.array([0.1, 0.2, 0.3]), np.array([0.4, 0.5, 0.6]),
                       np.array([0.7, 0.8, 0.9]), receiver, imu, motor)

# TODO: need to receive information from RPi, not the current script
class DroneGUI(QDialog):
    global l, w

    def __init__(self, parent=None):
        super(DroneGUI, self).__init__(parent)

        # creates the terminal
        self.create_terminal()

        # create tabs
        self.create_tabs()

        # set the size policy
        self.set_size_policy()

        # palette background setting, style
        self.create_palette()

        # adds the arm, kill, unarm buttons
        # self.create_arm_kill_buttons()

        # adds the PID insert text boxes
        self.create_PID_insert()

        # adds the log
        self.create_log()

        # stream for the command line output to fill the log
        sys.stdout = Stream(newText=self.onUpdateText)

        # adds the data monitor
        self.create_data_monitor()

        # installs the event filter for 'space bar' and 'a'
        qApp.installEventFilter(self)

        # creates a button to deliver a live motor output plot graph
        self.motor_output_plot_graph_is_open = False
        self.create_motor_output_plot_button()

        # create the flight motion labels
        self.create_flight_motion_labels()

        # shows the current information of the drone
        self.drone_information()

        # add logo
        self.set_logo()

        # set up about tab
        self.show_about_info()

        # commands currently allowed in the terminal
        self.accepted_commands = {"connect", "arm", "kill", "forward",
                                  "altitude", "vehicle status", "pi status", "square"}

        # accepted IPs for the Raspberry Pi
        self.accepted_ips = {"192.168.1.1"}

        self.ssh_client = None


    def eventFilter(self, obj, event):
        if event.type() == QEvent.KeyPress:
            if event.key() == Qt.Key_Return:
                x = self.term.toPlainText().strip()
                split = x.split(' ')
                if split[0] in self.accepted_commands:
                        if split[0] == "connect":
                            if len(split) > 1:
                                if split[1] in self.accepted_ips:
                                    print("> Connecting RPi...")
                                    print("=> " + split[1])
                                    self.ssh_client = self.connect_to_pi(split[1], 22, 'admin')
                        elif split[0] == "arm":
                            print("> Arming...")
                            # self.executeCommand(self.ssh_client, 'python ./RPi_FC.py')
                self.term.clear()
        return super(DroneGUI, self).eventFilter(obj, event)


    # filters space bar to allow it to be a killswitch only
    # filters 'a' to allow it to arm the drone only
    #def eventFilter(self, obj, event):
    #     if event.type() == QEvent.KeyPress:
    #         if event.key() == Qt.Key_Space:
    #             if self.arm_button.isChecked():
    #                 print("KILL SWITCH ACTIVATED")
    #                 self.arm_button.toggle()
    #                 self.arm_button.setEnabled(False)
    #                 self.undo_killswitch_button.setEnabled(True)
    #                 self.undo_killswitch_button.setStyleSheet("background-color: yellow; color:black")
    #                 self.arm_button.setStyleSheet("background-color: Gray")
    #                 self.killswitch_button.setEnabled(False)
    #                 self.killswitch_button.setStyleSheet("background-color: Darkred; color:black")
    #                 self.curr_status.setStyleSheet("background-color: #922B3E;")
    #                 self.curr_status.setText("Inactive")
    #                 self.flight_timer.stop()
    #                 # fc.receiver.ARM = False
    #                 return True
    #             else:
    #                 return True
    #         if event.key() == Qt.Key_A:
    #             if not(self.arm_button.isChecked()):
    #                 print()
    #                 print("Initialize Arming Process...")
    #                 self.arm_button.setStyleSheet("background-color: Green")
    #                 self.undo_killswitch_button.setEnabled(False)
    #                 self.killswitch_button.setEnabled(True)
    #                 self.killswitch_button.setStyleSheet("background-color: red")
    #                 self.undo_killswitch_button.setStyleSheet("background-color:rgb(53,53,53);")
    #                 self.arm_button.setEnabled(False)
    #                 self.arm_button.setChecked(True)
    #                 self.curr_status.setStyleSheet("background-color: #507D2A")
    #                 self.curr_status.setText("Active")
    #                 self.flight_timer.start(1000)
    #                 # Since we're skipping the unlock arm button,
    #                 # fc.run()
    #                 return True
    #             else:
    #                 return True
    #     return super(DroneGUI, self).eventFilter(obj, event)

    # Connects to the IP using a port, username, and password via SSH.
    def connect_to_pi(self, ip, port, username, password="raspberry"):
        sshClient = paramiko.SSHClient()
        sshClient.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        sshClient.load_system_host_keys()
        try:
            sshClient.connect(ip, port, username, password, timeout=10)
        except Exception as e:
            print(e)
            sshClient.connect(ip + ".local", port, username, password)

        return sshClient

    # Takes in an sshClient and sends the command to it
    def executeCommand(sshClient, command):
        stdin, stdout, stderr = sshClient.exec_command(command)
        print("> COMMAND:", command)

    # creates a log for the command line output
    def create_terminal(self):
        self.term = QTextEdit(self)
        self.term.move(l-260, 20)
        self.term.setFixedSize(260,22)
        self.term.setFixedHeight(18)
        self.term.moveCursor(QTextCursor.Start)
        self.term.setReadOnly(False)
        self.term.setCursorWidth(4)

    # resizes the GUI
    def set_size_policy(self):
        self.sizePolicy = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.sizePolicy.setHorizontalStretch(0)
        self.sizePolicy.setVerticalStretch(0)
        self.sizePolicy.setHeightForWidth(self.sizePolicy.hasHeightForWidth())
        self.setSizePolicy(self.sizePolicy)
        self.setMaximumSize(l, w)
        self.setMinimumSize(l, w)

    # Creates the tabs
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
        self.about_tab = QWidget()
        self.tabWidget.addTab(self.settings_tab, "Settings")
        self.tabWidget.addTab(self.data_tab, "Data")
        self.tabWidget.addTab(self.flight_pattern_tab, "Flight Pattern")
        self.tabWidget.addTab(self.about_tab, "About")
        self.tabWidget.show()

    # Defines the palette colors
    def create_palette(self):
        self.setWindowTitle("Flight Controller")
        app.setStyle("Fusion")
        app.setFont(QFont("Helvetica"))
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

    # # create buttons to arm, and kill, and un-kill the drone
    # def create_arm_kill_buttons(self):
    #     # button to arm the drone
    #     self.arm_button = QPushButton('ARM', self)
    #     self.arm_button.setDefault(False)
    #     self.arm_button.setAutoDefault(False)
    #     self.arm_button.setStyleSheet("background-color: green")
    #     self.arm_button.move(0, 0)
    #     self.arm_button.resize(70, 43)
    #     self.arm_button.setFont(QFont("Helvetica", 17.5))
    #     self.arm_button.setCheckable(True)
    #     self.arm_button.setEnabled(True)
    #     self.arm_button.clicked.connect(self.arm_drone)
    #     self.arm_button.setShortcut("A")
    #
    #     # button to kill the drone
    #     self.killswitch_button = QPushButton('KILL SWITCH', self)
    #     self.killswitch_button.move(l-110, 0)
    #     self.killswitch_button.setDefault(False)
    #     self.killswitch_button.setAutoDefault(False)
    #     self.killswitch_button.setFont(QFont("Helvetica", 17.0))
    #     self.killswitch_button.resize(110, 60)
    #     self.killswitch_button.clicked.connect(self.kill_motor)
    #     self.killswitch_button.setStyleSheet("background-color: red")
    #     self.killswitch_button.setEnabled(False)
    #     self.killswitch_button.setShortcut("Space")
    #
    #     # button to undo kill switch
    #     self.undo_killswitch_button = QPushButton('Unlock ARM', self)
    #     self.undo_killswitch_button.setDefault(False)
    #     self.undo_killswitch_button.setAutoDefault(False)
    #     self.undo_killswitch_button.move(l-110, 62)
    #     self.undo_killswitch_button.setFont(QFont("Helvetica", 12))
    #     self.undo_killswitch_button.resize(75, 30)
    #     self.undo_killswitch_button.clicked.connect(self.undo_killswitch)
    #     self.undo_killswitch_button.setStyleSheet("background-color:rgb(53,53,53);")
    #     self.undo_killswitch_button.setEnabled(False)
    #
    #     # button to exit the application
    #     self.exit_app_button = QPushButton('exit', self)
    #     self.exit_app_button.resize(0,0)
    #     self.exit_app_button.setShortcut("Shift+Q")
    #     self.exit_app_button.clicked.connect(self.exit_application)


    # killswitch for the drone
    def kill_motor(self):
        if self.arm_button.isChecked():
            print("KILL SWITCH ACTIVATED")
            self.flight_timer.stop()
            self.arm_button.toggle()
            self.arm_button.setEnabled(False)
            self.undo_killswitch_button.setEnabled(True)
            self.undo_killswitch_button.setStyleSheet("background-color: yellow; color:black")
            self.killswitch_button.setEnabled(False)
            self.killswitch_button.setStyleSheet("background-color: Darkred; color:black")
            self.arm_button.setStyleSheet("background-color: Gray")
            self.curr_status.setStyleSheet("background-color: #922B3E;")
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
        print()
        print("Initialize Arming Process...")
        self.flight_timer.start(1000)
        self.undo_killswitch_button.setEnabled(False)
        self.killswitch_button.setEnabled(True)
        self.undo_killswitch_button.setStyleSheet("background-color:rgb(53,53,53);")
        self.arm_button.setEnabled(False)
        self.curr_status.setStyleSheet("background-color: #507D2A")
        self.curr_status.setText("Active")
        # fc.run()

    # displays current drone information
    def drone_information(self):
        # widget for the drone status section
        self.drone_status = QWidget(self.settings_tab)
        self.drone_status.resize(180,110)
        self.drone_status.move(92,200)

        self.state_label = QPushButton("Drone Status", self.drone_status)
        self.state_label.setCheckable(False)
        self.state_label.setEnabled(False)
        self.state_label.setStyleSheet("background: #333332; color: white")
        self.state_label.move(0, 0)
        self.state_label.show()
        self.state_label.resize(80,20)
        self.curr_status = QPushButton("Inactive", self.drone_status)
        self.curr_status.setIcon(QIcon("Images/Icons/drone.png"))
        self.curr_status.setDefault(False)
        self.curr_status.setEnabled(False)
        self.curr_status.setStyleSheet("background-color: #922B3E;")
        self.curr_status.move(81,0)
        self.curr_status.resize(70,20)
        self.curr_P = QLineEdit(self.drone_status)
        self.curr_P.setReadOnly(True)
        self.curr_P_label = QLabel(self.drone_status)
        self.curr_P_label.move(2, 25)
        self.curr_P_label.setText("Proportional:")
        self.curr_P_label.show()
        self.curr_P.setText(str(fc.Kp))  # " ", str(fc.Ki), " ", str(fc.Kd))
        self.curr_P.move(80, 25)
        self.curr_P.resize(90,12)
        self.curr_P.show()
        self.curr_I_label = QLabel(self.drone_status)
        self.curr_I_label.move(2, 40)
        self.curr_I_label.setText("       Integral:")
        self.curr_I_label.show()
        self.curr_I = QLineEdit(self.drone_status)
        self.curr_I.setReadOnly(True)
        self.curr_I.setText(str(fc.Ki))  # " ", str(fc.Ki), " ", str(fc.Kd))
        self.curr_I.move(80, 40)
        self.curr_I.resize(90,12)
        self.curr_I.show()
        self.curr_D_label = QLabel(self.drone_status)
        self.curr_D_label.move(2, 55)
        self.curr_D_label.setText("   Derivative:")
        self.curr_D_label.show()
        self.curr_D = QLineEdit(self.drone_status)
        self.curr_D.setReadOnly(True)
        self.curr_D.setText(str(fc.Kd))  # " ", str(fc.Ki), " ", str(fc.Kd))
        self.curr_D.move(80, 55)
        self.curr_D.resize(90, 12)
        self.curr_D.show()
        self.flight_time_label = QLabel("Flight Time:", self.drone_status)
        self.flight_time_label.move(5, 85)
        self.flight_time = QLineEdit(self.drone_status)
        self.flight_time.setReadOnly(True)
        self.flight_time.move(80, 80)
        self.flight_time.setFont(QFont("Helvetica", 15))
        self.flight_time.resize(50,18)
        self.flight_timer = QTimer()
        self.time = QTime(0,0)
        self.flight_timer.timeout.connect(self.update_timer)

        self.hard_wired_button = QPushButton("Hard Wired Inputs", self.settings_tab)
        self.hard_wired_button.move(l-365,2)
        self.hard_wired_button.setStyleSheet("background-color:#002366;")
        self.hard_wired_button.clicked.connect(self.show_hard_wire_connections)

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
            print(time.strftime("   %H:%M:%S %Z"))
            print("Update PID Gains")
            print("P:    ", fc.Kp[0], " ", fc.Kp[1], " ", fc.Kp[2])
            print("I :    ", fc.Ki[0], " ", fc.Ki[1], " ", fc.Ki[2])
            print("D:    ", fc.Kd[0], " ", fc.Kd[1], " ", fc.Kd[2])
            print()

    # creates the PID text boxes, button for updating PID values
    def create_PID_insert(self):

        # widget for PID insertion
        self.PID_widget = QWidget(self.settings_tab)
        self.PID_widget.resize(300,200)
        self.PID_widget.move(10,20)
        # proportional text
        self.onlyDouble = QDoubleValidator()
        self.Kp0_textbox = QLineEdit(self.PID_widget)
        self.Kp0_textbox.clearFocus()
        self.Kp0_textbox.setValidator(self.onlyDouble)
        self.Kp0_textbox.resize(50, 23)
        self.Kp0_textbox.setText(str(fc.Kp[0]))
        self.Kp1_textbox = QLineEdit(self.PID_widget)
        self.Kp1_textbox.setValidator(self.onlyDouble)
        self.Kp1_textbox.resize(50, 23)
        self.Kp1_textbox.move(50,0)
        self.Kp1_textbox.setText(str(fc.Kp[1]))
        self.Kp2_textbox = QLineEdit(self.PID_widget)
        self.Kp2_textbox.setValidator(self.onlyDouble)
        self.Kp2_textbox.resize(50, 23)
        self.Kp2_textbox.move(100, 0)
        self.Kp2_textbox.setText(str(fc.Kp[2]))
        # proportional label
        self.Kp_label = QLabel(self.PID_widget)
        self.Kp_label.setText('Proportional')
        self.Kp_label.move(150, 0)
        self.Kp_label.resize(85, 23)
        self.Kp_label.setFrameShape(QFrame.Panel)
        self.Kp_label.setFrameShadow(QFrame.Sunken)
        self.Kp_label.setLineWidth(3)
        self.Kp_label.setStyleSheet("background-color:rgb(53,53,53);")

        # integral text
        self.Ki0_textbox = QLineEdit(self.PID_widget)
        self.Ki0_textbox.move(0, 27)
        self.Ki0_textbox.resize(50, 23)
        self.Ki0_textbox.setValidator(self.onlyDouble)
        self.Ki0_textbox.setText(str(fc.Ki[0]))
        self.Ki1_textbox = QLineEdit(self.PID_widget)
        self.Ki1_textbox.move(50, 27)
        self.Ki1_textbox.resize(50, 23)
        self.Ki1_textbox.setValidator(self.onlyDouble)
        self.Ki1_textbox.setText(str(fc.Ki[1]))
        self.Ki2_textbox = QLineEdit(self.PID_widget)
        self.Ki2_textbox.move(100, 27)
        self.Ki2_textbox.resize(50, 23)
        self.Ki2_textbox.setValidator(self.onlyDouble)
        self.Ki2_textbox.setText(str(fc.Ki[2]))
        # integral label
        self.Ki_label = QLabel(self.PID_widget)
        self.Ki_label.setText('Integral')
        self.Ki_label.move(150, 27)
        self.Ki_label.resize(85, 23)
        self.Ki_label.setFrameShape(QFrame.Panel)
        self.Ki_label.setFrameShadow(QFrame.Sunken)
        self.Ki_label.setLineWidth(3)
        self.Ki_label.setStyleSheet("background-color:rgb(53,53,53);")

        # derivative text
        self.Kd0_textbox = QLineEdit(self.PID_widget)
        self.Kd0_textbox.move(0, 54)
        self.Kd0_textbox.resize(50, 23)
        self.Kd0_textbox.setValidator(self.onlyDouble)
        self.Kd0_textbox.setText(str(fc.Kd[0]))
        self.Kd1_textbox = QLineEdit(self.PID_widget)
        self.Kd1_textbox.move(50, 54)
        self.Kd1_textbox.resize(50, 23)
        self.Kd1_textbox.setValidator(self.onlyDouble)
        self.Kd1_textbox.setText(str(fc.Kd[1]))
        self.Kd2_textbox = QLineEdit(self.PID_widget)
        self.Kd2_textbox.move(100, 54)
        self.Kd2_textbox.resize(50, 23)
        self.Kd2_textbox.setValidator(self.onlyDouble)
        self.Kd2_textbox.setText(str(fc.Kd[2]))
        # derivative label
        self.Kd_label = QLabel(self.PID_widget)
        self.Kd_label.resize(85, 23)
        self.Kd_label.setText('Derivative')
        self.Kd_label.move(150, 54)
        self.Kd_label.setFrameShape(QFrame.Panel)
        self.Kd_label.setFrameShadow(QFrame.Sunken)
        self.Kd_label.setLineWidth(3)
        self.Kd_label.setStyleSheet("background-color:rgb(53,53,53);")

        # button to insert new PID values
        self.insert_PID_values = QPushButton("Insert PID Gains", self.PID_widget)
        self.insert_PID_values.setStyleSheet("background-color:	#002366;")
        self.insert_PID_values.move(150, 80)
        self.insert_PID_values.resize(85, 25)
        self.insert_PID_values.setFont(QFont("Helvetica", 11.5))
        self.insert_PID_values.setCheckable(True)
        self.insert_PID_values.setEnabled(True)
        self.insert_PID_values.clicked.connect(self.get_PID_value)

        # label for Roll, Pitch, Yaw
        self.RPY = QLabel(self.PID_widget)
        self.RPY.move(0,80)
        self.RPY.setText(' Roll        Pitch         Yaw  ')
        self.RPY.setFrameShape(QFrame.Panel)
        self.RPY.setFrameShadow(QFrame.Sunken)
        self.RPY.setLineWidth(3)
        self.RPY.setStyleSheet("background-color:rgb(53,53,53);")

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
        self.log.move(l-260, w-465)
        self.log.resize(260, w-175)
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


        # pi connection status
        self.pi_connection_status_is_clicked = False
        self.pi_connection_label = QPushButton("Pi Connection", self)
        self.pi_connection_label.setCheckable(False)
        self.pi_connection_label.setEnabled(False)
        self.pi_connection_label.setStyleSheet("background: #333332; color: white")
        self.pi_connection_label.move(2, 96)
        self.pi_connection_label.show()
        self.pi_connection_label.resize(80, 20)
        self.pi_connection_status = QPushButton("Offline", self)
        self.pi_connection_status.move(2,115)
        self.pi_connection_status.setDefault(False)
        self.pi_connection_status.setEnabled(False)
        self.pi_connection_status.setIcon(QIcon('Images/Icons/connection.png'))
        self.pi_connection_status.setStyleSheet("background-color: #922B3E;")

        # set interval to update
        self.qTimer.setInterval(250)

        # connect timeout signal to signal handler
        self.qTimer.timeout.connect(self.get_sensor_value)

        self.qTimer.start()

    # creates the plot graph button
    def create_motor_output_plot_button(self):
        self.plot_button = QPushButton("Motor Output Plot Graph", self.data_tab)
        self.plot_button.setStyleSheet("background-color:#002366;")
        self.plot_button.setIcon(QIcon("Images/Icons/line_graph.png"))
        self.plot_button.clicked.connect(self.create_motor_output_plot)
        self.plot_button.move(2,20)
        # time stamp for the graph's delta time initialization
        self.timestamp = time.time()

    # function to start the timer for QTimer
    def start_timer(self):
        if self.timer.isActive():
            # sets the Y Range for the graph
            self.pw.setYRange(-2, 2)
            print("Graph is already updating at ", self.timer.interval(), " ms between data retrievals")
        else:
            self.timer.start()
            # sets the Y Range for the graph
            self.pw.setYRange(-2, 2)

    # create a live motor output plot graph
    def create_motor_output_plot(self):
        self.motor_output_plot_graph_is_open = True
        # widget for the motor output graph
        self.motor_output_graph_widget = QWidget(self.data_tab)
        self.motor_output_graph_widget.resize(l - 260, w - 100)
        self.motor_output_graph_widget.show()
        # motor 1 output title
        self.motor_1_output_title = QPushButton("Motor 1 Output", self.motor_output_graph_widget)
        self.motor_1_output_title.move(l - 400, 60)
        self.motor_1_output_title.setStyleSheet("background: black; color: gray;")
        self.motor_1_output_title.setEnabled(False)
        self.motor_1_output_title.setCheckable(False)
        self.motor_1_output_title.setFixedSize(90, 13)
        self.motor_1_output_title.setFont(QFont("Helvetica", 11.5))
        self.motor_1_output_title.show()

        self.motor_1_output_title_color = QLabel(self.motor_output_graph_widget)
        self.motor_1_output_title_color.setPixmap(QPixmap('Images/Icons/red_square.png'))
        self.motor_1_output_title_color.move(l - 413, 60)
        self.motor_1_output_title_color.show()

        # motor 2 output title
        self.motor_2_output_title = QPushButton("Motor 2 Output", self.motor_output_graph_widget)
        self.motor_2_output_title_color = QLabel(self.motor_output_graph_widget)
        self.motor_2_output_title_color.setPixmap(QPixmap('Images/Icons/blue_square.png'))
        self.motor_2_output_title_color.move(l - 413, 90)
        self.motor_2_output_title_color.show()
        self.motor_2_output_title.move(l - 400, 90)
        self.motor_2_output_title.setStyleSheet("background: black; color: gray;")
        self.motor_2_output_title.setEnabled(False)
        self.motor_2_output_title.setCheckable(False)
        self.motor_2_output_title.setFixedSize(90, 13)
        self.motor_2_output_title.setFont(QFont("Helvetica", 11.5))
        self.motor_2_output_title.show()

        # motor 3 output title
        self.motor_3_output_title = QPushButton("Motor 3 Output", self.motor_output_graph_widget)
        self.motor_3_output_title_color = QLabel(self.motor_output_graph_widget)
        self.motor_3_output_title_color.setPixmap(QPixmap('Images/Icons/yellow_square.png'))
        self.motor_3_output_title_color.move(l - 413, 120)
        self.motor_3_output_title_color.show()
        self.motor_3_output_title.move(l - 400, 120)
        self.motor_3_output_title.setStyleSheet("background: black; color: gray;")
        self.motor_3_output_title.setEnabled(False)
        self.motor_3_output_title.setCheckable(False)
        self.motor_3_output_title.setFixedSize(90, 13)
        self.motor_3_output_title.setFont(QFont("Helvetica", 11.5))
        self.motor_3_output_title.show()

        # motor 4 output title
        self.motor_4_output_title = QPushButton("Motor 4 Output", self.motor_output_graph_widget)
        self.motor_4_output_title_color = QLabel(self.motor_output_graph_widget)
        self.motor_4_output_title_color.setPixmap(QPixmap('Images/Icons/purple_square.png'))
        self.motor_4_output_title_color.move(l - 413, 150)
        self.motor_4_output_title_color.show()
        self.motor_4_output_title.move(l - 400, 150)
        self.motor_4_output_title.setStyleSheet("background: black; color: gray;")
        self.motor_4_output_title.setEnabled(False)
        self.motor_4_output_title.setCheckable(False)
        self.motor_4_output_title.setFixedSize(90, 13)
        self.motor_4_output_title.setFont(QFont("Helvetica", 11.5))
        self.motor_4_output_title.show()

        # motor 1 output
        self.motor_1_output = QLabel(str(fc.motor_output[0]), self.motor_output_graph_widget)
        self.motor_1_output.move(l - 395, 75)
        self.motor_1_output.resize(70, 12)
        self.motor_1_output.show()

        # motor 2 output
        self.motor_2_output = QLabel(str(fc.motor_output[1]), self.motor_output_graph_widget)
        self.motor_2_output.move(l - 395, 105)
        self.motor_2_output.resize(70, 12)
        self.motor_2_output.show()

        # motor 3 output
        self.motor_3_output = QLabel(str(fc.motor_output[2]), self.motor_output_graph_widget)
        self.motor_3_output.move(l - 395, 135)
        self.motor_3_output.resize(70, 12)
        self.motor_3_output.show()

        # motor 4 output
        self.motor_4_output = QLabel(str(fc.motor_output[3]), self.motor_output_graph_widget)
        self.motor_4_output.move(l - 395, 165)
        self.motor_4_output.resize(70, 12)
        self.motor_4_output.show()


        self.pw = pg.PlotWidget(self.motor_output_graph_widget)
        self.pw.showGrid(x=True,y=True)
        self.pw.setTitle('Live Update Graph (demonstration with sin function)')
        self.pw.move(0,20)
        self.pw.resize(l/2,w/2)
        self.pw.show()
        self.pw.setLabel('left', 'Motor Output')
        self.pw.setLabel('bottom', 'Time', units='s')
        self.pw.setAntialiasing(True)
        # sets the Y Range for the graph
        self.pw.setYRange(-2,2)
        self.timer = pg.QtCore.QTimer(self)

        self.stop_plot_button = QPushButton("Pause", self.motor_output_graph_widget)
        self.stop_plot_button.setStyleSheet("background-color:#002366;")
        self.stop_plot_button.setIcon(QIcon('Images/Icons/stop.png'))
        self.stop_plot_button.clicked.connect(self.timer.stop)
        self.stop_plot_button.resize(80,20)
        self.stop_plot_button.move(20, 0)
        self.stop_plot_button.show()

        self.start_plot_button = QPushButton("Start", self.motor_output_graph_widget)
        self.start_plot_button.setIcon(QIcon('Images/Icons/play.png'))
        self.start_plot_button.resize(80,20)
        self.start_plot_button.setStyleSheet("background-color:#002366;")
        self.start_plot_button.clicked.connect(self.start_timer)
        self.start_plot_button.move(l/2-110,0)
        self.start_plot_button.show()

        # buffer size for the data
        self.buffer = 200
        # queue to get the current delta time and values
        self.queue = Queue(self.buffer)
        # deque containing the values
        self.values_1 = deque([], maxlen=self.buffer)
        self.values_2 = deque([], maxlen=self.buffer)
        self.values_3 = deque([], maxlen = self.buffer)
        self.values_4 = deque([], maxlen=self.buffer)
        self.values_5 = deque([], maxlen=self.buffer)

        # deque containing the delta times
        self.times = deque([], maxlen=self.buffer)

        def update():
            # current delta time
            t = time.time() - self.timestamp

            # value(s) that we want to track
            v1 = fc.motor_output[0]
            v2 = fc.motor_output[1]
            v3 = fc.motor_output[2]
            v4 = fc.motor_output[3]
            # demonstration
            v5 = np.sin(t)


            # put the data into queue
            self.queue.put([t,v1,v2,v3,v4,v5])

            # get the data from the queue. Will wait until item is available
            data = self.queue.get(True, None)

            # append data in each deque
            self.times.append(data[0])
            self.values_1.append(data[1])
            self.values_2.append(data[2])
            self.values_3.append(data[3])
            self.values_4.append(data[4])
            self.values_5.append(data[5])


            # draw the incoming data
            self.pw.clear()
            self.pw.plot(x=list(self.times)[-self.buffer:], y=list(self.values_1)[-self.buffer:],pen='r')
            self.pw.plot(x=list(self.times)[-self.buffer:], y=list(self.values_2)[-self.buffer:],pen='b')
            self.pw.plot(x=list(self.times)[-self.buffer:], y=list(self.values_3)[-self.buffer:],pen='y')
            self.pw.plot(x=list(self.times)[-self.buffer:], y=list(self.values_4)[-self.buffer:], pen='m')
            self.pw.plot(x=list(self.times)[-self.buffer:], y=list(self.values_5)[-self.buffer:], pen='w')

        self.timer.timeout.connect(update)
        # length between updates (in ms)
        self.timer.start(2)

        # gets the sensor's value

    def get_sensor_value(self):

        # get's the motor output values
        if self.motor_output_plot_graph_is_open is True:
            self.motor_1_output.setText(str(fc.motor_output[0]))
            self.motor_2_output.setText(str(fc.motor_output[1]))
            self.motor_3_output.setText(str(fc.motor_output[2]))
            self.motor_4_output.setText(str(fc.motor_output[3]))

        # determines the status of the pi connection
        if fc.pi_online:
            self.pi_connection_status.setStyleSheet("background-color: #507D2A")
            self.pi_connection_status.setText("Online")
            self.pi_connection_status_is_clicked = True
        elif not fc.pi_online and self.pi_connection_status_is_clicked:
            self.pi_connection_status.setStyleSheet("background-color: #922B3E")
            self.pi_connection_status.setText("Offline")
            self.pi_connection_status_is_clicked = False

        # gets the current PID
        self.curr_P.setText(str(fc.Kp))
        self.curr_I.setText(str(fc.Ki))
        self.curr_D.setText(str(fc.Kd))

    # creates labels for the flight motion patterns
    def create_flight_motion_labels(self):
        # Widget for the flight pattern buttons
        self.flight_pattern_widget = QWidget(self.flight_pattern_tab)
        self.flight_pattern_widget.resize(200,200)

        self.flight_motion_label = QLabel(self.flight_pattern_widget)
        self.flight_motion_label.setFrameShape(QFrame.StyledPanel)
        self.flight_motion_label.setFrameShadow(QFrame.Raised)
        self.flight_motion_label.move(2,20)
        self.flight_motion_label.setText("Flight Motion Pattern")

        self.square_pattern = QPushButton("Square", self.flight_pattern_widget)
        self.square_pattern.setIcon(QIcon('Images/Icons/square.png'))
        self.square_pattern.move(2,35)
        self.square_pattern.clicked.connect(self.do_square_pattern)
        self.square_pattern.setStyleSheet("background-color:#002366;")

    # conducts the square pattern
    def do_square_pattern(self):
        print("Completing Square Flight Motion Pattern...")
        print("[Currently Under Development]")

    # exits the application
    def exit_application(self):
        DroneGUI.close(self)

    # updates the flight timer
    def update_timer(self):
        self.time = self.time.addSecs(1)
        self.flight_time.setText(self.time.toString("mm:ss"))

    # sets the logos for ARC and raspberry pi
    def set_logo(self):
        self.arc_logo = QLabel(self)
        self.arc_logo.setPixmap(QPixmap('Images/CUARClogo.png'))
        self.arc_logo.move(l-600,0)

        self.pi_logo = QLabel(self)
        self.pi_logo.setPixmap(QPixmap('Images/Icons/raspberry_pi_logo.png'))
        self.pi_logo.move(87,105)

    # shows pop up for the hard wire connections
    def show_hard_wire_connections(self):
        self.window = QDialog(self.settings_tab)
        self.window.setWindowTitle("Settings")
        self.window.resize(145,405)
        self.window.move(l - 408, 247)
        self.window.show()

        self.gpio_label = QPushButton("GPIO Reference", self.window)
        self.gpio_label.setCheckable(False)
        self.gpio_label.setEnabled(False)
        self.gpio_label.move(24,1)
        self.gpio_label.show()
        self.gpio_label.setStyleSheet("background: #333332; color: white")

        self.motor_1_label_pic = QLabel(self.window)
        self.motor_1_label_pic.setPixmap((QPixmap('Images/Icons/gear.png')))
        self.motor_1_label_pic.move(1,28)
        self.motor_1_label_pic.show()
        self.motor_1_label = QLabel("Motor 1:", self.window)
        self.motor_1_label.move(18,30)
        self.motor_1_label.show()

        self.motor_1_gpio = QLineEdit(self.window)
        self.motor_1_gpio.setReadOnly(True)
        self.motor_1_gpio.move(90,28)
        self.motor_1_gpio.resize(30,12)
        self.motor_1_gpio.show()
        self.motor_1_gpio.setText(str(fc.motor.MOTOR1))

        self.motor_2_label_pic = QLabel(self.window)
        self.motor_2_label_pic.setPixmap((QPixmap('Images/Icons/gear.png')))
        self.motor_2_label_pic.move(1, 48)
        self.motor_2_label_pic.show()
        self.motor_2_label = QLabel("Motor 2:", self.window)
        self.motor_2_label.move(18, 50)
        self.motor_2_label.show()

        self.motor_2_gpio = QLineEdit(self.window)
        self.motor_2_gpio.setReadOnly(True)
        self.motor_2_gpio.move(90, 48)
        self.motor_2_gpio.resize(30, 12)
        self.motor_2_gpio.show()
        self.motor_2_gpio.setText(str(fc.motor.MOTOR2))

        self.motor_3_label_pic = QLabel(self.window)
        self.motor_3_label_pic.setPixmap((QPixmap('Images/Icons/gear.png')))
        self.motor_3_label_pic.move(1, 68)
        self.motor_3_label_pic.show()
        self.motor_3_label = QLabel("Motor 3:", self.window)
        self.motor_3_label.move(18, 70)
        self.motor_3_label.show()

        self.motor_3_gpio = QLineEdit(self.window)
        self.motor_3_gpio.setReadOnly(True)
        self.motor_3_gpio.move(90, 68)
        self.motor_3_gpio.resize(30, 12)
        self.motor_3_gpio.show()
        self.motor_3_gpio.setText(str(fc.motor.MOTOR3))

        self.motor_4_label_pic = QLabel(self.window)
        self.motor_4_label_pic.setPixmap((QPixmap('Images/Icons/gear.png')))
        self.motor_4_label_pic.move(1, 88)
        self.motor_4_label_pic.show()
        self.motor_4_label = QLabel("Motor 4:", self.window)
        self.motor_4_label.move(18, 90)
        self.motor_4_label.show()

        self.motor_4_gpio = QLineEdit(self.window)
        self.motor_4_gpio.setReadOnly(True)
        self.motor_4_gpio.move(90, 88)
        self.motor_4_gpio.resize(30, 12)
        self.motor_4_gpio.show()
        self.motor_4_gpio.setText(str(fc.motor.MOTOR4))

        self.receiver_1_label = QLabel("Receiver 1:", self.window)
        self.receiver_1_label.move(18, 110)
        self.receiver_1_label.show()
        self.receiver_1_label_pic = QLabel(self.window)
        self.receiver_1_label_pic.setPixmap((QPixmap("Images/Icons/receiver.png")))
        self.receiver_1_label_pic.move(1, 107)
        self.receiver_1_label_pic.show()

        self.receiver_1_gpio = QLineEdit(self.window)
        self.receiver_1_gpio.setReadOnly(True)
        self.receiver_1_gpio.move(90,108)
        self.receiver_1_gpio.resize(30,12)
        self.receiver_1_gpio.show()
        self.receiver_1_gpio.setText(str(fc.receiver.RECEIVER_CH1))

        self.receiver_2_label = QLabel("Receiver 2:", self.window)
        self.receiver_2_label.move(18, 130)
        self.receiver_2_label.show()
        self.receiver_2_label_pic = QLabel(self.window)
        self.receiver_2_label_pic.setPixmap((QPixmap("Images/Icons/receiver.png")))
        self.receiver_2_label_pic.move(1, 127)
        self.receiver_2_label_pic.show()

        self.receiver_2_gpio = QLineEdit(self.window)
        self.receiver_2_gpio.setReadOnly(True)
        self.receiver_2_gpio.move(90, 128)
        self.receiver_2_gpio.resize(30, 12)
        self.receiver_2_gpio.show()
        self.receiver_2_gpio.setText(str(fc.receiver.RECEIVER_CH2))

        self.receiver_3_label = QLabel("Receiver 3:", self.window)
        self.receiver_3_label.move(18, 150)
        self.receiver_3_label.show()
        self.receiver_3_label_pic = QLabel(self.window)
        self.receiver_3_label_pic.setPixmap((QPixmap("Images/Icons/receiver.png")))
        self.receiver_3_label_pic.move(1, 147)
        self.receiver_3_label_pic.show()

        self.receiver_3_gpio = QLineEdit(self.window)
        self.receiver_3_gpio.setReadOnly(True)
        self.receiver_3_gpio.move(90, 148)
        self.receiver_3_gpio.resize(30, 12)
        self.receiver_3_gpio.show()
        self.receiver_3_gpio.setText(str(fc.receiver.RECEIVER_CH3))

        self.receiver_4_label = QLabel("Receiver 4:", self.window)
        self.receiver_4_label.move(18, 170)
        self.receiver_4_label.show()
        self.receiver_4_label_pic = QLabel(self.window)
        self.receiver_4_label_pic.setPixmap((QPixmap("Images/Icons/receiver.png")))
        self.receiver_4_label_pic.move(1, 167)
        self.receiver_4_label_pic.show()

        self.receiver_4_gpio = QLineEdit(self.window)
        self.receiver_4_gpio.setReadOnly(True)
        self.receiver_4_gpio.move(90, 168)
        self.receiver_4_gpio.resize(30, 12)
        self.receiver_4_gpio.show()
        self.receiver_4_gpio.setText(str(fc.receiver.RECEIVER_CH4))

        self.receiver_5_label = QLabel("Receiver 5:", self.window)
        self.receiver_5_label.move(18, 190)
        self.receiver_5_label.show()
        self.receiver_5_label_pic = QLabel(self.window)
        self.receiver_5_label_pic.setPixmap((QPixmap("Images/Icons/receiver.png")))
        self.receiver_5_label_pic.move(1, 187)
        self.receiver_5_label_pic.show()

        self.receiver_5_gpio = QLineEdit(self.window)
        self.receiver_5_gpio.setReadOnly(True)
        self.receiver_5_gpio.move(90, 188)
        self.receiver_5_gpio.resize(30, 12)
        self.receiver_5_gpio.show()
        self.receiver_5_gpio.setText(str(fc.receiver.RECEIVER_CH5))

        self.gpio_label = QPushButton("PWM Frequency (Hz)", self.window)
        self.gpio_label.setCheckable(False)
        self.gpio_label.setEnabled(False)
        self.gpio_label.move(2, 210)
        self.gpio_label.show()
        self.gpio_label.setStyleSheet("background: #333332; color: white")

        self.hz_pic = QLabel(self.window)
        self.hz_pic.setPixmap(QPixmap("Images/Icons/freq.png"))
        self.hz_pic.move(70,234)
        self.hz_pic.show()
        self.hz_count = QLineEdit(self.window)
        self.hz_count.setReadOnly(True)
        self.hz_count.move(84, 235)
        self.hz_count.resize(40, 12)
        self.hz_count.show()
        self.hz_count.setText(str(fc.motor.PWM_frequency))

        self.gpio_label = QPushButton("MPU6050 Address", self.window)
        self.gpio_label.setCheckable(False)
        self.gpio_label.setEnabled(False)
        self.gpio_label.move(18, 350)
        self.gpio_label.show()
        self.gpio_label.setStyleSheet("background: #333332; color: white")

        self.imu_pic = QLabel(self.window)
        self.imu_pic.setPixmap(QPixmap("Images/Icons/imu.png"))
        self.imu_pic.move(81, 374)
        self.imu_pic.show()

        self.mpu_address = QLineEdit(self.window)
        self.mpu_address.setReadOnly(True)
        self.mpu_address.move(95, 374)
        self.mpu_address.resize(30, 12)
        self.mpu_address.show()
        self.mpu_address.setText(str(fc.imu.mpu6050_handle))

        # sample time
        self.sample_time_label = QPushButton("Sample Time (ms)", self.window)
        self.sample_time_label.setCheckable(False)
        self.sample_time_label.setEnabled(False)
        self.sample_time_label.move(20, 300)
        self.sample_time_label.show()
        self.sample_time = QLineEdit(self.window)
        self.sample_time.setReadOnly(True)
        self.sample_time.resize(50, 12)
        self.sample_time.move(75, 324)
        self.sample_time.show()
        self.sample_time.setText(str(fc.imu.sample_time))
        self.sample_time_label.setStyleSheet("background: #333332; color: white")
        self.sample_time_pic = QLabel(self.window)
        self.sample_time_pic.setPixmap(QPixmap("Images/Icons/clock.png"))
        self.sample_time_pic.move(58, 323)
        self.sample_time_pic.show()

        # PID input/output limitations
        self.io_lim_label = QPushButton("PID Output Limits", self.window)
        self.io_lim_label.setCheckable(False)
        self.io_lim_label.setEnabled(False)
        self.io_lim_label.move(23, 258)
        self.io_lim_label.show()

        self.io_lim_lower = QLineEdit(self.window)
        self.io_lim_lower.setReadOnly(True)
        self.io_lim_lower.resize(38,12)
        self.io_lim_lower.move(24, 282)
        self.io_lim_lower.show()
        self.io_lim_lower.setText(str(fc.imu.output_min))
        self.io_lim_pic_dwn = QLabel(self.window)
        self.io_lim_pic_dwn.setPixmap(QPixmap("Images/Icons/down_arrow.png"))
        self.io_lim_pic_dwn.move(9, 280)
        self.io_lim_pic_dwn.show()

        self.io_lim_hi = QLineEdit(self.window)
        self.io_lim_hi.setReadOnly(True)
        self.io_lim_hi.resize(38, 12)
        self.io_lim_hi.move(87, 282)
        self.io_lim_hi.show()
        self.io_lim_hi.setText(str(fc.imu.output_max))
        self.io_lim_pic_up = QLabel(self.window)
        self.io_lim_pic_up.setPixmap(QPixmap("Images/Icons/up_arrow.png"))
        self.io_lim_pic_up.move(72, 280)
        self.io_lim_pic_up.show()

    def show_about_info(self):
        self.about_websites = QWidget(self.about_tab)
        self.about_websites.move(10,10)
        self.about_arc_label = QPushButton("ARC", self.about_websites)
        self.about_arc_label.setCheckable(False)
        self.about_arc_label.setEnabled(False)
        self.about_arc_label.move(0, 10)
        self.about_arc_label.show()
        self.arc_website = QLabel(self.about_websites)
        self.arc_website.move(2,36)
        self.arc_website.setText('<a href="http://CUAerialRobotics.github.io/">Aerial Robotics Team</a>')
        self.arc_website.setOpenExternalLinks(True)
        self.arc_website.show()
        self.arc_img = QLabel(self.about_websites)
        self.arc_img.setPixmap(QPixmap('Images/CUARC_about_logo.png'))
        self.arc_img.move(130,0)
        self.arc_img.show()

        self.about_git_label = QPushButton("GitHub", self.about_websites)
        self.about_git_label.setCheckable(False)
        self.about_git_label.setEnabled(False)
        self.about_git_label.move(0, 100)
        self.about_git_label.show()
        self.arc_website_git = QLabel(self.about_websites)
        self.arc_website_git.move(2, 126)
        self.arc_website_git.setText(
            '<a href="https://github.com/CornellAerialRobotics/">GitHub</a>')
        self.arc_website_git.setOpenExternalLinks(True)
        self.arc_website_git.show()
        self.git_img = QLabel(self.about_websites)
        self.git_img.setPixmap(QPixmap('Images/GitHub_logo.png'))
        self.git_img.move(130, 82)
        self.git_img.show()

        self.about_comp_label = QPushButton("Competition", self.about_websites)
        self.about_comp_label.setCheckable(False)
        self.about_comp_label.setEnabled(False)
        self.about_comp_label.move(0, 180)
        self.about_comp_label.show()
        self.comp_website = QLabel(self.about_websites)
        self.comp_website.move(2, 204)
        self.comp_website.setText(
            '<a href="http://www.aerialroboticscompetition.org/">IARC</a>')
        self.comp_website.setOpenExternalLinks(True)
        self.comp_website.show()
        self.comp_website_rules = QLabel(self.about_websites)
        self.comp_website_rules.move(2, 224)
        self.comp_website_rules.setText(
            '<a href="http://www.aerialroboticscompetition.org/rules.php">Mission 8 Rules</a>')
        self.comp_website_rules.setOpenExternalLinks(True)
        self.comp_website_rules.show()
        self.iarc_img = QLabel(self.about_websites)
        self.iarc_img.setPixmap(QPixmap('Images/IARC_logo.png'))
        self.iarc_img.move(130, 174)
        self.iarc_img.show()

        self.about_school_label = QPushButton("School", self.about_websites)
        self.about_school_label.setCheckable(False)
        self.about_school_label.setEnabled(False)
        self.about_school_label.move(0, 274)
        self.about_school_label.show()
        self.school_website = QLabel(self.about_websites)
        self.school_website.move(2, 298)
        self.school_website.setText(
            '<a href="https://www.cornell.edu/">Cornell University</a>')
        self.school_website.setOpenExternalLinks(True)
        self.school_website.show()
        self.cu_img = QLabel(self.about_websites)
        self.cu_img.setPixmap(QPixmap('Images/CU_logo.png'))
        self.cu_img.move(130, 260)
        self.cu_img.show()


# a class to read the command line output stream
class Stream(QObject):
    newText = pyqtSignal(str)

    def write(self, text):
        self.newText.emit(str(text))

    def flush(self):
        pass


def welcome_message():
    print("Cornell University Aerial Robotics 2019")
    print("                                       Version: ", version_number)


if __name__ == '__main__':
    import sys

    app = QApplication(sys.argv)
    gallery = DroneGUI()
    gallery.setGeometry(0, 0, l, w)
    gallery.show()
    welcome_message()
    sys.exit(app.exec_())
