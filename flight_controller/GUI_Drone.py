from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import datetime
from flight_controller import *
import numpy as np
import pyqtgraph as pg


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
# Finish graph
# potential sensor for 'wm' variable in FlightController.py
# buttons to have drone perform certain maneuvers (need further guidance)
#


class DroneGUI(QDialog):
    def __init__(self, parent=None):
        super(DroneGUI, self).__init__(parent)

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


    # filters space bar to allow it to be a killswitch only
    # filters 'a' to allow it to arm the drone only
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
                    #fc.receiver.ARM = 0
                    #fc.receiver.can_arm()
                    return True
                else:
                    return True
            if event.key() == Qt.Key_A:
                if not(self.arm_button.isChecked()):
                    print("ARMING")
                    self.arm_button.setStyleSheet("background-color: Green")
                    self.undo_killswitch_button.setEnabled(False)
                    self.killswitch_button.setEnabled(True)
                    self.killswitch_button.setStyleSheet("background-color: red")
                    self.undo_killswitch_button.setStyleSheet("background-color:rgb(53,53,53);")
                    self.arm_button.setEnabled(False)
                    self.arm_button.setChecked(True)
                    # fc.run()
                    return True
                else:
                    return True


        return super(DroneGUI, self).eventFilter(obj, event)


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
        self.arm_button.move(0, 150)
        self.arm_button.resize(70, 43)
        self.arm_button.setFont(QFont("Helvetica", 17.5))
        self.arm_button.setCheckable(True)
        self.arm_button.setEnabled(True)
        self.arm_button.clicked.connect(self.arm_drone)

        # button to kill the drone
        self.killswitch_button = QPushButton('KILLSWITCH', self)
        self.killswitch_button.move(690, 0)
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
        self.undo_killswitch_button.move(690, 62)
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
            #fc.receiver.ARM = 0
            #fc.receiver.can_arm()

    # allows the drone to be armed again
    def undo_killswitch(self):
        print("ARM button unlocked")
        self.arm_button.setEnabled(True)
        self.undo_killswitch_button.setEnabled(False)
        self.killswitch_button.setStyleSheet("background-color: red")
        self.undo_killswitch_button.setStyleSheet("background-color: Gold")
        self.arm_button.setStyleSheet("background-color: Green")
        #fc.receiver.ARM = 1

    # arms the drone
    def arm_drone(self):
        print("ARMING")
        self.undo_killswitch_button.setEnabled(False)
        self.killswitch_button.setEnabled(True)
        self.undo_killswitch_button.setStyleSheet("background-color:rgb(53,53,53);")
        self.arm_button.setEnabled(False)
        # fc.run()

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

    # creates the PID text boxes, button for updating PID values
    def create_PID_insert(self):

        # proportional text
        self.onlyDouble = QDoubleValidator()
        self.Kp0_textbox = QLineEdit(self)
        self.Kp0_textbox.clearFocus()
        self.Kp0_textbox.setValidator(self.onlyDouble)
        self.Kp0_textbox.resize(50, 23)
        self.Kp0_textbox.setText(str(fc.Kp[0]))
        self.Kp1_textbox = QLineEdit(self)
        self.Kp1_textbox.setValidator(self.onlyDouble)
        self.Kp1_textbox.resize(50, 23)
        self.Kp1_textbox.move(50,0)
        self.Kp1_textbox.setText(str(fc.Kp[1]))
        self.Kp2_textbox = QLineEdit(self)
        self.Kp2_textbox.setValidator(self.onlyDouble)
        self.Kp2_textbox.resize(50, 23)
        self.Kp2_textbox.move(100, 0)
        self.Kp2_textbox.setText(str(fc.Kp[2]))
        # proportional label
        self.Kp_label = QLabel(self)
        self.Kp_label.setText('Proportional')
        self.Kp_label.move(150, 0)
        self.Kp_label.resize(85, 23)
        self.Kp_label.setFrameShape(QFrame.Panel)
        self.Kp_label.setFrameShadow(QFrame.Sunken)
        self.Kp_label.setLineWidth(3)
        self.Kp_label.setStyleSheet("background-color:rgb(53,53,53);")

        # integral text
        self.Ki0_textbox = QLineEdit(self)
        self.Ki0_textbox.move(0, 27)
        self.Ki0_textbox.resize(50, 23)
        self.Ki0_textbox.setValidator(self.onlyDouble)
        self.Ki0_textbox.setText(str(fc.Ki[0]))
        self.Ki1_textbox = QLineEdit(self)
        self.Ki1_textbox.move(50, 27)
        self.Ki1_textbox.resize(50, 23)
        self.Ki1_textbox.setValidator(self.onlyDouble)
        self.Ki1_textbox.setText(str(fc.Ki[1]))
        self.Ki2_textbox = QLineEdit(self)
        self.Ki2_textbox.move(100, 27)
        self.Ki2_textbox.resize(50, 23)
        self.Ki2_textbox.setValidator(self.onlyDouble)
        self.Ki2_textbox.setText(str(fc.Ki[2]))
        # integral label
        self.Ki_label = QLabel(self)
        self.Ki_label.setText('Integral')
        self.Ki_label.move(150, 27)
        self.Ki_label.resize(85, 23)
        self.Ki_label.setFrameShape(QFrame.Panel)
        self.Ki_label.setFrameShadow(QFrame.Sunken)
        self.Ki_label.setLineWidth(3)
        self.Ki_label.setStyleSheet("background-color:rgb(53,53,53);")

        # derivative text
        self.Kd0_textbox = QLineEdit(self)
        self.Kd0_textbox.move(0, 54)
        self.Kd0_textbox.resize(50, 23)
        self.Kd0_textbox.setValidator(self.onlyDouble)
        self.Kd0_textbox.setText(str(fc.Kd[0]))
        self.Kd1_textbox = QLineEdit(self)
        self.Kd1_textbox.move(50, 54)
        self.Kd1_textbox.resize(50, 23)
        self.Kd1_textbox.setValidator(self.onlyDouble)
        self.Kd1_textbox.setText(str(fc.Kd[1]))
        self.Kd2_textbox = QLineEdit(self)
        self.Kd2_textbox.move(100, 54)
        self.Kd2_textbox.resize(50, 23)
        self.Kd2_textbox.setValidator(self.onlyDouble)
        self.Kd2_textbox.setText(str(fc.Kd[2]))
        # derivative label
        self.Kd_label = QLabel(self)
        self.Kd_label.resize(85, 23)
        self.Kd_label.setText('Derivative')
        self.Kd_label.move(150, 54)
        self.Kd_label.setFrameShape(QFrame.Panel)
        self.Kd_label.setFrameShadow(QFrame.Sunken)
        self.Kd_label.setLineWidth(3)
        self.Kd_label.setStyleSheet("background-color:rgb(53,53,53);")

        # button to insert new PID values
        self.insert_PID_values = QPushButton("Insert PID Gains", self)
        self.insert_PID_values.setStyleSheet("background-color: purple")
        self.insert_PID_values.move(150, 80)
        self.insert_PID_values.resize(85, 25)
        self.insert_PID_values.setFont(QFont("Helvetica", 11.5))
        self.insert_PID_values.setCheckable(True)
        self.insert_PID_values.setEnabled(True)
        self.insert_PID_values.clicked.connect(self.get_PID_value)

        # label for Roll, Pitch, Yaw
        self.RPY = QLabel(self)
        self.RPY.move(0,80)
        self.RPY.setText(' ROLL     PITCH      YAW')
        self.RPY.setFrameShape(QFrame.Panel)
        self.RPY.setFrameShadow(QFrame.Sunken)
        self.RPY.setLineWidth(3)
        self.RPY.setStyleSheet("background-color:rgb(53,53,53);")

    # creates shortcut labels
    def create_shortcut_labels(self):
        # label for key shortcuts
        self.key_a_shortcut = QLabel(self)
        self.key_a_shortcut.move(712, 472)
        self.key_a_shortcut.setText("Press 'a' to arm")
        self.key_spacebar_shortcut = QLabel(self)
        self.key_spacebar_shortcut.move(640, 485)
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
        self.log.move(0, 215)
        self.log.resize(260,283)
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
        self.sensor_label_title = QLabel("insert_sensor_title_here", self)
        self.sensor_label_title.move(650,168)
        self.sensor_label_title.setStyleSheet("background-color:rgb(53,53,53);")

        # label to hold the data
        self.sensor_label = QLabel("sensor_value", self)
        self.sensor_label.move(650,180)
        self.sensor_label.resize(50,12)

        # set interval to update every 500 ms
        self.qTimer.setInterval(500)

        # connect timeout signal to signal handler
        self.qTimer.timeout.connect(self.get_sensor_value)

        self.qTimer.start()

        # dummy variable to demonstrate the monitor
        self.i = 0

    # gets the sensor's value
    def get_sensor_value(self):
        self.i += 1.32

        # get's the sensor value
        self.sensor_label.setText(str(self.i))

    def create_plot_button(self):
        self.plot_button = QPushButton("Show Plot Graph", self)
        self.plot_button.clicked.connect(self.create_plot)
        self.plot_button.move(265,215)
        self.plot_button.setStyleSheet("Black")

    # create a live plot graph
    def create_plot(self):
        self.pw = pg.PlotWidget(self)
        self.pw.showGrid(x=True,y=True)
        self.pw.setTitle('insert_title')
        self.pw.move(265,215)
        self.pw.resize(400,282)
        self.pw.show()
        self.pw.setLabel('left', 'insert_value')
        self.pw.setLabel('bottom', 'Time', units='s')
        self.timer = pg.QtCore.QTimer(self)

        def update():
            x = np.random.normal(size=(100))
            y = np.random.normal(size=(100))
            self.pw.plot(x, y, clear=True)

        self.timer.timeout.connect(update)
        # length between updates (in ms)
        self.timer.start(1000)


# a class to read the command line output stream
class Stream(QObject):
    newText = pyqtSignal(str)

    def write(self, text):
        self.newText.emit(str(text))



if __name__ == '__main__':
    import sys

    app = QApplication(sys.argv)
    gallery = DroneGUI()
    gallery.setGeometry(0, 0,800, 500)
    gallery.show()
    sys.exit(app.exec_())



