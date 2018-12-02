from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import datetime
from FlightController import Motor, IMU, Receiver, FlightController
import numpy as np
import pigpio

# receiver created with receiver channels initialized
receiver = Receiver(17, 27, 22, 18, 23)
# IMU created with MPU_6050, alpha initialized
imu = IMU(0x68, 0.98)
# motors created with motors initialized
motor = Motor(10, 9, 25, 8)
#initialize the flight controller
fc = FlightController(np.array([0.1, 0.2, 0.3]), np.array([0.4, 0.5, 0.6]), np.array([0.7, 0.8, 0.9]), receiver, imu, motor)

# TODO:
#
#


class DroneGUI(QDialog):
    def __init__(self, parent=None):
        super(DroneGUI, self).__init__(parent)

        # palette background setting, style
        self.create_palette()

        # create the arm, kill, unarm buttons
        self.create_arm_kill_buttons()

        # create the PID insert text boxes
        self.create_PID_insert()


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
        self.killswitch_button.move(590, 0)
        self.killswitch_button.setFont(QFont("Helvetica", 17.5))
        self.killswitch_button.resize(110, 60)
        self.killswitch_button.clicked.connect(self.kill_motor)
        self.killswitch_button.setStyleSheet("background-color: red")
        self.killswitch_button.setEnabled(False)

        # button to undo kill switch
        self.undo_killswitch_button = QPushButton('Unlock ARM', self)
        self.undo_killswitch_button.move(590, 62)
        self.undo_killswitch_button.setFont(QFont("Helvetica", 12))
        self.undo_killswitch_button.resize(75, 30)
        self.undo_killswitch_button.clicked.connect(self.undo_killswitch)
        self.undo_killswitch_button.setStyleSheet("background-color:rgb(53,53,53);")
        self.undo_killswitch_button.setEnabled(False)

    # killswitch for the drone
    def kill_motor(self):
        print("kill switch activated")
        if self.arm_button.isChecked():
            self.arm_button.toggle()
            self.arm_button.setEnabled(False)
            self.undo_killswitch_button.setEnabled(True)
            self.undo_killswitch_button.setStyleSheet("background-color: yellow; color:black")
            self.killswitch_button.setEnabled(False)
            #fc.receiver.ARM = 0
            #fc.receiver.can_arm()

    # allows the drone to be armed again
    def undo_killswitch(self):
        print("arm button unlocked")
        self.arm_button.setEnabled(True)
        self.undo_killswitch_button.setEnabled(False)
        #fc.receiver.ARM = 1

    # arms the drone
    def arm_drone(self):
        print("arming drone")
        self.undo_killswitch_button.setEnabled(False)
        self.killswitch_button.setEnabled(True)
        self.undo_killswitch_button.setStyleSheet("background-color:rgb(53,53,53);")
        self.arm_button.setEnabled(False)
        #fc.run()


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
            print("Updated Proportional Gain: ", "Roll: ", self.Kp0_textbox.text(), " Pitch: ", self.Kp1_textbox.text(),
                  " Yaw: ", self.Kp2_textbox.text())
            print("Updated Integral Gain: ", "    Roll: ", self.Ki0_textbox.text(), " Pitch: ", self.Ki1_textbox.text(),
                  " Yaw: ", self.Ki2_textbox.text())
            print("Updated Proportional Gain: ", "Roll: ", self.Kd0_textbox.text(), " Pitch: ", self.Kd1_textbox.text(),
                  " Yaw: ", self.Kd2_textbox.text())

    # creates the PID textboxes, button for updating PID values
    def create_PID_insert(self):

        # proportional text
        self.onlyDouble = QDoubleValidator()
        self.Kp0_textbox = QLineEdit(self)
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

if __name__ == '__main__':
    import sys

    app = QApplication(sys.argv)
    gallery = DroneGUI()
    gallery.setGeometry(0, 0, 700, 415)
    gallery.show()
    sys.exit(app.exec_())
