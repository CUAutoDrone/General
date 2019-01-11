# Graphical User Interface (GUI)
### Version 0.1.0
**Purpose:**
A GUI designed to help with and simplify interacting with the quadcopter components. It also provides
live feed data for debugging and performance optimization.

## To run on a Mac
First, open Terminal. Find the folder that setup.py and GUI_Drone.py are located in terminal.
For example,
```
cd Desktop/PyCharm/AerialRobotics/flight_controller/GUI
```
Then, type the following in your terminal:
```
python3 GUI_Drone.py
```


## To create an executable on a Mac
First, type in your terminal:
```pip install -U py2app```
This will install py2app, which converts python files into programs

To convert the Drone GUI to an executable, open your terminal.
Find the folder that setup.py and GUI_Drone.py are located.
For example,
```
cd Desktop/PyCharm/AerialRobotics/flight_controller/GUI
```
 Then, type the following in your terminal:
```
python3 setup.py py2app -A --packages=PyQt5
```

## To create an executable on PC, Linux
First type in your terminal:
```
pip install pyinstaller
```
Then type:
```
pyinstall --onefile --windowed GUI_Drone.py
```
