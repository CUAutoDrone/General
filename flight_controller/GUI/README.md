# Drone Graphical User Interface
<kbd>Purpose: 
</kbd>
A GUI designed to help with and simplify interacting with the quadcopter components. It also provides
live feed data for debugging and performance optimization. This interface revolves around the ```flight_controller``` package.

<kbd>Notes:</kbd>
* I have not been able to test anything yet for flight_controller since I do not have access to the correct sensors. Thus, all the actual code to execute drone commands is currently commented out. Rigorous testing will conclude when possible.
* Please let me know if there is any additional features or changes you would like. Examples may include types of data you want to plot, external features, etc.


# Current Features [<kbd>Version: 0.1.0</kbd>]
**Settings Tab**
![Imgur](https://i.imgur.com/ZE0I5WC.png?3)
* Buttons, shortcuts keys to **arm**, **kill**, and **unlock arm**
* Insert **PID gains** directly
* Display current drone status and flight time

**Data Tab**
![Imgur](https://i.imgur.com/f5Qin1C.png)
* Live update data graph plot
* Live update sensor readings

**Flight Pattern Tab**
* Currently Under Construction

**CV Tab**
* Currently Under Construction

## To Execute:
### Prerequisites:
* ```Python 3```
* ```PyQt5```
* ```numpy```
* ```pyqtgraph```
* ```flight_controller```

First, open Terminal. Find the folder that setup.py and GUI_Drone.py are located in terminal.
For example,
```
cd Desktop/PyCharm/AerialRobotics/flight_controller/GUI
```
Then, type the following in your terminal:
```
python3 GUI_Drone.py
```


### To Create an Executable on Mac OS:
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

### To Create an Executable on PC, Linux:
First type in your terminal:
```
pip install pyinstaller
```
Then type:
```
pyinstall --onefile --windowed GUI_Drone.py
```

### Versioning
We use [SemVer](http://semver.org/) for versioning.

### Author(s)
Chris Gyurgyik



