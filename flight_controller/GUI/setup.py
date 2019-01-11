from setuptools import setup

# To create an executable on Mac,
# First, type in your terminal:
#       pip install -U py2app
# This will install py2app, which converts python files into programs
#
# to convert the Drone GUI to an executable, first find the folder that this file
# and GUI_Drone.py are located. Then, type the following in your terminal:
#                       python3 setup.py py2app -A --packages=PyQt5

setup(app=['GUI_Drone.py'], setup_requires=['py2app'])


# To make an executable on Linux, PC:
# First type in your terminal:
#        pip install pyinstaller
# Then type:
#        pyinstall --onefile --windowed GUI_Drone.py
