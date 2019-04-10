import vassal
from vassal.terminal import Terminal
from vassal.scheduler import Scheduler

# https://github.com/Shawn-Shan/vassal


if __name__ == "__main__":
    print("Connecting...")
    shell = Terminal(["ssh pi@192.168.1.12", "scp 1.txt pi@192.168.1.12:/home/pi/"], password="raspberry")
    shell.run()


#### Thoughts ####
# Command to execute script to run the drone
# In that script, ways to gather data to either Webserver/TCP
### - Options: write data to .txt file, read .txt file and copy to Webserver
# How to send data to the drone?