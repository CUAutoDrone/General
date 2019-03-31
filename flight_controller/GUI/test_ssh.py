import vassal
from vassal.terminal import Terminal
from vassal.scheduler import Scheduler

# https://github.com/Shawn-Shan/vassal


# shell = Terminal(["scp username@host:/home/foo.txt foo_local.txt"])

if __name__ == "__main__":
    print("Connecting...")
    shell = Terminal(["ssh pi@192.168.1.12", "raspberry", "scp 1.txt pi@192.168.1.12:/home/pi/"])
    shell.run()