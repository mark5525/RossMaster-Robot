from Rosmaster_Lib import Rosmaster
import math
import time

if __name__ == '__main__':
    ros = Rosmaster()
    try:
        while True:
            ros.set_motor(10, 10, 10, 10)
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down")