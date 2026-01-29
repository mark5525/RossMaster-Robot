import Rosmaster_Lib
import math
import time

if __name__ == '__main__':
    ros = Rosmaster_Lib.Rosmaster_Lib()

    ros.set_motor(10, 10, 10, 10)