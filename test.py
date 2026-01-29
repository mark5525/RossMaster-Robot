from Rosmaster_Lib import Rosmaster
import time

if __name__ == '__main__':
    ros = Rosmaster()
    try:
        while True:
            ros.set_motor(30, 30, 30, 30)
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        # Always stop motors on exit
        try:
            ros.set_motor(0, 0, 0, 0)
        except Exception:
            pass