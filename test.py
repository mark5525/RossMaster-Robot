from Rosmaster_Lib import Rosmaster
import time

if __name__ == '__main__':
    ros = Rosmaster()

    forward_speed = 30
    run_time = 6   # seconds

    try:
        # ---- Move Forward ----
        print("Moving forward...")
        ros.set_motor(forward_speed, forward_speed, forward_speed, forward_speed)
        time.sleep(run_time)

        # ---- Stop briefly (optional but recommended) ----
        ros.set_motor(0, 0, 0, 0)
        time.sleep(0.5)

        # ---- Move Backward ----
        print("Reversing...")
        ros.set_motor(-forward_speed, -forward_speed, -forward_speed, -forward_speed)
        time.sleep(run_time)

    except KeyboardInterrupt:
        print("Shutting down")

    finally:
        # Always stop motors on exit
        ros.set_motor(0, 0, 0, 0)