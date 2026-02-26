from Rosmaster_Lib import Rosmaster
import time

if __name__ == '__main__':
    ros = Rosmaster()

    speed = 50  # adjust as needed

    try:
        # ---- 1️⃣ Forward 5 seconds ----
        print("Forward 5s")
        ros.set_car_run(1, speed)
        time.sleep(3)
        ros.set_car_run(0, 0)
        time.sleep(0.5)

        # ---- 2️⃣ Turn Left briefly ----
        print("Turning left")
        ros.set_car_run(3, speed)
        time.sleep(1.2)  # adjust until ~90° turn
        ros.set_car_run(0, 0)
        time.sleep(0.5)

        # ---- 3️⃣ Forward 3 seconds ----
        print("Forward 3s")
        ros.set_car_run(1, speed)
        time.sleep(3)
        ros.set_car_run(0, 0)
        time.sleep(0.5)

        # ---- 4️⃣ Spin Right 3 times ----
        print("Spinning right 3 times")
        for _ in range(3):
            ros.set_car_run(6, speed)   # spin right
            time.sleep(1.3)             # adjust for one full 360°
            ros.set_car_run(0, 0)
            time.sleep(0.5)

        # ---- 5️⃣ Forward 6 seconds ----
        print("Forward 6s")
        ros.set_car_run(1, speed)
        time.sleep(6)
        ros.set_car_run(0, 0)

    except KeyboardInterrupt:
        print("Interrupted")

    finally:
        ros.set_car_run(0, 0)
