import math
import cv2
import time
from gamepad import XboxController
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)


class Robot:
    GPIO.setup(16, GPIO.OUT)
    GPIO.setup(18, GPIO.OUT)
    GPIO.setup(13, GPIO.OUT)
    GPIO.setup(15, GPIO.OUT)
    GPIO.setup(40, GPIO.OUT)
    GPIO.setup(37, GPIO.OUT)
    GPIO.setup(38, GPIO.OUT)

    Right_PWM_Pin = GPIO.PWM(40, 100)
    Right_PWM_Pin.start(0)
    Left_PWM_Pin = GPIO.PWM(38, 100)
    Left_PWM_Pin.start(0)

    Right_Motor, Left_Motor = (13, 15, Right_PWM_Pin), (16, 18, Left_PWM_Pin)

    @staticmethod
    def Set_Motor(Motor, direction: int, speed: float) -> None:
        def stick_to_cycle(value) -> float:
            # Old = 0.0 - 1.0
            # New 35 - 100
            return (((value - 0.0) * (100 - 35)) / (1.0 - 0.0)) + 35

        p1, p2, p3 = Motor
        match_dict = {1: (1, 0), 0: (0, 0), -1: (0, 1)}
        o1, o2 = match_dict.get(direction)
        GPIO.output(p1, o1)
        GPIO.output(p2, o2)
        p3.ChangeDutyCycle(speed)

    @staticmethod
    def Turn(degrees: float):
        pass

    @staticmethod
    def Jump_Start():
        Motors = (Robot.Left_Motor, Robot.Right_Motor)
        for Motor in Motors:
            Robot.Set_Motor(Motor, 0, 1.00)
        time.sleep(1)
        for Motor in Motors:
            Robot.Set_Motor(Motor, 1, 100)

    @staticmethod
    def Kick():
        print("Kick")
        GPIO.output(37, False)
        time.sleep(0.33)
        GPIO.output(37, True)

    @staticmethod
    def Get_Distance() -> float:
        trigger: int = 0
        echo: int = 0
        GPIO.output(trigger, True)
        time.sleep(0.00001)
        GPIO.output(trigger, False)

        StartZeit = time.time()
        StopZeit = time.time()
        while GPIO.input(echo) == 0:
            StartZeit = time.time()

        while GPIO.input(echo) == 1:
            StopZeit = time.time()

        return ((StopZeit - StartZeit) * 34300) / 2

    def get_motor_speed2(Axis, view_field=180.0):
        def dir_to(mp, tp) -> float:
            dx = tp[0] - mp[0]
            dy = tp[1] - mp[1]
            rads = math.atan2(-dy, dx)
            rads %= 2 * math.pi
            return math.degrees(rads)

        def dis_to(mp, tp: tuple) -> float:
            return math.hypot(mp[0] - tp[0], mp[1] - tp[1])

        if Axis[1] < 0:
            Axis = (Axis[0], abs(Axis[1]))
        angle_to_ball = 360 - dir_to((0, 0), Axis)
        L = view_field - angle_to_ball
        R = view_field - L
        max_p = dis_to((0, 0), Axis) * (1 / 1.20)
        if R > L:
            return max_p, max_p * L / R
        else:
            return max_p * R / L, max_p

    def Use_Stick(X_AXIS: float, Y_AXIS: float, Motors: tuple):
        def convert_to_one(value):
            if value == 0:
                return 0
            return value * (1 / abs(value))

        Ways = Robot.get_motor_speed2((X_AXIS, -Y_AXIS))
        for Motor, Way in zip(Motors, Ways):
            D = 1 if Y_AXIS < 0 else -1
            Robot.Set_Motor(Motor, D, Way * 90)

    def Use_Triggers(Sticks: tuple, Motors: tuple):
        Right_Motor, Left_Motor = Motors
        RightTrigger, LeftTrigger = Sticks
        for Motor, Trigger in zip([Right_Motor, Left_Motor], [RightTrigger, LeftTrigger]):
            if Trigger > 0.1:
                Robot.Set_Motor(Motor, 1)
                Robot.Set_Speed(Motor, Trigger)
            else:
                Robot.Set_Motor(Motor, 0)


def main(with_cam=False):
    joy = XboxController()
    if with_cam:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    while True:
        if with_cam:
            _, frame = cap.read()
            frame = cv2.flip(frame, -1)
            cv2.imshow("Window", frame)
            for Button, Folder in ((joy.X, "Positive"), (joy.B, "Negative")):
                if Button:
                    print("Shot")
                    cv2.imwrite(f"{Folder}/{time.time()}.png", frame)

        Robot.Use_Stick(joy.LeftJoystickX, joy.LeftJoystickY, (Robot.Right_Motor, Robot.Left_Motor))
        if joy.Y:
            Robot.Jump_Start()
        GPIO.output(37, joy.RightTrigger > 0.1)
        key = cv2.waitKey(1)
        if key == 27:
            break


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        cv2.destroyAllWindows()
        exit()
