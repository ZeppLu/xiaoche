import RPi.GPIO as GPIO
from Adafruit_MotorHAT import Adafruit_MotorHAT


class Motor(object):
    I2C_BUS = 1
    UP_LIMIT_PIN = 17  # BCM
    DOWN_LIMIT_PIN = 18

    def __init__(self):
        driver = Adafruit_MotorHAT(i2c_bus=self.I2C_BUS)
        self.left_motor = driver.getMotor(1)
        self.right_motor = driver.getMotor(2)
        self.vertical_motor = driver.getMotor(3)
        # monitor
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.UP_LIMIT_PIN, GPIO.IN)
        GPIO.setup(self.DOWN_LIMIT_PIN, GPIO.IN)
        GPIO.add_event_detect(self.UP_LIMIT_PIN, GPIO.FALLING, callback=self.vertical_limit_callback)
        GPIO.add_event_detect(self.DOWN_LIMIT_PIN, GPIO.FALLING, callback=self.vertical_limit_callback)

    def __del__(self):
        # release motors
        self.left_motor.run(Adafruit_MotorHAT.RELEASE, 0)
        self.right_motor.run(Adafruit_MotorHAT.RELEASE, 0)
        self.vertical_motor.run(Adafruit_MotorHAT.RELEASE, 0)
        #GPIO.remove_event_detect(self.UP_LIMIT_PIN)
        #GPIO.remove_event_detect(self.DOWN_LIMIT_PIN)
        GPIO.cleanup()
        print("cleaned")

    def cleanup(self):
        self.__del__()

    def __run(self, motor, speed):
        speed = min(1, max(-1, speed))  # clamp into [-1,1]
        speed = int(speed * 255)  # convert to int8
        if speed >= 0:
            motor.run(Adafruit_MotorHAT.FORWARD, speed)
        else:  # speed < 0
            motor.run(Adafruit_MotorHAT.BACKWARD, -speed)

    def up(self, speed):
        # check whether limit has been reached
        up_limit = not GPIO.input(self.UP_LIMIT_PIN)
        down_limit = not GPIO.input(self.DOWN_LIMIT_PIN)
        can_goup = (speed > 0) and (not up_limit)
        can_godown = (speed < 0) and (not down_limit)
        if can_goup or can_godown or (speed==0):
            self.__run(self.vertical_motor, speed)

    def vertical_limit_callback(self, _):
        # check whether limit has been reached
        up_limit = not GPIO.input(self.UP_LIMIT_PIN)
        down_limit = not GPIO.input(self.DOWN_LIMIT_PIN)
        # stop vertical motor is required
        if up_limit or down_limit:
            self.up(0)
