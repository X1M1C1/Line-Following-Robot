import RPi.GPIO as GPIO
import time
import pigpio

#from intersection_detection_v7 import *
from picamera2 import Picamera2
from line_barycenter_detection_v6 import *

# GPIO SETUP FOR SENSORS
LEFT_WHEEL_PWM = 12
RIGHT_WHEEL_PWM = 13
ULTRASONIC_TRIG = 26
ULTRASONIC_ECHO = 19
LEFT_WHEEL_ANGLE = 16
RIGHT_WHEEL_ANGLE = 21

pi = pigpio.pi()
picam2 = Picamera2()

camera_config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(camera_config)
picam2.start()

GPIO.setmode(GPIO.BCM)
pi.hardware_PWM(LEFT_WHEEL_PWM, 50, 0)
pi.hardware_PWM(RIGHT_WHEEL_PWM, 50, 0)
#GPIO.setup(LEFT_WHEEL_PWM, GPIO.OUT)
#GPIO.setup(RIGHT_WHEEL_PWM, GPIO.OUT)
GPIO.setup(ULTRASONIC_TRIG, GPIO.OUT)
GPIO.setup(ULTRASONIC_ECHO, GPIO.IN)

#pwm_left = GPIO.PWM(LEFT_WHEEL_PWM, 50)
#pwm_right = GPIO.PWM(RIGHT_WHEEL_PWM, 50)
#pwm_left.start(50)
#pwm_right.start(50)

# input is a int from -100 to 100
def set_right_motor_speed(input_speed):
    scaled = 0.011 * (input_speed) + 7.5
    #print("Right Motor Control: %d", scaled)
    scaled = scaled * 10000
    pi.hardware_PWM(RIGHT_WHEEL_PWM, 50, round(scaled))
    #pwm_right.ChangeDutyCycle(scaled)

def set_left_motor_speed(input_speed):
    scaled = -0.011 * (input_speed) + 7.5
    #print("Left Motor Control: %d", scaled)
    scaled = scaled * 10000
    pi.hardware_PWM(LEFT_WHEEL_PWM, 50, round(scaled))
    #pwm_left.ChangeDutyCycle(scaled)
    # stop is 7.5
    # full bore is 8.6
    # reverse is 6.4

# distance is in cm
def get_distance():
    # Send a pulse to the TRIG pin
    GPIO.output(ULTRASONIC_TRIG, GPIO.LOW)
    time.sleep(0.1)  # Wait for a moment to clear the previous signal
    GPIO.output(ULTRASONIC_TRIG, GPIO.HIGH)
    time.sleep(0.00001)  # Send a short pulse
    GPIO.output(ULTRASONIC_TRIG, GPIO.LOW)

    # Wait for the Echo pin to go HIGH, then measure how long it stays HIGH
    while GPIO.input(ULTRASONIC_ECHO) == GPIO.LOW:
        pulse_start = time.time()
    
    while GPIO.input(ULTRASONIC_ECHO) == GPIO.HIGH:
        pulse_end = time.time()

    # Calculate the pulse duration and distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound is 34300 cm/s, so we divide by 2
    distance = round(distance, 2)  # Round to two decimal places

    return distance

r_last_tick = None
r_high_time = 0
r_frequency = 0
def right_wheel_rising_funct(gpio, level, tick):
    global r_last_tick
    r_last_tick = tick

def right_wheel_falling_funct(gpio, level, tick):
    if r_last_tick is not None:
        time_diff = pigpio.tickDiff(r_last_tick, tick) # Time in milliseconds
        duty = time_diff / 11 # divided by 1.1ms
        angle = 3.822 * (duty - 2.9)   # convert the pwm duty to angle, see datasheet
        
pi.callback(RIGHT_WHEEL_ANGLE, pigpio.RISING_EDGE, right_wheel_rising_funct)
pi.callback(RIGHT_WHEEL_ANGLE, pigpio.FALLING_EDGE, right_wheel_falling_funct)

def move_forward():
    set_left_motor_speed(25)
    set_right_motor_speed(25)

def turn_right():
    set_left_motor_speed(-15)
    set_right_motor_speed(15)

def turn_left():
    set_left_motor_speed(15)
    set_right_motor_speed(-15)

try:
    past_dir = 'R'
    while True:
        frame = picam2.capture_array() 
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        angle, img, is_line = hybrid_angle_detection(frame)
        
        print(angle, " ", is_line)
        if angle is None:
            if (past_dir == 'R'):
                turn_left()
            else:
                turn_right()
        else:
            if (angle > 20):
                turn_right()
                past_dir = 'R'
            elif (angle < -20):
                turn_left()
                past_dir = 'L'
            else:
                move_forward()
        time.sleep(0.2)
        
except KeyboardInterrupt:
    pass
finally:
    frame = picam2.capture_array() 
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    angle, img, is_line = hybrid_angle_detection(frame)

    cv2.imwrite("test.png",img)

    pi.hardware_PWM(RIGHT_WHEEL_PWM, 50, 0)
    pi.hardware_PWM(LEFT_WHEEL_PWM, 50, 0)
    pi.stop()
    GPIO.cleanup()