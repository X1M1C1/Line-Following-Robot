import RPi.GPIO as GPIO
import time
import pigpio

# GPIO SETUP FOR SENSORS
LEFT_WHEEL_PWM = 16
RIGHT_WHEEL_PWM = 20
ULTRASONIC_TRIG = 26
ULTRASONIC_ECHO = 19
LEFT_WHEEL_ANGLE = 12
RIGHT_WHEEL_ANGLE = 21

pi = pigpio.pi()

GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_WHEEL_PWM, GPIO.OUT)
GPIO.setup(RIGHT_WHEEL_PWM, GPIO.OUT)
GPIO.setup(ULTRASONIC_TRIG, GPIO.OUT)
GPIO.setup(ULTRASONIC_ECHO, GPIO.IN)

pwm_left = GPIO.PWM(LEFT_WHEEL_PWM, 50)
pwm_right = GPIO.PWM(RIGHT_WHEEL_PWM, 50)
pwm_left.start(50)
pwm_right.start(50)

# input is a int from -100 to 100
def set_right_motor_speed(input_speed):
    scaled = 0.011 * (input_speed + 100) + 6.4
    pwm_right.ChangeDutyCycle(scaled)
    # stop is 7.5
    # full bore is 8.6
    # reverse is 6.4

def set_left_motor_speed(input_speed):
    scaled = 0.011 * (input_speed + 100) + 6.4
    pwm_left.ChangeDutyCycle(scaled)

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
    global r_last_tick, r_high_time, r_frequency
    r_last_tick = tick

def right_wheel_falling_funct(gpio, level, tick):
    #if r_last_tick is not None:
    time_diff = pigpio.tickDiff(r_last_tick, tick) # Time in milliseconds
    duty = time_diff / 1100 # divided by 1.1ms
    angle = 3.822 * (time_diff - 2.9)   # convert the pwm duty to angle, see datasheet
        
    print(f"Angle: {angle:.2f}")

pi.callback(RIGHT_WHEEL_ANGLE, pigpio.RISING_EDGE, right_wheel_rising_funct)
pi.callback(RIGHT_WHEEL_ANGLE, pigpio.FALLING_EDGE, right_wheel_falling_funct)


try:
    while True:
        set_right_motor_speed(30)
except KeyboardInterrupt:
    pass
finally:
    pi.stop()
    GPIO.cleanup()