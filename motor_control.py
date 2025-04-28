import smbus
import time
import RPi.GPIO as GPIO

# Common setup
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# ----------------------------------
# DC Motor configuration
# ----------------------------------
RIGHT_PIN = 13
LEFT_PIN = 15
PROXIMITY_PIN = 18

pwm_pin = 32
motor_dir_pin = 11

GPIO.setup(motor_dir_pin, GPIO.OUT)
GPIO.setup(pwm_pin, GPIO.OUT)
GPIO.setup(RIGHT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LEFT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PROXIMITY_PIN, GPIO.IN)

pwm = GPIO.PWM(pwm_pin, 1000)  # Set PWM frequency to 1000 Hz
pwm.start(0)

def rotate_right(speed):
    GPIO.output(motor_dir_pin, GPIO.HIGH)
    pwm.ChangeDutyCycle(speed)

def rotate_left(speed):
    GPIO.output(motor_dir_pin, GPIO.LOW)
    pwm.ChangeDutyCycle(speed)

def stop_motor():
    pwm.ChangeDutyCycle(0)

# ----------------------------------
# LCD configuration
# ----------------------------------
I2C_ADDR = 0x27  # I2C address of the LCD
LCD_WIDTH = 16

LCD_CHR = 1
LCD_CMD = 0
LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0

ENABLE = 0b00000100
BACKLIGHT = 0b00001000

SWITCH = 11
GPIO.setup(SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_UP)

bus = smbus.SMBus(1)

def lcd_write(bits, mode):
    high_bits = mode | (bits & 0xF0) | BACKLIGHT
    low_bits = mode | ((bits << 4) & 0xF0) | BACKLIGHT

    bus.write_byte(I2C_ADDR, high_bits)
    lcd_toggle_enable(high_bits)
    bus.write_byte(I2C_ADDR, low_bits)
    lcd_toggle_enable(low_bits)

def lcd_toggle_enable(bits):
    time.sleep(0.0005)
    bus.write_byte(I2C_ADDR, (bits | ENABLE))
    time.sleep(0.0005)
    bus.write_byte(I2C_ADDR, (bits & ~ENABLE))
    time.sleep(0.0005)

def lcd_init():
    lcd_write(0x33, LCD_CMD)
    lcd_write(0x32, LCD_CMD)
    lcd_write(0x06, LCD_CMD)
    lcd_write(0x0C, LCD_CMD)
    lcd_write(0x28, LCD_CMD)
    lcd_write(0x01, LCD_CMD)
    time.sleep(0.0005)

def lcd_message(message):
    message = message.ljust(LCD_WIDTH, " ")
    for char in message:
        lcd_write(ord(char), LCD_CHR)

def lcd_clear():
    lcd_write(0x01, LCD_CMD)

# ----------------------------------
# Independent motor and LCD functions
# ----------------------------------

def motor_control():
    if GPIO.input(RIGHT_PIN) == GPIO.LOW:
        print("Rotating right")
        rotate_right(50)

    elif GPIO.input(LEFT_PIN) == GPIO.LOW:
        if GPIO.input(PROXIMITY_PIN) == GPIO.HIGH:
            print("Proximity sensor activated! Motor stopped.")
            stop_motor()
            time.sleep(1)
            print("Changing direction: Rotating right")
            rotate_right(50)
        else:
            print("Rotating left")
            rotate_left(50)
    else:
        stop_motor()

def lcd_control():
    if GPIO.input(SWITCH):
        lcd_clear()
        lcd_write(LCD_LINE_1, LCD_CMD)
        lcd_message("Right")
    else:
        lcd_clear()
        lcd_write(LCD_LINE_1, LCD_CMD)
        lcd_message("Left")

# ----------------------------------
# Main program
# ----------------------------------

def main():
    lcd_init()  # Initialize LCD

    try:
        while True:
            motor_control()  # Control the motor independently
            lcd_control()    # Control the LCD independently
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        lcd_clear()
        pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
