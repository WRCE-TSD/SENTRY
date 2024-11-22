import machine
import time
import neopixel

# Define GPIO pins
motor1_pwm = machine.PWM(machine.Pin(15))      # Flywheel motor PWM
motor2_pwm = machine.PWM(machine.Pin(14))      # Feed mechanism motor PWM
motor3_pwm1 = machine.PWM(machine.Pin(9))      # Pan motor control PWM pin 1
motor3_pwm2 = machine.PWM(machine.Pin(10))     # Pan motor control PWM pin 2
motor4_pwm1 = machine.PWM(machine.Pin(12))     # Tilt motor control PWM pin 1
motor4_pwm2 = machine.PWM(machine.Pin(13))     # Tilt motor control PWM pin 2
button_pin = machine.Pin(5, machine.Pin.IN, machine.Pin.PULL_UP)  # Start button with pull-up resistor
gp22_5v_output = machine.Pin(22, machine.Pin.OUT)  # 5V Output control

# I2C setup for INA260 (I2C0) and BNO055 (I2C1)
i2c0 = machine.I2C(0, scl=machine.Pin(1), sda=machine.Pin(0), freq=400000)  # INA260
i2c1 = machine.I2C(1, scl=machine.Pin(3), sda=machine.Pin(2), freq=400000)  # BNO055

INA260_ADDRESS = 0x40
INA260_CURRENT_REGISTER = 0x01
BNO055_ADDRESS = 0x28

# Initialize NeoPixel LEDs
num_leds = 2
np = neopixel.NeoPixel(machine.Pin(4), num_leds)

# Initialize ADC for QRD1114 sensors
sensor1_adc = machine.ADC(26)  # QRD1114 connected to GP26
sensor2_adc = machine.ADC(27)  # QRD1114 connected to GP27

# System parameters
BALL_TARGET_COUNT = 5  # Number of balls to fire
TIMEOUT_DURATION = 20000  # Timeout in milliseconds for firing sequence
current_spike_threshold = 1000  # Threshold in mA for detecting a ball

# Pulsing effect variables for idle state
pulsing_brightness = 0.3  # 30% brightness
pulsing_direction = 1
pulsing_level = 0
pulsing_step = 30  # Faster pulsing

def initialize_hardware():
    """Initialize hardware components and set initial states."""
    # Set initial PWM frequency and duty cycle
    motor1_pwm.freq(1000)
    motor2_pwm.freq(1000)
    motor3_pwm1.freq(1000)
    motor3_pwm2.freq(1000)
    motor4_pwm1.freq(1000)
    motor4_pwm2.freq(1000)
    motor1_pwm.duty_u16(0)
    motor2_pwm.duty_u16(0)
    motor3_pwm1.duty_u16(0)
    motor3_pwm2.duty_u16(0)
    motor4_pwm1.duty_u16(0)
    motor4_pwm2.duty_u16(0)
    # Pull GP22 low at startup
    gp22_5v_output.value(0)
    print("GP22 set to low at startup.")

def flash_led(index, color, flashes):
    """Flash a specific LED a given number of times with the specified color."""
    for _ in range(flashes):
        np[index] = color
        np.write()
        time.sleep(0.2)
        np[index] = (0, 0, 0)
        np.write()
        time.sleep(0.2)

def check_bno055():
    """Check if the BNO055 sensor is connected."""
    BNO055_CHIP_ID_ADDR = 0x00
    BNO055_EXPECTED_CHIP_ID = 0xA0

    # Power-up delay (minimum 650 ms)
    time.sleep(0.7)

    try:
        chip_id = i2c1.readfrom_mem(BNO055_ADDRESS, BNO055_CHIP_ID_ADDR, 1)
        if chip_id[0] == BNO055_EXPECTED_CHIP_ID:
            print("BNO055 detected.")
            return True
        else:
            print(f"BNO055 not detected. Unexpected CHIP ID: {chip_id[0]:#04x}")
            return False
    except OSError as e:
        print(f"BNO055 not detected. I2C error: {e}")
        return False

def check_ina260():
    """Check if the INA260 sensor is connected."""
    try:
        i2c0.readfrom_mem(INA260_ADDRESS, INA260_CURRENT_REGISTER, 2)
        print("INA260 detected.")
        return True
    except OSError:
        print("INA260 not detected.")
        return False

def test_qrd1114_sensors():
    """Test the QRD1114 sensors and flash LEDs accordingly."""
    # Read sensor values
    sensor1_value = sensor1_adc.read_u16()
    sensor2_value = sensor2_adc.read_u16()

    print(f"Sensor 1 Value: {sensor1_value}")
    print(f"Sensor 2 Value: {sensor2_value}")

    # Threshold for determining sensor status
    THRESHOLD = 64000

    # Test Sensor 1
    if sensor1_value > THRESHOLD:
        flash_led(0, (0, 255, 0), 3)  # Green flashes
    else:
        flash_led(0, (255, 0, 0), 3)  # Red flashes

    # Test Sensor 2
    if sensor2_value > THRESHOLD:
        flash_led(1, (0, 255, 0), 3)  # Green flashes
    else:
        flash_led(1, (255, 0, 0), 3)  # Red flashes

def run_startup_tests():
    """Run startup tests for INA260 and BNO055 sensors and flash LEDs accordingly."""
    # Test INA260 and flash LED 1
    if check_ina260():
        flash_led(0, (0, 255, 0), 3)  # Green flashes
    else:
        flash_led(0, (255, 0, 0), 5)  # Red flashes

    # Test BNO055 and flash LED 2
    if check_bno055():
        flash_led(1, (0, 255, 0), 3)  # Green flashes
    else:
        flash_led(1, (255, 0, 0), 5)  # Red flashes

    # Test QRD1114 sensors
    test_qrd1114_sensors()

def pulse_idle_leds():
    """Pulse LEDs white while waiting for the button press."""
    global pulsing_level, pulsing_direction

    pulsing_level += pulsing_direction * pulsing_step
    if pulsing_level >= 255:
        pulsing_level = 255
        pulsing_direction = -1
    elif pulsing_level <= 0:
        pulsing_level = 0
        pulsing_direction = 1

    level = int(pulsing_level * pulsing_brightness)
    color = (level, level, level)  # White color
    for i in range(num_leds):
        np[i] = color
    np.write()
    time.sleep(0.05)  # Faster pulsing

def pan_tilt_test():
    """Perform pan and tilt test movements with LED indications."""
    print("Starting pan-tilt test.")

    # Pan test with LED 1 green
    np[0] = (0, 255, 0)  # LED 1 green
    np.write()
    # Pan left
    motor3_pwm1.duty_u16(32768)  # Set duty cycle to 50%
    motor3_pwm2.duty_u16(0)
    time.sleep(0.5)
    motor3_pwm1.duty_u16(0)
    motor3_pwm2.duty_u16(0)
    time.sleep(0.5)
    # Pan right
    motor3_pwm1.duty_u16(0)
    motor3_pwm2.duty_u16(32768)  # Set duty cycle to 50%
    time.sleep(0.5)
    motor3_pwm1.duty_u16(0)
    motor3_pwm2.duty_u16(0)
    np[0] = (0, 0, 0)    # Turn off LED 1
    np.write()
    time.sleep(0.5)

    # Tilt test with LED 2 green
    np[1] = (0, 255, 0)  # LED 2 green
    np.write()
    # Tilt up
    motor4_pwm1.duty_u16(32768)  # Set duty cycle to 50%
    motor4_pwm2.duty_u16(0)
    time.sleep(0.5)
    motor4_pwm1.duty_u16(0)
    motor4_pwm2.duty_u16(0)
    time.sleep(0.5)
    # Tilt down
    motor4_pwm1.duty_u16(0)
    motor4_pwm2.duty_u16(32768)  # Set duty cycle to 50%
    time.sleep(0.5)
    motor4_pwm1.duty_u16(0)
    motor4_pwm2.duty_u16(0)
    np[1] = (0, 0, 0)    # Turn off LED 2
    np.write()
    print("Pan-tilt test complete.")

def read_current():
    """Read current from INA260."""
    try:
        data = i2c0.readfrom_mem(INA260_ADDRESS, INA260_CURRENT_REGISTER, 2)
        raw_current = int.from_bytes(data, 'big')
        if raw_current & 0x8000:
            raw_current -= 1 << 16
        current_ma = raw_current * 1.25  # Convert to mA
        return current_ma
    except OSError:
        print("Error reading INA260.")
        return 0  # Return 0 if read fails

def ramp_motor_pwm(motor_pwm, target_duty, step=1000, delay=0.01):
    """Ramp the motor PWM duty cycle gradually to the target duty."""
    current_duty = motor_pwm.duty_u16()
    while current_duty != target_duty:
        if current_duty < target_duty:
            current_duty = min(current_duty + step, target_duty)
        else:
            current_duty = max(current_duty - step, target_duty)
        motor_pwm.duty_u16(current_duty)
        time.sleep(delay)

def firing_sequence():
    """Execute the firing sequence."""
    # Pull GP22 low at the start
    gp22_5v_output.value(0)
    print("GP22 set to low.")

    # Run pan-tilt test
    pan_tilt_test()

    # Begin firing sequence
    print("Firing sequence started.")

    # Set GP22 high when flywheels start spinning
    gp22_5v_output.value(1)
    print("GP22 set to high (Flywheels spinning).")

    # Ramp up flywheel motor and set LED 2 to blue
    motor1_pwm.duty_u16(65535)  # Set to full speed
    np[1] = (0, 0, 255)  # LED 2 blue
    np.write()

    # Wait for flywheels to spool up
    time.sleep(2)

    # Ramp up feed mechanism and set LED 1 to yellow
    ramp_motor_pwm(motor2_pwm, 65535)  # Gradually ramp to full speed  # Ramp to full speed
    np[0] = (255, 255, 0)  # LED 1 yellow
    np.write()

    print("Monitoring INA260 for current spikes.")

    # Read baseline current
    baseline_current = read_current()
    print(f"Baseline current: {baseline_current:.2f} mA")

    ball_count = 0
    spike_detected = False
    start_time = time.ticks_ms()

    while True:
        # Timeout check
        if time.ticks_diff(time.ticks_ms(), start_time) > TIMEOUT_DURATION:
            print("Timeout: Firing sequence took too long.")
            break

        current_ma = read_current()

        # Check for current spike
        if (current_ma - baseline_current) >= current_spike_threshold and not spike_detected:
            spike_detected = True
            ball_count += 1
            print(f"Ball {ball_count} detected!")

            # Stop the feed motor immediately if target ball count is reached
            if ball_count >= BALL_TARGET_COUNT:
                print("Ball target count reached, stopping feed mechanism.")
                motor2_pwm.duty_u16(0)
                np[0] = (0, 0, 0)  # Turn off LED 1
                np.write()
            else:
                # Flash LED 1 green briefly to indicate detection
                np[0] = (0, 255, 0)
                np.write()
                time.sleep(0.05)  # Short flash duration
                np[0] = (255, 255, 0)
                np.write()

            # Delay to prevent double-counting
            time.sleep(0.25)
            spike_detected = False

        time.sleep(0.005)  # Short delay for faster response

        # Break the loop if the target ball count is reached
        if ball_count >= BALL_TARGET_COUNT:
            break

    # Ensure the feed mechanism is stopped
    motor2_pwm.duty_u16(0)

    # Stop flywheel motor
    motor1_pwm.duty_u16(0)
    np[1] = (0, 0, 0)  # Turn off LED 2
    np.write()

    # Set GP22 low
    gp22_5v_output.value(0)
    print("GP22 set to low (Flywheels stopped).")

    print("Firing sequence complete.")

def main():
    """Main function to run the program."""
    initialize_hardware()
    run_startup_tests()

    while True:
        print("Waiting for button press to start firing sequence.")
        # Pulse LEDs while waiting
        while button_pin.value() == 1:
            pulse_idle_leds()
        # Button pressed, proceed to firing sequence
        firing_sequence()
        print("Firing sequence finished. System resetting.")

# Run the main function
main()
