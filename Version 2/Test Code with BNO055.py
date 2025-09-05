import machine
import time
import neopixel

# Define GPIO pins and PWM frequency
PWM_FREQ = 1000  # PWM frequency in Hz

# Flywheel and feed mechanism motors (remain as digital outputs)
motor1_pin = machine.Pin(15, machine.Pin.OUT)      # Flywheel motor
motor2_pin = machine.Pin(14, machine.Pin.OUT)      # Feed mechanism motor

# Pan motor (Motor 3) PWM setup
motor3_pwm1 = machine.PWM(machine.Pin(9))          # Pan motor control pin 1 (IN1)
motor3_pwm2 = machine.PWM(machine.Pin(10))         # Pan motor control pin 2 (IN2)
motor3_pwm1.freq(PWM_FREQ)
motor3_pwm2.freq(PWM_FREQ)

# Tilt motor (Motor 4) PWM setup
motor4_pwm1 = machine.PWM(machine.Pin(12))         # Tilt motor control pin 1 (IN1)
motor4_pwm2 = machine.PWM(machine.Pin(13))         # Tilt motor control pin 2 (IN2)
motor4_pwm1.freq(PWM_FREQ)
motor4_pwm2.freq(PWM_FREQ)

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
BALL_TARGET_COUNT = 5          # Number of balls to fire
TIMEOUT_DURATION = 20000       # Timeout in milliseconds for firing sequence
current_spike_threshold = 1000  # Threshold in mA for detecting a ball

# Pulsing effect variables for idle state
pulsing_brightness = 0.3  # 30% brightness
pulsing_direction = 1
pulsing_level = 0
pulsing_step = 30  # Faster pulsing

# Global variable for zero-point offset
zero_point_offset = 0.0

def initialize_hardware():
    """Initialize hardware components and set initial states."""
    global zero_point_offset
    # Pull GP22 low at startup
    gp22_5v_output.value(0)
    print("GP22 set to low at startup.")

    # Set initial PWM duty cycles to 0 (motors stopped)
    motor3_pwm1.duty_u16(0)
    motor3_pwm2.duty_u16(0)
    motor4_pwm1.duty_u16(0)
    motor4_pwm2.duty_u16(0)

    # Initialize zero-point offset
    zero_point_offset = 0.0

def initialize_bno055():
    """Initialize the BNO055 sensor into NDOF mode and perform axis remapping."""
    BNO055_OPR_MODE_ADDR = 0x3D
    BNO055_PWR_MODE_ADDR = 0x3E
    BNO055_SYS_TRIGGER_ADDR = 0x3F
    BNO055_PAGE_ID_ADDR = 0x07

    # Set to config mode
    i2c1.writeto_mem(BNO055_ADDRESS, BNO055_OPR_MODE_ADDR, bytes([0x00]))
    time.sleep(0.025)  # Delay per datasheet

    # Perform axis remapping
    remap_bno055_axes()

    # Set power mode to normal
    i2c1.writeto_mem(BNO055_ADDRESS, BNO055_PWR_MODE_ADDR, bytes([0x00]))
    time.sleep(0.010)

    # Set to page 0
    i2c1.writeto_mem(BNO055_ADDRESS, BNO055_PAGE_ID_ADDR, bytes([0x00]))

    # Set to NDOF mode
    i2c1.writeto_mem(BNO055_ADDRESS, BNO055_OPR_MODE_ADDR, bytes([0x0C]))
    time.sleep(0.020)  # Delay per datasheet

    print("BNO055 initialized to NDOF mode.")

def remap_bno055_axes():
    """Remap BNO055 axes to match turret orientation."""
    BNO055_OPR_MODE_ADDR = 0x3D
    AXIS_MAP_CONFIG_ADDR = 0x41
    AXIS_MAP_SIGN_ADDR = 0x42

    # Set to config mode to change settings
    i2c1.writeto_mem(BNO055_ADDRESS, BNO055_OPR_MODE_ADDR, bytes([0x00]))
    time.sleep(0.025)

    # Remap axes as needed (example values, adjust based on mounting)
    # Since all turrets have identical mounting, set the appropriate values
    axis_map_config = 0x21  # Example: Remap ZYX to XZY
    axis_map_sign = 0x04    # Example: Invert Z axis

    i2c1.writeto_mem(BNO055_ADDRESS, AXIS_MAP_CONFIG_ADDR, bytes([axis_map_config]))
    i2c1.writeto_mem(BNO055_ADDRESS, AXIS_MAP_SIGN_ADDR, bytes([axis_map_sign]))

    print(f"Axis remapping applied: CONFIG=0x{axis_map_config:02X}, SIGN=0x{axis_map_sign:02X}")

    # Set back to operation mode
    i2c1.writeto_mem(BNO055_ADDRESS, BNO055_OPR_MODE_ADDR, bytes([0x0C]))
    time.sleep(0.020)

def calibrate_zero_point():
    """Calibrate the zero-point offset for the pitch angle."""
    global zero_point_offset
    print("Calibrating zero-point offset...")
    # Wait a moment for the sensor to stabilize
    time.sleep(1)
    pitch = read_bno055_pitch(raw=True)
    if pitch is not None:
        zero_point_offset = pitch
        print(f"Zero-point offset set to: {zero_point_offset:.2f}°")
    else:
        print("Failed to read pitch during calibration.")

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
        # Initialize BNO055 if detected
        initialize_bno055()
        calibrate_zero_point()       # Calibrate zero-point offset
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

    # Pan left (Inverted speed to correct spinning direction)
    set_motor_speed(motor3_pwm1, motor3_pwm2, speed=-32768)  # -50% duty cycle
    time.sleep(0.5)
    set_motor_speed(motor3_pwm1, motor3_pwm2, speed=0)      # Stop
    time.sleep(0.5)

    # Pan right (Inverted speed to correct spinning direction)
    set_motor_speed(motor3_pwm1, motor3_pwm2, speed=32768)  # 50% duty cycle
    time.sleep(0.5)
    set_motor_speed(motor3_pwm1, motor3_pwm2, speed=0)       # Stop
    np[0] = (0, 0, 0)    # Turn off LED 1
    np.write()
    time.sleep(0.5)

    # Tilt test with LED 2 green
    np[1] = (0, 255, 0)  # LED 2 green
    np.write()

    # Tilt up
    set_motor_speed(motor4_pwm1, motor4_pwm2, speed=32768)  # 50% duty cycle
    time.sleep(0.5)
    set_motor_speed(motor4_pwm1, motor4_pwm2, speed=0)      # Stop
    time.sleep(0.5)

    # Tilt down
    set_motor_speed(motor4_pwm1, motor4_pwm2, speed=-32768)  # -50% duty cycle
    time.sleep(0.5)
    set_motor_speed(motor4_pwm1, motor4_pwm2, speed=0)       # Stop
    np[1] = (0, 0, 0)    # Turn off LED 2
    np.write()
    print("Pan-tilt test complete.")

def set_motor_speed(pwm_pin1, pwm_pin2, speed):
    """Set the motor speed and direction using PWM.

    Args:
        pwm_pin1: PWM object for motor control pin 1 (IN1).
        pwm_pin2: PWM object for motor control pin 2 (IN2).
        speed (int): Speed value from -65535 to 65535.
                     Positive for forward, negative for reverse.
    """
    if speed > 0:
        # Forward direction
        duty_cycle = min(speed, 65535)
        pwm_pin1.duty_u16(duty_cycle)
        pwm_pin2.duty_u16(0)
    elif speed < 0:
        # Reverse direction
        duty_cycle = min(-speed, 65535)
        pwm_pin1.duty_u16(0)
        pwm_pin2.duty_u16(duty_cycle)
    else:
        # Stop
        pwm_pin1.duty_u16(0)
        pwm_pin2.duty_u16(0)

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

def read_bno055_pitch(raw=False):
    """Read the pitch angle from the BNO055 sensor."""
    BNO055_EULER_PITCH_LSB_ADDR = 0x1E  # Euler Pitch LSB
    global zero_point_offset

    try:
        data = i2c1.readfrom_mem(BNO055_ADDRESS, BNO055_EULER_PITCH_LSB_ADDR, 2)
        raw_pitch = int.from_bytes(data, 'little')
        # Manually handle signed 16-bit integer
        if raw_pitch >= 0x8000:
            raw_pitch -= 0x10000
        pitch = raw_pitch / 16.0  # Convert to degrees (1/16 degree units)
        if not raw:
            # Adjust pitch with zero-point offset
            pitch -= zero_point_offset
        print(f"Raw pitch data: {raw_pitch}, Pitch: {pitch:.2f}°")
        return pitch
    except OSError:
        print("Error reading pitch from BNO055.")
        return None

def tilt_leveling_control():
    """Control the tilt motor to level the turret using BNO055."""
    desired_pitch = 0.0  # Desired pitch angle after zero-point adjustment
    Kp = 1000           # Proportional gain
    deadband = 2.0      # Degrees within which we consider the pitch acceptable

    pitch = read_bno055_pitch()
    if pitch is None:
        print("Failed to read pitch")
        # Stop motors to be safe
        set_motor_speed(motor4_pwm1, motor4_pwm2, speed=0)
        return

    # Calculate error
    error = desired_pitch - pitch
    print(f"Pitch: {pitch:.2f}°, Error: {error:.2f}°")

    # Check if error is within deadband
    if abs(error) <= deadband:
        # Stop motors
        set_motor_speed(motor4_pwm1, motor4_pwm2, speed=0)
        # Turn off LED indicating leveling
        np[1] = (0, 0, 0)
        np.write()
        print("Turret leveled.")
        return True  # Indicate leveling is complete

    # Calculate control effort (speed)
    control_effort = int(Kp * error)
    print(f"Initial Control Effort: {control_effort}")

    # Limit control effort to minimum speed to overcome static friction
    min_duty = 20000  # Adjust as necessary
    if 0 < abs(control_effort) < min_duty:
        control_effort = min_duty if control_effort > 0 else -min_duty

    # Limit control effort to maximum duty cycle
    max_duty = 65535
    if control_effort > max_duty:
        control_effort = max_duty
    elif control_effort < -max_duty:
        control_effort = -max_duty

    print(f"Adjusted Control Effort: {control_effort}")

    # Set motor speed
    set_motor_speed(motor4_pwm1, motor4_pwm2, speed=control_effort)

    # Turn on LED to indicate leveling in progress
    np[1] = (0, 0, 255)  # Blue LED
    np.write()
    return False  # Indicate leveling is not yet complete

def firing_sequence():
    """Execute the firing sequence."""
    # Pull GP22 low at the start
    gp22_5v_output.value(0)
    print("GP22 set to low.")

    # Run pan-tilt test
    pan_tilt_test()

    # Begin firing sequence
    print("Firing sequence started.")

    # Level the turret before firing
    leveling_timeout = 5000  # Leveling timeout in milliseconds
    leveling_start_time = time.ticks_ms()
    while True:
        leveled = tilt_leveling_control()
        # Check if leveling is complete
        if leveled:
            break
        # Timeout check
        if time.ticks_diff(time.ticks_ms(), leveling_start_time) > leveling_timeout:
            print("Leveling timeout.")
            break
        time.sleep(0.1)  # Small delay between adjustments

    # Ensure tilt motor is stopped
    set_motor_speed(motor4_pwm1, motor4_pwm2, speed=0)
    np[1] = (0, 0, 0)
    np.write()

    # Set GP22 high when flywheels start spinning
    gp22_5v_output.value(1)
    print("GP22 set to high (Flywheels spinning).")

    # Start flywheel motor and set LED 2 to blue
    motor1_pin.value(1)
    np[1] = (0, 0, 255)  # LED 2 blue
    np.write()

    # Wait for flywheels to spool up
    time.sleep(2)

    # Start feed mechanism and set LED 1 to yellow
    motor2_pin.value(1)
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
                motor2_pin.value(0)
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
    motor2_pin.value(0)

    # Stop flywheel motor
    motor1_pin.value(0)
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
        while button_pin.value() == 1:
            pulse_idle_leds()
            time.sleep(0.1)
        firing_sequence()
        print("Firing sequence finished. System resetting.")

# Run the main function
main()
