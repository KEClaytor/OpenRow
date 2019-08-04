
import os
import time
import math
import random

import board
import busio
import storage
import analogio
import digitalio

import adafruit_gps
import adafruit_sdcard
import adafruit_lis3dh
import adafruit_framebuf
import adafruit_is31fl3731
import adafruit_ht16k33.segments

# Battepy pin
vbat_pin = analogio.AnalogIn(board.VOLTAGE_MONITOR)

def get_voltage():
    return (vbat_pin.value * 3.3) / 65536 * 2

#######################
## Configure Sensors ##
#######################

# Configure LIS3DH Accelerometer
i2c = busio.I2C(board.SCL, board.SDA)
int1 = None # digitalio.DigitalInOut(board.D6)  # Set this to the correct pin for the interrupt!
lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c, int1=int1)
# Set range of accelerometer (can be RANGE_2_G, RANGE_4_G, RANGE_8_G or RANGE_16_G).
lis3dh.range = adafruit_lis3dh.RANGE_2_G

# Configure SD Card
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.D10)

sdcard = adafruit_sdcard.SDCard(spi, cs)
vfs = storage.VfsFat(sdcard)
storage.mount(vfs, "/sd")

########################
## Configure Displays ##
########################

# Configure DisplayBonnet
display = adafruit_is31fl3731.CharlieWing(i2c)
display.fill(10)

# Configure 7-segment display
segment = adafruit_ht16k33.segments.Seg7x4(i2c, address=0x70)
segment.print(8888)
segment.show()

###################
## Configure GPS ##
###################

# Define px and TX pins for the board's serial port connected to the GPS.
# These are the defaults you should use for the GPS FeatherWing.
# For other boards set px = GPS module TX, and TX = GPS module px pins.
RX = board.RX
TX = board.TX

# Create a serial connection for the GPS connection using default speed and
# a slightly higher timeout (GPS modules typically update once a second).
uart = busio.UART(TX, RX, baudrate=9600, timeout=30)
gps = adafruit_gps.GPS(uart, debug=False)

# Initialize the GPS module by changing what data it sends and at what rate.
# These are NMEA extensions for PMTK_314_SET_NMEA_OUTPUT and
# PMTK_220_SET_NMEA_UPDATERATE but you can send anything from here to adjust
# the GPS module behavior:
#   https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf
# Turn on the basic GGA and RMC info (what you typically want)
gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
# Set update rate to once a second (1hz) which is what you typically want.
gps.send_command(b'PMTK220,1000')

#################
## Vector Math ##
#################
def vector_mean(vector_list):
    """ Average a list of 3-vectors

    [(x1, y1, z1), ..., (xn, yn, zn)] -> (xm, ym, zm)
    """
    vlen = len(vector_list)
    mv = tuple(
        sum(v[ii] for v in vector_list) / vlen for ii in range(3)
    )
    return mv

def vector_multiply(s, v):
    """ Multiply a vector by a scalar
    """
    return tuple(s * vi for vi in v)

def vector_subtract(va, vb):
    """ Vector version of a - b
    """
    return tuple(a - b for a, b in zip(va, vb))

def dot(xv, yv):
    """ Dot product: xv.yv
    """
    return sum(x*y for x, y in zip(xv, yv))

def norm(v):
    """ Euclidean norm: sqrt(v.v)
    """
    mv = math.sqrt(dot(v, v))
    if mv == 0.0:
        return (0, 0, 0)
    else:
        return tuple(vi/mv for vi in v)

#############
## Classes ##
#############

# Letters for the display
Sy = [2, 2, 3, 4, 4, 4, 5, 6, 6]
Sx = [12, 13, 11, 11, 12, 13, 13, 11, 12]
Dy = [2, 2, 3, 3, 4, 4, 5, 5, 6, 6]
Dx = [11, 12, 11, 13, 11, 13, 11, 13, 11, 12]
Ky = [2, 3, 3, 4, 4, 5, 5, 6, 6]
Kx = [11, 11, 13, 11, 12, 11, 12, 11, 13]

class ScrollingDisplay():
    """ Main class for storing values and updating the display
    """
    def __init__(self):
        # Histopy length
        self.hlen = 10
        # Acceleration
        self.disp_g = [0]*self.hlen
        # Stroke
        self.disp_s = [0]*self.hlen
        # Do we have GPS?
        self.fix = False
        # Display mode
        self.mode = "k"
        # Frame number (alternates between 0 and 1)
        # Allows us to write the next frame without display
        self.f = 0
        # GPS ticker
        self.gi = 0

    def update_g(self, g):
        self.disp_g.pop(0)
        gpx = int((g - 1)*4/1.5)
        if gpx > 4:
            gpx = 4
        elif gpx < 0:
            gpx = 0
        self.disp_g.append(gpx + 1)

    def update_stroke(self, stroke):
        self.disp_s.pop(0)
        self.disp_s.append(stroke)

    def update_battery(self, battery_voltage):
        battery_display_height = int((battery_voltage-3.3)/(3.7-3.3) * display.height)
        self.b = battery_display_height

    def update_fix(self, gps_fix):
        self.fix = gps_fix

    def draw_frame(self):
        ''' Update the display
        Displays:
            - Current G's pulled (scrolling bar)
            - Mode (letter)
            - GPS fix
            - Battepy fraction
        '''
        frame = self.f
        display.frame(frame, show=False)
        display.fill(0)
        for ii in range(self.hlen):
            # Display the acceleration
            value = self.disp_g[ii]
            for jj in range(value+1):
                x = ii
                y = display.height - jj
                display.pixel(x, y, 20)

            # Display the shake
            if self.disp_s[ii]:
                display.pixel(x, 0, 50)

        # Display the battepy fraction
        for jj in range(self.b):
            x = 14
            y = display.height - jj
            display.pixel(x, y, 20)

        # Display the mode:
        # Split (S), Distance (D), or Stroke (K) rate
        if self.mode == "s":
            X, Y = Sx, Sy
            pass
        elif self.mode == "d":
            X, Y = Dx, Dy
            pass
        elif self.mode == "k":
            X, Y = Kx, Ky
        for x, y in zip(X, Y):
            display.pixel(x, y, 20)

        # Display the GPS fix
        Gx = [10, 11, 12, 13]
        Gy = [0, 0, 0, 0]
        if self.fix:
            for x, y in zip(Gx, Gy):
                display.pixel(x, y, 20)
        else:
            for x, y in zip(Gx, Gy):
                display.pixel(x, y, 0)
            display.pixel(Gx[self.gi], Gy[self.gi], 20)
            self.gi = (self.gi + 1) % 4

        # Show the new frame
        display.frame(frame, show=True)

        # Cycle to the next frame to draw in the background
        frame = 0 if frame else 1
        self.f = frame


##############
## Closures ##
##############


def GravityRemover(history=5):
    """ Track and remove the average component of N acceleration vectors

    Internally track the last N accleration vectors.
    Compute the average acceleration vector from these.
    Then subtract that from the current acceleration vector.
    And return this reduced acceleration vector.
    """
    hv = []
    def _call(v):

        if len(hv) < 1:
            v_reduced = (0, 0, 0)
        else:
            # Compute the direction of average acceleration
            a_hat = norm(vector_mean(hv))
            # Remove the component parallel to average accleration
            v_parallel = vector_multiply(dot(v, a_hat), a_hat)
            v_reduced = vector_subtract(v, v_parallel)
        # Update the histopy elements with the new vector
        hv.append(v)
        if len(hv) > history:
            hv.pop(0)
        return v_reduced

    return _call


def HistoryThresholder(threshold=1, history=5, mode="rising"):
    """ Pulse when something exceeds our threshold
    But maintain a history with hystersis so we don't accidentally trigger.
    """
    if mode == "rising":
        comparitor = lambda x, y: x > y
    elif mode == "falling":
        comparitor = lambda x, y: x < y
    else:
        raise ValueError("mode can be 'rising' or 'falling'")

    previous = []
    def _call(value):
        condition_met = comparitor(value, threshold)
        if condition_met and not any(previous):
            trigger = True
        else:
            trigger = False
        previous.append(condition_met)
        if len(previous) > history:
            previous.pop(0)
        return trigger

    return _call


def StrokeRate(history=3, forget=120):
    """ Compute stroke rate
    Inputs:
        history = remember this many strokes
        forget = forget strokes older than this time
    """
    # A single value becomes a local variable
    stroke_rate = [0]
    stroke_times = []
    def _call(stroke):
        currently = time.monotonic()
        # Drop old measurements
        if stroke_times:
            if (currently - stroke_times[0]) > forget:
                stroke_times.pop(0)
        # Add new measurements
        if stroke:
            stroke_times.append(currently)
            if len(stroke_times) > history:
                stroke_times.pop(0)
        # Update our rate
        strokes = len(stroke_times)
        if strokes > 1:
            delta_t = stroke_times[-1] - stroke_times[0]
            # strokes / delta_t [s] * 60 [s]/minute
            stroke_rate[0] = round(60 * strokes / delta_t, 1)
        else:
            stroke_rate[0] = 0
        return stroke_rate[0]

    return _call


###############
## Functions ##
###############

def format_log(*args):
    log = ",".join([str(a) for a in args]) + "\n"
    return log

def write_header(filename):
    print("Starting log file")
    header = format_log(
        "gps_time",
        "time",
        "battery_voltage",
        "ax",
        "ay",
        "az",
        "a_magnitude",
        "px",
        "py",
        "pz",
        "a_perpendicular",
        "shaken",
        "latitude",
        "longitude",
        "fix_quality",
        "satellites",
        "altitude_m",
        "speed_knots",
        "track_angle_deg",
    )
    with open("/sd/" + filename, "w") as f:
        f.write(header)


def write_data(filename, log):
    print("Writing to log...")
    # Display the GPS fix
    Wx = [10, 11, 12, 13]
    Wy = [1, 1, 1, 1]
    # Turn on write indicator
    for x, y in zip(Wx, Wy):
        display.pixel(x, y, 70)
    # Write data
    with open("/sd/" + filename, "a") as f:
        f.write(log)
        f.flush()
        log = ""
    # Turn off write indicator
    for x, y in zip(Wx, Wy):
        display.pixel(x, y, 0)
    print("Written")


##########
## MAIN ##
##########

multidisplay = ScrollingDisplay()
accleration_fixer = GravityRemover()
stroke_computer = HistoryThresholder()
rate_computer = StrokeRate()

# Find the number of files in the directopy
nfiles = len(os.listdir("/sd"))
filename = "row_{}.csv".format(nfiles+1)
write_header(filename)

log = ""
last_time = time.monotonic()
gps.update()
while True:
    # Wait until we have a fix
    t = time.monotonic()
    if t - last_time > 1:
        sentence = uart.readline()
        print(str(sentence, 'ascii').strip())
        gps.update()
        last_time = t

    if gps.has_fix:
        # We have a fix!
        gps_time = "{}/{}/{} {:02}:{:02}:{:02}".format(
            gps.timestamp_utc.tm_year,  # Grab parts of the time from the
            gps.timestamp_utc.tm_mon,   # struct_time object that holds
            gps.timestamp_utc.tm_mday,  # the fix time.  Note you might
            gps.timestamp_utc.tm_hour,  # not get all data like year, day,
            gps.timestamp_utc.tm_min,   # month!
            gps.timestamp_utc.tm_sec)
    else:
        gps_time = None

    # Get battepy voltage
    battery_voltage = get_voltage()

    # Now get acceleration
    a_vec = lis3dh.acceleration
    a_magnitude_squared = dot(a_vec, a_vec)
    # Compute the perpendicular vector
    p_vec = accleration_fixer(a_vec)
    p_magnitude_squared = dot(p_vec, p_vec)

    # Determine if we have a stroke and update the rate
    stroke = stroke_computer(p_magnitude_squared)
    rate = rate_computer(stroke)

    # Update the display
    multidisplay.update_g(p_magnitude_squared)
    multidisplay.update_stroke(stroke)
    multidisplay.update_battery(battery_voltage)
    multidisplay.update_fix(gps.has_fix)
    multidisplay.draw_frame()
    segment.print(rate)

    # Append to the log
    ax, ay, az = a_vec
    px, py, pz = p_vec
    log += format_log(
        gps_time,
        time,
        battery_voltage,
        ax,
        ay,
        az,
        a_magnitude_squared,
        px,
        py,
        pz,
        p_magnitude_squared,
        gps.latitude,
        gps.longitude,
        gps.fix_quality,
        gps.satellites,
        gps.altitude_m,
        gps.speed_knots,
        gps.track_angle_deg,
    )

    # If we have enough data write the log and reset it
    if len(log) > 2048:
        write_data(filename, log)
        log = ""

    # Small delay to keep things responsive but give time for interrupt processing.
    time.sleep(0.1)
