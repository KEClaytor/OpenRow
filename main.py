
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

##############
## Geo Math ##
##############

def calculate_distance(lat0, lon0, lat1, lon1):
    """ Compute distance in m between to latitude and longitude points

    Approximation only, good if
        1. the points are not separated too much (we're talking km usually)
        2. you are not near the poles
    """
    deglen = 111.1*1e3
    x = lat1 - lat0
    y = (lon1 - lon0)*math.cos(lat0*math.pi/180)
    d = deglen*math.sqrt(x*x + y*y)
    return d

#############
## Classes ##
#############

# GPS Indicator pixels
Gx = [10, 11, 12, 13]
Gy = [0, 0, 0, 0]
# DIsk write indicator pixels
Wx = [10, 11, 12, 13]
Wy = [1, 1, 1, 1]
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
        # Display modes and tracking index
        self.mi = 0
        self.modes = ["k", "s", "d"]
        # Switch display modes this many seconds
        self.dt = 5
        self.mt = time.monotonic()
        # Current stats
        self.rate = 0
        self.split = 0
        self.distance = 0
        # Frame number (alternates between 0 and 1)
        # Allows us to write the next frame without display
        self.f = 0
        # GPS ticker
        self.gi = 0

    def update_g(self, g):
        self.disp_g.pop(0)
        gpx = int((g - 1)*4/40.0)
        if gpx > 4:
            gpx = 4
        elif gpx < 0:
            gpx = 0
        self.disp_g.append(gpx + 1)

    def update_stats(self, rate, split, distance):
        self.rate = rate
        self.split = split
        self.distance = distance

    def update_stroke(self, stroke):
        self.disp_s.pop(0)
        self.disp_s.append(stroke)

    def update_battery(self, battery_voltage):
        # Convert battery voltage to display height in pixels
        self.b = int((battery_voltage-3.3)/(3.7-3.3) * display.height)

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
        currently = time.monotonic()

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
        mode = self.modes[self.mi]
        if mode == "s":
            X, Y = Sx, Sy
            segment.print(self.split)
        elif mode == "d":
            X, Y = Dx, Dy
            segment.print(self.distance)
        elif mode == "k":
            X, Y = Kx, Ky
            segment.print(self.rate)
        for x, y in zip(X, Y):
            display.pixel(x, y, 20)
        # Do we need to change modes?
        if (currently - self.mt) > self.dt:
            self.mi = (self.mi + 1) % 3
            self.mt = currently

        # Display the GPS fix
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


def HistoryThresholder(threshold=20.0, mode="rising"):
    """ Pulse when something exceeds our threshold and we're on a rising edge
    """
    if mode == "rising":
        comparitor = lambda x0, x1, thresh: x1 > thresh and x0 < thresh
    elif mode == "falling":
        comparitor = lambda x0, x1, thresh: x1 < thresh and x0 > thresh
    else:
        raise TypeError("mode can be 'rising' or 'falling'")

    previous = [0, 0]
    def _call(value):
        previous.append(value)
        previous.pop(0)

        return comparitor(previous[0], previous[1], threshold)

    return _call


def StrokeRate(history_num=3, history_time=20):
    """ Compute stroke rate

    We want a fairly updated (instaneous) stroke rate
    But we also don't want to compute rates from strokes separated by a long time.
    Inputs:
        history_num = Drop more than history_num strokes
        history_time = Drop strokes older than this time
    Internally:
        stroke_data = List of tuples [(time, distance)]
    """
    # A single value becomes a local variable
    stroke_data = []

    def _call(stroke, distance):
        """ Update the stroke data and return current rate and split.
            stroke = has a stroke occured?
            distance = current distance travelled
        """
        currently = time.monotonic()

        # Only update the data on a stroke event
        if stroke:
            stroke_data.append((currently, distance))

        # Drop points based on number
        while len(stroke_data) > history_num:
            stroke_data.pop(0)

        # Drop points based on time
        while stroke_data and (currently - stroke_data[0][0] > history_time):
            stroke_data.pop(0)

        # Update our rate
        strokes = len(stroke_data)
        if strokes > 1:
            time0, dist0 = stroke_data[0]
            time1, dist1 = stroke_data[-1]
            delta_t = time1 - time0
            delta_d = dist1 - dist0

            # Stroke rate = strokes / delta_t [s] * 60 [s]/minute
            stroke_rate = round(60 * strokes / delta_t, 1)

            # Split = time for 500 m
            if delta_d > 0:
                split_time = 500.0 * delta_t / delta_d
            else:
                split_time = 0
        else:
            stroke_rate, split_time = 0, 0

        return (stroke_rate, split_time)

    return _call


def DistanceTracker():
    """ Compute our running distance
    """
    ilat = [None, None]
    ilon = [None, None]
    total_distance = 0

    def _call(lat, lon):
        # Update the lat / lon arrays
        ilat.append(lat)
        ilat.pop(0)
        ilon.append(lon)
        ilon.pop(0)

        if ilat[0] is not None and ilat[1] is not None:
            new_distance = calculate_distance(ilat[0], ilon[0], ilat[1], ilon[1])
            total_distance += new_distance

        return total_distance

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
distance_tracker = DistanceTracker()
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
        distance = distance_tracker(gps.latitude, gps.longitude)

        gps_time = "{}/{}/{} {:02}:{:02}:{:02}".format(
            gps.timestamp_utc.tm_year,  # Grab parts of the time from the
            gps.timestamp_utc.tm_mon,   # struct_time object that holds
            gps.timestamp_utc.tm_mday,  # the fix time.  Note you might
            gps.timestamp_utc.tm_hour,  # not get all data like year, day,
            gps.timestamp_utc.tm_min,   # month!
            gps.timestamp_utc.tm_sec)
    else:
        distance = distance_tracker(None, None)
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
    rate, split = rate_computer(stroke, distance)

    # Update the display
    multidisplay.update_g(p_magnitude_squared)
    multidisplay.update_stroke(stroke)
    multidisplay.update_battery(battery_voltage)
    multidisplay.update_fix(gps.has_fix)
    multidisplay.update_stats(rate, split, distance)
    multidisplay.draw_frame()

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
