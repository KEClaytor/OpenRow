
import os
import time
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

# Battery pin
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

# Define RX and TX pins for the board's serial port connected to the GPS.
# These are the defaults you should use for the GPS FeatherWing.
# For other boards set RX = GPS module TX, and TX = GPS module RX pins.
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
        # History length
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
        gpx = int((g - 1)*4/0.5)
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
            - Battery fraction
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

        # Display the battery fraction
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

class StrokeCounter():
    """ Compute stroke rate
    """

    def __init__(self):
        # TODO: Adjustable
        self.threshold = 1.2
        self.n_strokes = 3
        # Final result we want
        self.rate = 0
        # History
        self.strokes = []
        self.stimes = []

    def _calculate_rate(self):
        strokes = len(self.strokes)
        if strokes > 1:
            time = self.stimes[-1] - self.stimes[0]
            # strokes / time [s] * 60 [s]/minute
            self.rate = round(60 * strokes / time, 1)

    def update(self, stroke):

        if stroke:
            self.strokes.append(stroke)
            self.stimes.append(time.monotonic())

        if len(self.strokes) > self.n_strokes:
            self.strokes.pop(0)
            self.stimes.pop(0)

        self._calculate_rate()

        segment.print(self.rate)

sd = ScrollingDisplay()
sc = StrokeCounter()

# Find the number of files in the directory
nfiles = len(os.listdir("/sd"))
filename = "row_{}.csv".format(nfiles+1)

def write_header(filename):
    print("Starting log file")
    with open("/sd/" + filename, "w") as f:
        f.write("gps_time,time,battery,g,shaken,latitude,longitude,quality,satellites,altitude,speed,track_angle\n")

def format_log(*args):
    log = ",".join([str(a) for a in args]) + "\n"
    return log


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

# Only log for a specified duration
log = ""
last_time = time.monotonic()
gps.update()
write_header(filename)
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

    # Get battery voltage
    v_batt = get_voltage()

    # Now get acceleration
    x, y, z = lis3dh.acceleration
    a_mag = (x**2 + y**2 + z**2)**0.5
    G = a_mag / adafruit_lis3dh.STANDARD_GRAVITY
    shaken = lis3dh.shake(shake_threshold=11)

    # Update the display
    sd.update_g(G)
    sd.update_stroke(shaken)
    sd.update_battery(v_batt)
    sd.update_fix(gps.has_fix)
    sd.draw_frame()

    # Update stroke rate
    sc.update(shaken)

    log += format_log(
        gps_time,
        t,
        v_batt,
        G,
        shaken,
        gps.latitude,
        gps.longitude,
        gps.fix_quality,
        gps.satellites,
        gps.altitude_m,
        gps.speed_knots,
        gps.track_angle_deg,
    )

    if len(log) > 2048:
        write_data(filename, log)
        log = ""

    # Small delay to keep things responsive but give time for interrupt processing.
    time.sleep(0.1)
