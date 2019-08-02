
import os
import time

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

# Battery pin
vbat_pin = analogio.AnalogIn(board.VOLTAGE_MONITOR)

def get_voltage():
    return (vbat_pin.value * 3.3) / 65536 * 2

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

# Configure DisplayBonnet
display = adafruit_is31fl3731.CharlieWing(i2c)

# Configure GPS

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

##############
#### MAIN ####
##############

class scrollingDisplay():
    """ Main class for storing values and updating the display
    """
    def __init__(self):
        self.dg = [0]*12
        self.ds = [0]*12
        self.f = 0

    def update_g(self, g):
        self.dg.pop(0)
        gpx = int((g - 1)/0.5 * 4)
        if gpx > 4:
            gpx = 4
        elif gpx < 0:
            gpx = 0
        self.dg.append(gpx + 1)

    def update_s(self, s):
        self.ds.pop(0)
        self.ds.append(s)

    def update_b(self, b):
        db = int((b-3.3)/(3.7-3.3) * 7)
        self.b = db

    def draw_frame(self):
        ''' Update the display
        Show: current g's, shaken, and battery %
        '''
        frame = self.f
        display.frame(frame, show=False)
        display.fill(0)
        for ii in range(12):
            # Display the acceleration
            value = self.dg[ii]
            for jj in range(value+1):
                x = ii
                y = display.height - jj
                display.pixel(x, y, 20)

            # Display the shake
            if self.ds[ii]:
                display.pixel(x, 0, 50)

        # Display the battery fraction
        for jj in range(self.b):
            x = 14
            y = display.height - jj
            display.pixel(x, y, 20)

        # Show the new frame
        display.frame(frame, show=True)
        frame = 0 if frame else 1
        self.f = frame

sd = scrollingDisplay()
# Find the number of files in the directory
nfiles = len(os.listdir("/sd"))
filename = "data_{}.txt".format(nfiles+1)

# Wait until we have a fix
start_time = time.monotonic()
with open("/sd/gps.txt", "w") as f:
    while True:
        sentence = uart.readline()
        print(str(sentence, 'ascii').strip())
        f.write(sentence)
        gps.update()
        elapsed = time.monotonic() - start_time
        if not gps.has_fix:
            print('Waiting for fix (%ds)...' % elapsed)
            if gps.fix_quality:
                print('    quality: %f' % gps.fix_quality)
            time.sleep(1)
            continue
        else:
            print("Fix acquired in %ds." % elapsed)
            break

# # Use the date as a filename
filename = "{}-{}-{}T{:02}{:02}.csv".format(
            gps.timestamp_utc.tm_year,
            gps.timestamp_utc.tm_mon,
            gps.timestamp_utc.tm_mday,
            gps.timestamp_utc.tm_hour,
            gps.timestamp_utc.tm_min
            )

# Only log for a specified duration
duration = 60
start_time = time.monotonic()
with open("/sd/" + filename, "w") as f:
    # Write header
    print("Logging...")
    f.write("gps_time,time,battery,g,shaken,latitude,longitude,quality,satellites,altitude,speed,track_angle\n")
    while True:
        # Wait until we have a fix
        gps.update()
        t = time.monotonic()

        if not gps.has_fix:
            # Try again if we don't have a fix yet.
            print('Lost fix, re-waiting for fix...')
            time.sleep(1)
            continue

        # We have a fix! (gps.has_fix is true)
        time_str = "{}/{}/{} {:02}:{:02}:{:02}".format(
            gps.timestamp_utc.tm_year,  # Grab parts of the time from the
            gps.timestamp_utc.tm_mon,   # struct_time object that holds
            gps.timestamp_utc.tm_mday,  # the fix time.  Note you might
            gps.timestamp_utc.tm_hour,  # not get all data like year, day,
            gps.timestamp_utc.tm_min,   # month!
            gps.timestamp_utc.tm_sec)

        # Get battery voltage
        v_batt = get_voltage()

        # Now get acceleration
        x, y, z = lis3dh.acceleration
        a_mag = (x**2 + y**2 + z**2)**0.5
        G = a_mag / adafruit_lis3dh.STANDARD_GRAVITY
        shaken = lis3dh.shake(shake_threshold=11)
        sd.update_g(G)
        sd.update_s(shaken)
        sd.update_b(v_batt)

        # Update the display
        sd.draw_frame()

        # Start writing to the file
        f.write("%s,%f,%f,%f,%d,%f,%f,%f," %
            (
                time_str, t, v_batt, G, shaken,
                gps.latitude, gps.longitude, gps.fix_quality,
            )
        )
        if gps.satellites is not None:
            f.write("%d," % gps.satellites)
        else:
            f.write("None,")
        if gps.altitude_m is not None:
            f.write("%f," % gps.altitude_m)
        else:
            f.write("None,")
        if gps.speed_knots is not None:
            f.write("%f," % gps.speed_knots)
        else:
            f.write("None,")
        if gps.track_angle_deg is not None:
            f.write("%f" % gps.track_angle_deg)
        else:
            f.write("None")
        f.write("\n")

        # Small delay to keep things responsive but give time for interrupt processing.
        time.sleep(0.1)

        # If we have recorded long enough, stop
        if (t - start_time) > duration:
            break

print("Finished.")
