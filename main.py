import math
import utime
import _thread
from m5stack import lcd
from micropyGPS import MicropyGPS
from machine import UART, I2C, Pin
from mpu9250 import MPU9250

target_coords = (35.388127, 139.427293)  # replace with where you want to go
current_coord = (0, 0)  # lat, lon
timestamp = ("N/A", 0, 0, 0)  # date_str, h, m, s
corrected_gauss = (0, 0, 0)
azimuth_arg = 7.2  # default to the arg for Japan
azimuth = 0

# gps
gps_serial = UART(2, tx=17, rx=16, baudrate=9600)
gps_parser = MicropyGPS(9, 'dd')  # time offset to JST (+9)

# magnetometer
mpu_i2c = I2C(sda=Pin(21), scl=Pin(22))
mpu = MPU9250(mpu_i2c)
gauss_min = gauss_max = mpu.magnetic


def calcAndUpdateGauss(x, y, z):
  global gauss_min, gauss_max, corrected_gauss

  # update max and min value of gauss
  gauss_max = (
      max(x, gauss_max[0]),
      max(y, gauss_max[1]),
      max(z, gauss_max[2]),
  )
  gauss_min = (
      min(x, gauss_min[0]),
      min(y, gauss_min[1]),
      min(z, gauss_min[2]),
  )

  # offset
  offset_x = (gauss_max[0] + gauss_min[0]) / 2
  offset_y = (gauss_max[1] + gauss_min[1]) / 2
  offset_z = (gauss_max[2] + gauss_min[2]) / 2

  # scale
  # add 0.0001 to avoid zero division error
  delta_x = (gauss_max[0] - gauss_min[0]) / 2 + 0.0001
  delta_y = (gauss_max[1] - gauss_min[1]) / 2 + 0.0001
  delta_z = (gauss_max[2] - gauss_min[2]) / 2 + 0.0001
  delta = (delta_x + delta_y + delta_z) / 3 + 0.0001
  scale_x = delta / delta_x
  scale_y = delta / delta_y
  scale_z = delta / delta_z

  # update coords
  corrected_gauss = ((x - offset_x) * scale_x, (y - offset_y) * scale_y,
                     (z - offset_z) * scale_z)


def calcAndUpdateAzimuthArgument(lat, lon):
  """
    国土地理院の偏角近似式
    https://vldb.gsi.go.jp/sokuchi/geomag/menu_04/index.html
    """

  global azimuth_arg

  delta_phi = lat - 37.0
  delta_lambda = lon - 138.0
  azimuth_arg = 7.95335 \
      + 0.31253 * delta_phi \
      - 0.11268 * delta_lambda \
      - 0.00098 * delta_phi**2 \
      - 0.00023 * delta_phi * delta_lambda \
      - 0.00965 * delta_lambda**2


def calcAndUpdateAzimuth():
  global azimuth

  gauss_x = corrected_gauss[0]
  gauss_y = corrected_gauss[1]
  azimuth = (math.atan2(gauss_y, gauss_x) *
             (180 / math.pi) + 270 + azimuth_arg) % 360


def updateGPS():
  global timestamp, current_coord

  byte_length = gps_serial.any()
  if byte_length > 0:
    for nmea_char in gps_serial.read(byte_length):
      gps_parser.update(chr(nmea_char))

    current_date = gps_parser.date_string()
    timestamp = (current_date, gps_parser.timestamp[0], gps_parser.timestamp[1],
                 gps_parser.timestamp[2])
    current_coord = (gps_parser.latitude[0], gps_parser.longitude[0])

    calcAndUpdateAzimuthArgument(*current_coord)


def calcTargetRadian(coord, target_coord):
  (x, y) = coord
  (tx, ty) = target_coord
  return 90 - math.atan2(
      math.sin(tx - x),
      math.cos(y) * math.tan(ty) - math.sin(y) * math.cos(tx - x))


def printResult():
  lcd.clear()
  lcd.print('Date: ' + timestamp[0], 10, 10)
  lcd.print('Blue = Destination, Red = North', 10, 20)
  # lcd.print(
  #     'Location: ' + str(current_coord[0]) + ', ' + str(current_coord[1]), 10,
  #     30)
  # lcd.print("Azimuth Arg: {}".format(azimuth_arg), 10, 50)
  # lcd.print("Azimuth: {}".format(azimuth), 10, 70)

  rad = calcTargetRadian(current_coord, target_coords)
  # lcd.print("Rad: {}".format(rad), 10, 90)
  # lcd.print("Target Rad: {}".format(str((rad + azimuth) % 360)), 10, 110)
  lcd.lineByAngle(161, 119, 0, 120, round(azimuth), lcd.RED)
  lcd.lineByAngle(160, 120, 0, 120, round(azimuth), lcd.RED)
  lcd.lineByAngle(161, 121, 0, 120, round(azimuth), lcd.RED)

  lcd.lineByAngle(161, 119, 0, 120, round(rad + azimuth) % 360, lcd.BLUE)
  lcd.lineByAngle(160, 120, 0, 120, round(rad + azimuth) % 360, lcd.BLUE)
  lcd.lineByAngle(161, 121, 0, 120, round(rad + azimuth) % 360, lcd.BLUE)

  print('Date: ' + timestamp[0])
  print('Location: ' + str(current_coord[0]) + ', ' + str(current_coord[1]))
  print("Azimuth Arg: {}".format(azimuth_arg))
  print("Azimuth: {}".format(azimuth))
  print("Rad: {}".format(rad))
  print("Target Rad: {}".format(str((rad + azimuth) % 360)))


def main():
  print("Navicompass: start")

  while True:
    # sleep 100ms to prevent error
    utime.sleep_ms(100)

    # update magnetometer values
    calcAndUpdateGauss(*mpu.magnetic)

    # update azimuth
    calcAndUpdateAzimuth()

    # update gps location
    updateGPS()

    # print result
    printResult()


main_thread = _thread.start_new_thread("Main", main, ())
