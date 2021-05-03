import argparse
import math
import os
import random
import sys
import subprocess
import struct
# Suppress hello from pygame so that stdout is clean
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame  # noqa


# Structs used to pack/unpack the API messages
# milliseconds [2 bytes]
# recovery_x error [2 bytes]
# wind_x [4 bytes]
# wind_y [4 bytes]
# recovery_y error [1 byte]
# 31 lidar samples [31 bytes]
TELEMETRY_STRUCT = struct.Struct(">Hhffb31B")
COMMAND_STRUCT = struct.Struct(">fB3s")
obstacle = []
delivery = []
roi = {}
def compute_des_y(lidar, x_err, y_err):
	if x_err < 80:
		return 0
	else:
		fac = 0.0175
		des = y_err
		for i in lidar:
			if i > 0:
				x = x_err - i
				y = int(y_err - (lidar.index(i)-15)*fac*i)
				if x in roi.keys():
					y_temp = roi.get(x)
					y = y_temp.append(y)
				else:
					y = [y]
				roi[x]=y
				print(roi, file = sys.stderr)
		'''try:
			m = min(i for i in lidar if i > 0)
			i = lidar.index(m)
			des = y_err + (i-15)*fac*lidar[i]
			if (lidar[i]<100):
				if (abs(lidar[(i-1)%31]-lidar[i])<4) or (abs(lidar[(i+1)%31]-lidar[i])<4):
					obstacle.append([x_err - lidar[i],des])
					des = des + 4
				else:
					delivery.append([x_err - lidar[i], des])
		except:
			des = y_err'''
		return des

def compute_lat_arspd(y_err, y_err_last, y_err_sum):
	# PID Gains
	kp = 5
	kd = 5
	ki = 0.2
	desired = 20
	lateral_arspd = float(kp*y_err - kd*(y_err-y_err_last) + ki*y_err_sum)
	return lateral_arspd

def compute_drop(drop):
	result = 1
	if result == 1:
		return validate_drop(drop)
	else:
		return result

def validate_drop(flag):
	if flag == 0:
		drop = 1
	else:
		drop = 0
	return drop

drop = 0
y_err_last = 0
y_err_sum = 0
desired = 0
while True:

	abc = sys.stdin.buffer.read(TELEMETRY_STRUCT.size)
	data = TELEMETRY_STRUCT.unpack(abc)

	if (data ==""):
		continue
	
	[t_step, x_err, w_x, w_y, y_err] = data[0:5]
	lidar = data[5:36]

	drop = compute_drop(drop)

	desired = compute_des_y(lidar, x_err, y_err)

	y_err = y_err - desired

	lat = compute_lat_arspd(y_err, y_err_last, y_err_sum)
	y_err_last = y_err
	y_err_sum = y_err_sum + y_err

	command = COMMAND_STRUCT.pack(lat, drop, b'3s')
	sys.stdout.buffer.write(command)
	sys.stdout.flush()
