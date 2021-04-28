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

def compute_lat_arspd():
	lateral_arspd = float(0.0)
	return lateral_arspd

def compute_drop(flag):
	if flag == 0:
		drop = 1
	else:
		drop = 0
	return drop

drop = 0

while True:

	abc = sys.stdin.buffer.read(TELEMETRY_STRUCT.size)
	data = TELEMETRY_STRUCT.unpack(abc)
	#print(data, file = sys.stderr)

	if (data ==""):
		continue
	
	[t_step, x_err, w_x, w_y, y_err] = data[0:5]
	lidar = data[6:36]

	drop = compute_drop(drop)

	lat = compute_lat_arspd()
	#lat = (float)(0.0)
	#dr = int(1)

	command = COMMAND_STRUCT.pack(lat, drop, b'3s')
	sys.stdout.buffer.write(command)
	sys.stdout.flush()
