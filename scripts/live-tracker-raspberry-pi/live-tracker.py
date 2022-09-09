from subprocess import Popen, PIPE
import os.path
import io
import traceback
import time
import requests
import pandas
import pandas as pd
import strymread_lite

VIN_PATH="/etc/libpanda.d/vin"
PANDACORD_FOLDER="/var/panda/CyverseData/JmscslgroupData/PandaData/"
LIVE_TRACKER_TEMPORARY_FOLDER="/var/panda/CyverseData/JmscslgroupData/live_tracker_temporary_files/"
LAST_N_LINES=10000
WEB_PATH="http://ransom.isis.vanderbilt.edu/LIVE_VIEWER_SITE/rest.php"

def buf_count_newlines_gen(fname):
	def _make_gen(reader):
		while True:
			b = reader(2 ** 16)
			if not b: break
			yield b
	with open(fname, "rb") as f:
 		count = sum(buf.count(b"\n") for buf in _make_gen(f.raw.read))
	return count

def getFirstNLinesFromFile(path, n):
	assert os.path.exists(path), path + " file for head does not exist apparently!"
	p = Popen(['head', "-" + str(n), path], shell=False, stderr=PIPE, stdout=PIPE)
	res,err = p.communicate()
	if err:
		raise AssertionError("Head has failed with this error: " + err.decode())
	else:
		return res.decode()

def getLastNLinesFromFile(path, n):
	assert os.path.exists(path), path + " file for tail does not exist apparently!"
	p = Popen(['tail', "-" + str(n), path], shell=False, stderr=PIPE, stdout=PIPE)
	res,err = p.communicate()
	if err:
		raise AssertionError("Tail has failed with this error: " + err.decode())
	else:
		return res.decode()

def readAllFile(path):
	assert os.path.exists(path), path + " file does not exist apparently!"
	file = open(path, mode='r')
	res = file.read()
	file.close()
	return res

def getLastNCSV(path, n, toDataFrame=True):
	raw = ''
	if (buf_count_newlines_gen(path) <= n):
		raw = readAllFile(path)
	else:
		# Grab the header like the cheeky bois we are, so we don't have to guess what the column names are...
		header = getFirstNLinesFromFile(path, 1) + "\n"
		data = getLastNLinesFromFile(path, n)
		raw = header + data
	if toDataFrame:
		string_file = io.StringIO(raw)
		return pandas.read_csv(string_file, sep=",")
	return raw

def getVIN():
	return readAllFile(VIN_PATH)

#We grab the second to last row, just in case the last line is only half-flushed.
#I dunno if the code on the other side only does full-line flushes, but better safe than sorry.
def getGPSData(gps_file):
	return getLastNCSV(gps_file, 2)

def getGPSResultStr(gps_file):
	gps_data = getGPSData(gps_file)
	gpstime = str(gps_data.iloc[0]["Gpstime"])
	systime = str(gps_data.iloc[0]["Systime"])
	latitude = str(gps_data.iloc[0]["Lat"])
	longitude = str(gps_data.iloc[0]["Long"])
	altitude = str(gps_data.iloc[0]["Alt"])
	status = str(gps_data.iloc[0]["Status"])
	return ','.join([gpstime, systime, latitude, longitude, altitude, status])

def getAllFilesInFolder(path, substring=None, recursive=True):
	entries = [(os.path.join(path, f), f) for f in os.listdir(path)]
	files = []
	for (full_path, name) in entries:
		if (os.path.isdir(full_path) and recursive):
			files = files + getAllFilesInFolder(full_path, substring, recursive)
		elif (os.path.isfile(full_path)):
			if (substring is not None):
				if substring in name:
					files.append(full_path)
			else:
				files.append(full_path)
	return files

def determineCANAndGPSFiles():
	can_files = getAllFilesInFolder(PANDACORD_FOLDER, "CAN_Messages")
	gps_files = getAllFilesInFolder(PANDACORD_FOLDER, "GPS_Messages")
	can_files_mtime = [os.path.getmtime(f) for f in can_files]
	gps_files_mtime = [os.path.getmtime(f) for f in gps_files]
	assert len(can_files) > 0, "No CAN file found in folder!"
	assert len(gps_files) > 0, "No GPS file found in folder!"
	most_recent_time_can = max(can_files_mtime)
	most_recent_time_gps = max(gps_files_mtime)
	assert (time.time() - most_recent_time_can) <= 60, "CAN data more than a minute stale!"
	assert (time.time() - most_recent_time_gps) <= 60, "GPS data more than a minute stale!"
	chosen_can = can_files[can_files_mtime.index(most_recent_time_can)]
	chosen_gps = gps_files[gps_files_mtime.index(most_recent_time_gps)]
	return chosen_can, chosen_gps

def getCANData(can_file):
	raw = getLastNCSV(can_file, LAST_N_LINES, toDataFrame=False)
	tmp_can_filename = os.path.join(LIVE_TRACKER_TEMPORARY_FOLDER, os.path.basename(can_file))
	file = open(tmp_can_filename, mode='w+')
	file.write(raw)
	file.flush()
	file.close()
	return strymread_lite.strymread_lite(tmp_can_filename)

def getCANValue(strym_obj, method):
	try:
		frame = getattr(strym_obj, method)()
		return str(frame.iloc[-1]["Message"]) if len(frame) > 0 else "NULL"
	except Exception as e:
		print(e)
		print(traceback.print_exc())
		return "NULL"

def getCANResultStr(can_file):
	strym_obj = getCANData(can_file)
	speed = getCANValue(strym_obj, "speed")
	accel = getCANValue(strym_obj, "accelx")
	relative_speed = getCANValue(strym_obj, "relative_leadervel")
	relative_distance = getCANValue(strym_obj, "lead_distance")
	acc_status = getCANValue(strym_obj, "acc_state")
	return ','.join([speed, accel, relative_speed, relative_distance, acc_status])

while True:
	try:
		vin = getVIN()
		can_file, gps_file = determineCANAndGPSFiles()
		print(can_file, gps_file)
		gps = getGPSResultStr(gps_file)
		can = getCANResultStr(can_file)
		data_str = "?circles," + vin + "," + gps + "," + can
		get_str = WEB_PATH + data_str
		print(get_str)
		print(requests.get(get_str))
	except Exception as e:
		print(e)
		traceback.print_exc()
		print("Not uploading any data at this time.")
		time.sleep(9.0) # Just to make it spam less at startup.
	time.sleep(1.0)
