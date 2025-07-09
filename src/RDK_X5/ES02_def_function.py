import sbus_out
import time
from threading import Thread, Lock



JOYSTICK_DEFAULT = 1000 #Mid-value of joystick channel

channel_holding_time = [0]*4 #After the joystick channel changes, the intermediate value is restored automatically and the unit is milliseconds.

#
def advance(distance):
	print(f"advancce {distance} ")
	sbus_out.channels[2] 	= 1300
	channel_holding_time[2] = 0.5 + distance * 1
	if(channel_holding_time[2]>1) :
		channel_holding_time[2] = 1
	return 0

#
def retreat(distance):
	print(f"retreat {distance} ")
	sbus_out.channels[2] 	= 700
	channel_holding_time[2] = 0.5 + distance * 1
	if(channel_holding_time[2]>1) :
		channel_holding_time[2] = 1
	return 0

#
def left_rotation(angle):
	print(f"left_rotation {angle} ")
	sbus_out.channels[3] 	= 700
	channel_holding_time[3] = angle * 0.005
	return 0

#
def right_rotation(angle):
	print(f"right_rotation {angle} ")
	sbus_out.channels[3] 	= 1300
	channel_holding_time[3] = angle * 0.005
	return 0

#
def leg_length(length):
	print(f"leg_length {length} ")
	length = length * 90
	if 	length >  666 :
		length = 666
	if 	length < -666 :
		length = -666
	sbus_out.channels[1] 	= 1000 + length
	print(f"ch[2] = {sbus_out.channels[1]} ")
	return 0



def ch_timing_thread():

	print("The channel timing thread has been started.")
	last_time = time.time()
	while True:
		# 
		for i in range(2,4):
			if channel_holding_time[i] > 0 :
				channel_holding_time[i] -= 0.1
			else :
				channel_holding_time[i] = 0
				sbus_out.channels[i] = JOYSTICK_DEFAULT

		time.sleep(0.1)





# Channel timing thread
def start_ES02_ch_timing_processing_thread():
	print("Start the channel timing thread")
	# Start the output thread
	output_thread = Thread(target=ch_timing_thread)
	output_thread.daemon = True
	output_thread.start()
	
def init_sbus():
	#初始化操纵杆通道为Initialize the joystick channel to the middle position中位
	for i in range(0,4):
		sbus_out.channels[i] = 1000
	sbus_out.channels[4] = 1000
	print("Start the SBUS output thread")
	sbus_out.start_output_thread()

def test():
	advance(10)
	retreat(11)
	left_rotation(12)
	right_rotation(13)
	leg_length(14)

if __name__ == '__main__':
	test()