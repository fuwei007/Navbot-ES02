#!/usr/bin/env python3

import sys
# import signal
import os
import time
import serial
import serial.tools.list_ports
from threading import Thread, Lock

# SBUS Protocol constant
SBUS_HEADER = 0x0F
SBUS_FOOTER = 0x00
SBUS_CHANNELS = 16
SBUS_MIN = 0
SBUS_MAX = 2047
OUTPUT_FREQ = 42  #(Hz) 
OUTPUT_PERIOD = 1.0 / OUTPUT_FREQ  #Output period (seconds)

#
channels = [333] * SBUS_CHANNELS  # Initialize all channels to 333.
channels_lock = Lock()  # Lock for thread safety

def set_ch(ch,value):
    channels[ch] = value
    
def start_output_thread():
    output_thread = Thread(target=sbus_output)
    output_thread.daemon = True
    output_thread.start()

def encode_sbus(channels):
    """Encode the values of 16 channels into the data format of the SBUS protocol"""
    data = [SBUS_HEADER] + [0] * 23 + [SBUS_FOOTER]
    for i in range(SBUS_CHANNELS):
        value = max(SBUS_MIN, min(SBUS_MAX, channels[i]))
        value -= SBUS_MIN
        for j in range(11):
            bit = (value >> j) & 1
            byte_index = 1 + (i * 11 + j) // 8
            bit_index = (i * 11 + j) % 8
            if bit:
                data[byte_index] |= (1 << bit_index)
            else:
                data[byte_index] &= ~(1 << bit_index)
    return bytes(data)

def sbus_output():
    uart_dev = "/dev/ttyS1"
    baudrate = 100000
    
    try:
        ser = serial.Serial(uart_dev, baudrate, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_TWO, timeout=1)
    except Exception as e:
        print(f"Serial port opening failed: {e}")
        return

    print(f"SBUS output has been activated, frequency: {OUTPUT_FREQ}Hz")
    
    last_time = time.time()
    
    while True:
        try: 
            # Copy the current channel value (thread-safe)
            with channels_lock:
                current_channels = channels.copy()
            
            # Encode and send SBUS data
            sbus_data = encode_sbus(current_channels)
            ser.write(sbus_data)
            
            # Control the output frequency
            current_time = time.time()
            elapsed = current_time - last_time
            if elapsed < OUTPUT_PERIOD:
                time.sleep(OUTPUT_PERIOD - elapsed)
            last_time = current_time
            
        except Exception as e:
            print(f"Error occurred while sending data: {e}")
            break

    ser.close()

if __name__ == '__main__':
    start_output_thread()
