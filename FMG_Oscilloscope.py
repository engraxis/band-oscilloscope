# Abdullah Tahir atah@es.aau.dk
# 05 Apr 2025

import serial
import threading
import matplotlib.pyplot as plt
from collections import deque
import numpy as np

# ESP's serial port
esp_port = '/dev/tty.usbserial-0001'

# Global variables
num_fsrs = 8
display_length = 100
data_queue_shank = deque([np.zeros(num_fsrs) for _ in range(display_length)], maxlen=display_length)
data_queue_thigh = deque([np.zeros(num_fsrs) for _ in range(display_length)], maxlen=display_length)
stop_threads = False

print('Press Esc button to close the program.')

# Function to read data from the serial port
def read_serial_data():
    global stop_threads
    ser = serial.Serial(esp_port, 230400)
    while not stop_threads:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()  # Read and decode data from serial
            fsr = data.split(',')
            if len(fsr) < 10:
                continue
            fsr = [float(x) for x in fsr[1:num_fsrs+1]]
            fsr = [fsr[3], fsr[2], fsr[1], fsr[0], fsr[7], fsr[6], fsr[5], fsr[4]]
            if data.startswith('t'):
                data_queue_thigh.append(np.array(fsr))
            elif data.startswith('s'):
                data_queue_shank.append(np.array(fsr))
            else:
                print('CAUTION: Error in data format.')
    ser.close()
    #time.sleep(0.1) # The data processing after serial read is already taking time. Testing shows that let's not add this delay.

# Function to plot the data (called on the main thread)
def plot_data():
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots(2, 1, figsize=(11, 7))
    fig.suptitle('FMG Oscilloscope', fontweight='bold', y=0.95)
    fig.canvas.mpl_connect('key_press_event', lambda event: globals().__setitem__('stop_threads', True) if event.key == 'escape' else None)
    titles = ['Thigh Band', 'Shank Band']
    colors = ['blue', 'green', 'red', 'darkorange', 'black', 'darkviolet', 'darkcyan', 'tomato']
    while not stop_threads:
        if len(data_queue_thigh) > 0 and len(data_queue_shank) > 0:
            for idx in range(2):
                ax[idx].cla()
                ax[idx].set_ylim(0, 550)
                ax[idx].set_xlim(0, display_length + 20)
                data = np.array(data_queue_thigh) if idx == 0 else np.array(data_queue_shank)
                y_pos = 0.9
                for fsr in range(num_fsrs):
                    ax[idx].plot(data[:, fsr], color=colors[fsr], label=f'FSR{fsr}')
                    ax[idx].text(0.98, y_pos, f'FSR {fsr+1} = {float(data[-1, fsr]):05.1f}', transform=ax[idx].transAxes, fontsize=11, verticalalignment='top', horizontalalignment='right', color=colors[fsr], multialignment='left')
                    y_pos -= 0.1
                ax[idx].set_title(titles[idx])
                plt.draw()  # Redraw the plot
            plt.pause(0.001)  # Pause to update the plot

# Start the threads
def start_threads():
    # Create and start the serial reading thread
    serial_thread = threading.Thread(target=read_serial_data)
    serial_thread.daemon = True
    serial_thread.start()

    try:
        # Start plotting from the main thread (not in a separate thread)
        plot_data()
    except KeyboardInterrupt:
        global stop_threads
        stop_threads = True
        print("Stopping threads...")


# Main function to initiate everything
if __name__ == "__main__":
    start_threads()
