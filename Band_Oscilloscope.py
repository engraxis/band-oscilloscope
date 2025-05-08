# Abdullah Tahir atah@es.aau.dk

# View the serial ports using terminal ls /dev/tty.*

import serial
import threading
import matplotlib
matplotlib.use('QtAgg')  # This backend will allow to have the title of the figure.
import matplotlib.pyplot as plt
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

# ESP's serial port
esp_port = '/dev/tty.usbserial-0001'

# Global standard variables.
num_fsrs = 8
num_IMUs = 3
data_length = 140

# Data containers for the respective data type:
fsr_queue = deque([np.zeros(num_fsrs) for _ in range(data_length)], maxlen=data_length)
euler_queue = deque([np.zeros(num_IMUs) for _ in range(data_length)], maxlen=data_length)
gravity_queue = deque([np.zeros(num_IMUs) for _ in range(data_length)], maxlen=data_length)
gyro_queue = deque([np.zeros(num_IMUs) for _ in range(data_length)], maxlen=data_length)
acc_queue = deque([np.zeros(num_IMUs) for _ in range(data_length)], maxlen=data_length)

stop_threads = False
data_lock = threading.Lock()

print('[Info] Press Esc button to close the program.')

band_id = 'z'
while band_id != 's' and band_id != 't':
    band_id = input('>> Which band\'s data you want to view, s: shank, t: thigh? [s/t] ')
    band_id = band_id.lower()


def read_serial_data():
    """
    Function reads the data from the serial port and extracts respective components of data.
    Runs the thread to continuously read the serial port.
    Data is added in the queue with the data_lock.
    :return: nothing
    """
    global stop_threads
    try:
        ser = serial.Serial(esp_port, 230400)
    except Exception as e:
        print(f"Error connecting with serial port. {e}\nExiting.")
        exit(0)
    while not stop_threads:
        if ser.in_waiting > 0:
            raw_data = ser.readline().decode('utf-8').strip()  # Read and decode data from serial. First element is either 't' or 's'
            data = raw_data.split(',')

            if raw_data[0] != band_id:
                continue

            if len(data) < 10:
                continue

            # See the arduino code to check the indices
            fsr = [float(x) for x in data[1:num_fsrs+1]]
            fsr = [fsr[3], fsr[2], fsr[1], fsr[0], fsr[7], fsr[6], fsr[5], fsr[4]]

            gravity = [float(x) for x in data[9:12]]
            gyro = [float(x) for x in data[12:15]]
            quat = [float(x) for x in data[15:19]]
            acc = [float(x) for x in data[19:22]]
            euler = [float(x) for x in data[22:25]]  # Yaw, Roll, Pitch
            euler = [euler[2], euler[1], euler[0]]  # X [-180, 180], Y [-90, 90], Z [0, 360]

            with data_lock:
                fsr_queue.append(np.array(fsr))
                gravity_queue.append(np.array(gravity))
                gyro_queue.append(np.array(gyro))
                euler_queue.append(np.array(euler))
                acc_queue.append(np.array(acc))
        time.sleep(0.00001)
    ser.close()


def plot_data():
    """
    Function plots the FMG data on the subplot (0, 0), Gyroscope plot on (0, 1)
                    Euler XYZ angles on (1, 0), and Accelerometer plot on (1, 1)
    Function is optimized for fast plotting since all the axes and data handling is done before the thread.
    The thread only updates the lines and texts using set_line() and set_text() functions.
    :return:
    """
    plt.ion()
    plt.grid(True)
    # Following grid is created:
    # FMG    Gyro
    # Euler  Lin. acc.
    fig, ax = plt.subplots(2, 2, figsize=(18, 9))
    fig.canvas.manager.window.setWindowTitle("BioX Band Oscilloscope") # 'TkAgg' was slower. 'QtAgg' is setup at the start of this file for fast update. They provide provision to rename the window.
    plt.tight_layout(pad=2.0)  # Removing some white spaces
    #fig.suptitle('Band Oscilloscope', fontweight='bold', y=0.95)
    fig.canvas.mpl_connect('key_press_event', lambda event: globals().__setitem__('stop_threads', True) if event.key == 'escape' else None)  # Assigning the keyboard interrupt 'Esc' key
    colors = ['blue', 'green', 'red', 'darkorange', 'black', 'darkviolet', 'darkcyan', 'tomato']
    titles = [['FMG', 'Gyro'], ['Euler', 'Linear acc.']]
    ranges = [[[0, 515], [-100, 100]], [[-180, 360],[-30, 30]]]
    imu_labels = ['X', 'Y', 'Z']

    # Initialize lines and text objects
    lines = []
    texts = []
    for row in range(ax.shape[0]):
        for col in range(ax.shape[1]):
            num_elements = num_fsrs if row == 0 and col == 0 else num_fsrs
            lines.append([ax[row, col].plot([], [], color=colors[i], label=f'FSR{i}')[0] for i in range(num_elements)])
            texts.append([ax[row, col].text(0.995, 0.9 - 0.1 * i, '', transform=ax[row, col].transAxes, fontsize=10, verticalalignment='top', horizontalalignment='right', color=colors[i],  multialignment='left') for i in range(num_elements)])

    # Setup up limits and titles
    for i in range(ax.shape[0]):
        for j in range(ax.shape[1]):
            ax[i, j].set_ylim(ranges[i][j][0], ranges[i][j][1])
            ax[i, j].set_xlim(0, data_length + 20)
            ax[i, j].set_title(titles[i][j])
            ax[i, j].grid(True, which='both', linestyle='--', linewidth=0.4)

    # Run the thread
    while not stop_threads:
        # Copy the data with thread lock
        with data_lock:
            fsr_data = list(fsr_queue)
            gravity_data = list(gravity_queue)
            gyro_data = list(gyro_queue)
            euler_data = list(euler_queue)
            acc_data = list(acc_queue)
        fsr_data = np.array(fsr_data)
        gravity_data = np.array(gravity_data)
        gyro_data = np.array(gyro_data)
        euler_data = np.array(euler_data)
        acc_data = np.array(acc_data)

        data = []

        # Plotting
        for row in range(ax.shape[0]):
            for col in range(ax.shape[1]):
                index = row * ax.shape[1] + col

                # Text and data is different for FSR so let's keep it separate
                if row == 0 and col == 0:
                    for fsr in range(num_fsrs):
                        lines[index][fsr].set_data(np.arange(data_length), fsr_data[:, fsr])
                        texts[index][fsr].set_text(f'FSR {fsr + 1} = {float(fsr_data[-1, fsr]):05.1f}')
                    continue

                if row == 0 and col == 1:
                    data = gyro_data
                if row == 1 and col == 0:
                    data = euler_data
                if row == 1 and col == 1:
                    data = acc_data

                for imu in range(num_IMUs):
                    lines[index][imu].set_data(np.arange(data_length), data[:, imu])
                    texts[index][imu].set_text(f'{imu_labels[imu]} = {float(data[-1, imu]):05.1f}')

        plt.pause(0.01)



def start_threads():
    """
    Create and start the serial reading thread
    :return:
    """
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
