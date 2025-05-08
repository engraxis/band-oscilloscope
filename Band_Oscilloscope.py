import serial
import threading
import matplotlib
import numpy as np
import time
from collections import deque
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

matplotlib.use('QtAgg')  # This backend will allow to have the title of the figure.


class SerialReader:
    """
    Class reads the data from the serial port and extracts respective components of data.
    Runs the thread to continuously read the serial port.
    Data is added in the queue with the data_lock.
    """
    def __init__(self, port, band_id, num_fsrs, num_imus, data_length):
        self.port = port
        self.band_id = band_id
        self.num_fsrs = num_fsrs
        self.num_imus = num_imus
        self.data_length = data_length

        # Data containers for the respective data type
        self.fsr_queue = deque([np.zeros(self.num_fsrs) for _ in range(self.data_length)], maxlen=self.data_length)
        self.gravity_queue = deque([np.zeros(self.num_imus) for _ in range(self.data_length)], maxlen=self.data_length)
        self.gyro_queue = deque([np.zeros(self.num_imus) for _ in range(self.data_length)], maxlen=self.data_length)
        self.euler_queue = deque([np.zeros(self.num_imus) for _ in range(self.data_length)], maxlen=self.data_length)
        self.acc_queue = deque([np.zeros(self.num_imus) for _ in range(self.data_length)], maxlen=self.data_length)

        self.stop_threads = False
        self.data_lock = threading.Lock()

    def start_reading(self):
        try:
            self.ser = serial.Serial(self.port, 230400)
        except Exception as e:
            print(f"Error connecting with serial port. {e}\nExiting.")
            exit(0)

        while not self.stop_threads:
            if self.ser.in_waiting > 0:
                raw_data = self.ser.readline().decode('utf-8').strip()  # Read and decode data from serial.
                data = raw_data.split(',')

                if raw_data[0] != self.band_id:
                    continue

                if len(data) < 10:
                    continue

                fsr = [float(x) for x in data[1:self.num_fsrs+1]]
                fsr = [fsr[3], fsr[2], fsr[1], fsr[0], fsr[7], fsr[6], fsr[5], fsr[4]]

                gravity = [float(x) for x in data[9:12]]
                gyro = [float(x) for x in data[12:15]]
                quat = [float(x) for x in data[15:19]]
                acc = [float(x) for x in data[19:22]]
                euler = [float(x) for x in data[22:25]]  # Yaw, Roll, Pitch
                euler = [euler[2], euler[1], euler[0]]  # X [-180, 180], Y [-90, 90], Z [0, 360]

                with self.data_lock:
                    self.fsr_queue.append(np.array(fsr))
                    self.gravity_queue.append(np.array(gravity))
                    self.gyro_queue.append(np.array(gyro))
                    self.euler_queue.append(np.array(euler))
                    self.acc_queue.append(np.array(acc))
            time.sleep(0.00001)
        self.ser.close()


class DataPlotter:
    """
    This class plots the FMG data on the subplot (0, 0), Gyroscope plot on (0, 1)
                    Euler XYZ angles on (1, 0), and Accelerometer plot on (1, 1)
    lot_data() is optimized for fast plotting since all the axes and data handling is done before the thread.
    The thread only updates the lines and texts using set_line() and set_text() functions.
    """
    def __init__(self, num_fsrs, num_imus, data_length, fsr_queue, gravity_queue, gyro_queue, euler_queue, acc_queue, data_lock):
        self.num_fsrs = num_fsrs
        self.num_imus = num_imus
        self.data_length = data_length
        self.fsr_queue = fsr_queue
        self.gravity_queue = gravity_queue
        self.gyro_queue = gyro_queue
        self.euler_queue = euler_queue
        self.acc_queue = acc_queue
        self.data_lock = data_lock  # Store the data_lock here
        self.stop_threads = False

    def on_key_press(self, event):
        """Handle key press events."""
        if event.key == 'escape':  # Check if the pressed key is 'Escape'
            self.stop_threads = True
            print("Stopping threads...")

    def plot_data(self):
        plt.ion()
        plt.grid(True)
        fig, ax = plt.subplots(2, 2, figsize=(18, 9))
        fig.canvas.manager.window.setWindowTitle("BioX Band Oscilloscope")
        plt.tight_layout(pad=2.0)

        fig.canvas.mpl_connect('key_press_event', self.on_key_press)  # Connect the key press event handler
        colors = ['blue', 'green', 'red', 'darkorange', 'black', 'darkviolet', 'darkcyan', 'tomato']
        titles = [['FMG', 'Gyro'], ['Euler', 'Linear acc.']]
        ranges = [[[0, 515], [-100, 100]], [[-180, 360],[-30, 30]]]
        imu_labels = ['X', 'Y', 'Z']

        lines = []
        texts = []
        for row in range(ax.shape[0]):
            for col in range(ax.shape[1]):
                num_elements = self.num_fsrs if row == 0 and col == 0 else self.num_fsrs
                lines.append([ax[row, col].plot([], [], color=colors[i], label=f'FSR{i}')[0] for i in range(num_elements)])
                texts.append([ax[row, col].text(0.995, 0.9 - 0.1 * i, '', transform=ax[row, col].transAxes, fontsize=10, verticalalignment='top', horizontalalignment='right', color=colors[i], multialignment='left') for i in range(num_elements)])

        for i in range(ax.shape[0]):
            for j in range(ax.shape[1]):
                ax[i, j].set_ylim(ranges[i][j][0], ranges[i][j][1])
                ax[i, j].set_xlim(0, self.data_length + 20)
                ax[i, j].set_title(titles[i][j])
                ax[i, j].grid(True, which='both', linestyle='--', linewidth=0.4)

        while not self.stop_threads:
            with self.data_lock:
                fsr_data = list(self.fsr_queue)
                gravity_data = list(self.gravity_queue)
                gyro_data = list(self.gyro_queue)
                euler_data = list(self.euler_queue)
                acc_data = list(self.acc_queue)

            fsr_data = np.array(fsr_data)
            gravity_data = np.array(gravity_data)
            gyro_data = np.array(gyro_data)
            euler_data = np.array(euler_data)
            acc_data = np.array(acc_data)

            data = []

            for row in range(ax.shape[0]):
                for col in range(ax.shape[1]):
                    index = row * ax.shape[1] + col

                    if row == 0 and col == 0:
                        for fsr in range(self.num_fsrs):
                            lines[index][fsr].set_data(np.arange(self.data_length), fsr_data[:, fsr])
                            texts[index][fsr].set_text(f'FSR {fsr + 1} = {float(fsr_data[-1, fsr]):05.1f}')
                        continue

                    if row == 0 and col == 1:
                        data = gyro_data
                    if row == 1 and col == 0:
                        data = euler_data
                    if row == 1 and col == 1:
                        data = acc_data

                    for imu in range(self.num_imus):
                        lines[index][imu].set_data(np.arange(self.data_length), data[:, imu])
                        texts[index][imu].set_text(f'{imu_labels[imu]} = {float(data[-1, imu]):05.1f}')

            plt.pause(0.01)


class BioXBandApp:
    def __init__(self, esp_port, band_id='z', num_fsrs=8, num_imus=3, data_length=140):
        self.esp_port = esp_port
        self.band_id = band_id
        self.num_fsrs = num_fsrs
        self.num_imus = num_imus
        self.data_length = data_length

        # Instantiate the SerialReader and DataPlotter classes
        self.serial_reader = SerialReader(self.esp_port, self.band_id, self.num_fsrs, self.num_imus, self.data_length)
        self.data_plotter = DataPlotter(self.num_fsrs, self.num_imus, self.data_length,
                                        self.serial_reader.fsr_queue, self.serial_reader.gravity_queue,
                                        self.serial_reader.gyro_queue, self.serial_reader.euler_queue,
                                        self.serial_reader.acc_queue, self.serial_reader.data_lock)

    def start_threads(self):
        serial_thread = threading.Thread(target=self.serial_reader.start_reading)
        serial_thread.daemon = True
        serial_thread.start()

        try:
            self.data_plotter.plot_data()
        except KeyboardInterrupt:
            self.serial_reader.stop_threads = True
            print("Stopping threads...")


if __name__ == "__main__":
    band_id = 'z'
    while band_id != 's' and band_id != 't':
        band_id = input('>> Which band\'s data you want to view, s: shank, t: thigh? [s/t] ')
        band_id = band_id.lower()

    # Setup all the parameters for each thread
    app = BioXBandApp('/dev/tty.usbserial-0001', band_id=band_id)

    # Start the threads.
    app.start_threads()
