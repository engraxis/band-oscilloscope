import serial
from tkinter import *
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# To view the available serial ports, open terminal
# ls /dev/tty.*

# --------- SETUP PARAMETERS -------------- #
# Arduino serial port for trigger
trigger_serial_port = '/dev/tty.usbmodem101'
ser_trigger = serial.Serial(trigger_serial_port, 9600)

# ESP32 serial port for receiving data from bands
esp_serial_port = '/dev/tty.usbserial-0001'
ser_esp = serial.Serial(esp_serial_port, 230400)


def start_record():
    if act_1.get() == "":
        return
    ser_trigger.write("Pulse\n".encode())
    start_button.config(state="disabled")
    stop_button.config(state="normal")
    global recording
    global name
    global start_time
    name = act_1.get() + ".txt"
    recording = True
    start_time = datetime.now()
    update()


def stop_record():
    global data
    global name
    global recording
    global end_time
    recording = False

    ser_trigger.write("Pulse\n".encode())

    start_button.config(state="normal")
    stop_button.config(state="disabled")
    act_1.delete(0, END)

    end_time = datetime.now()

    with open(name, "w+") as file:
        # write sample rate
        time_taken = len(data) / (int((end_time - start_time).total_seconds()))
        file.write("Sample Rate = " + str(time_taken) + "\n")

        # write header
        file.write("FSR1,FSR2,FSR3,FSR4,FSR5,FSR6,FSR7,FSR8,GravX,GravY,GravZ,GyroX,GyroY,GyroZ,QuatW,QuatX,QuatY,QuatZ,LiAccX,LiAccY,LiAccZ,GyroCal,AccelCal,MagCal\n")

        # write data
        for row in data:
            file.write(row)

    data = []
    name = ""


def update():
    # get current line (can fail so try a couple of times)
    global data
    time_taken = str(datetime.now() - start_time)
    serial_data = ser_esp.readline().decode()
    fsr_only = serial_data.split(',')[1:9]
    alert = any(float(fsr) > 500 for fsr in fsr_only)
    fsr_only = [f'{float(fsr):05.1f}' for fsr in fsr_only]
    fsr_only = " ".join(fsr_only)

    try:
        row = time_taken + "," + serial_data
    except:
        row = time_taken + serial_data

    if recording:
        data.append(row)

    if serial_data.startswith('t'):
        thigh_box.delete("1.0", END)
        if alert:
            thigh_box.insert("3.0", "Thigh FSR:  " + fsr_only, "colored_text")
        else:
            thigh_box.insert("3.0", "Thigh FSR:  " + fsr_only)

    if serial_data.startswith('s'):
        shank_box.delete("1.0", END)
        if alert:
            shank_box.insert("3.0", "Thigh FSR:  " + fsr_only, "colored_text")
        else:
            shank_box.insert("3.0", "Thigh FSR:  " + fsr_only)


    output_box.delete("1.0", END)  # remove everything in the box
    output_box.insert("3.0", row + "\n")

    global name
    if name != "":
        output_box.insert("1.0", "Writing to " + name + "\n")
    else:
        output_box.insert("1.0", "Please input file name and click record\n")

    window.after(1, update)  # run itself again after 1 ms


def on_closing():
    try:
        ser_trigger.close()
        ser_esp.close()
    except:
        pass
    window.destroy()


# Initial variables
start_time = datetime.now()
end_time = datetime.now()
recording = False
name = ""
data = []

# Main window
window = Tk()
window.title('Serial Interface')
window.geometry("1000x150+10+20")  # WIDTHxHEIGHT+XPOS+YPOS

# Activity 1
act_1 = Entry(window)
act_1.grid(row=0, column=0, columnspan=2)
start_button = Button(text="Start Recording", command=start_record)
start_button.grid(row=1, column=0, columnspan=2)

# Stop Button
stop_button = Button(text="Stop Recording", command=stop_record)
stop_button.grid(row=2, column=0, columnspan=2)
stop_button.config(state="disabled")

# Output
output_box = Text(window, width=110, height=3)
output_box.grid(row=0, column=2, columnspan=4, rowspan=1)

thigh_box = Text(window, width=110, height=3)
thigh_box.grid(row=1, column=2, columnspan=4, rowspan=1)
thigh_box.tag_configure("colored_text", foreground="blue")

shank_box = Text(window, width=110, height=3)
shank_box.grid(row=2, column=2, columnspan=4, rowspan=1)
shank_box.tag_configure("colored_text", foreground="blue")

window.mainloop()


