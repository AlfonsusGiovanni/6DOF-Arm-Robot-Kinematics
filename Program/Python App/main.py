import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import numpy as np
import struct

# Data header class
class Data_Header:
    HEADER1 = 0xFF
    HEADER2 = 0xAA

# Data command class
class Data_Command:
    NO_CMD = 0x00
    CALC_FK = 0x01
    FK_RESULT = 0x02
    CALC_IK = 0x03
    IK_RESULT = 0x04
    CONNECT_DEVICES = 0x05

# Apps class 
class KinematicsApp:
    def __init__(self, root):
        self.root = root
        self.root.title("6-DOF Kinematics Simulator")
        self.root.resizable(False, False)

        self.ser = None

        self.serial_run = False
        self.serial_connected = False
        self.com_port = 'COM13'  # Default Com Port
        self.baud_rate = 9600   # Default Baud Rate

        self.fk_entry_input = [0] * 6
        self.ik_entry_input = [0] * 6

        self.joint_angle = [0.0] * 6
        self.tool_pos = [0.0] * 6

        self.show_window()

    # Show serial text file on text box
    def append_text(self, text):
        self.txt_box.config(state='normal')
        self.txt_box.insert('end', text + '\n')
        self.txt_box.see('end')  # Auto-scroll to bottom
        self.txt_box.config(state='disabled')

    # Get com port
    def get_com_port(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    # Refresh com port
    def refresh_port(self):
        ports = self.get_com_port()
        self.com_combo['values'] = ports

    # Clear serial monitor
    def clear_monitor(self):
        self.txt_box.config(state='normal')
        self.txt_box.delete("1.0", tk.END)
        self.txt_box.config(state='disabled')

    # Start serial communication
    def connect_serial(self):        
        try:
            self.com_port = self.com_combo.get()
            self.baud_rate = self.baud_combo.get()
            #print(self.com_port, self.baud_rate)
            
            if not self.serial_connected:
                print("Connecting")
                self.ser = serial.Serial(port=self.com_port, baudrate=self.baud_rate, timeout=0.1)
                self.serial_connected = True

            data_to_send = [Data_Header.HEADER1, Data_Header.HEADER2, Data_Command.CONNECT_DEVICES]
            self.ser.write(data_to_send)

        except Exception as e:
            self.append_text(f"[ERROR] {e}")

    # Read serial data
    def read_serial(self):
        if self.serial_run and self.ser and self.ser.in_waiting:
            try:
                data = self.ser.read_all()
                if len(data) > 0:
                    print(data)
                    if data[0] == Data_Header.HEADER1 and data[1] == Data_Header.HEADER2:
                        if data[2] == Data_Command.CONNECT_DEVICES:
                            data_char = [0] * 20
                            for i in range(0, 20):
                                data_char[i] = data[i+3]

                            arr = np.array(data_char, dtype=np.uint8)
                            data_string = ''.join(arr.view('S1').astype(str))
                            print(data_string)
                            self.append_text(data_string)

                        elif data[2] == Data_Command.FK_RESULT:
                            tool_array = [0] * 6    # Toolframe pos byte array
                            tool_data = [0] * 6     # Merged toolframe byte array

                            for i in range(0, 6):
                                self.tool_pos[i] = 0.0

                                tool_array[i] = bytearray([data[3+i*4], data[4+i*4], data[5+i*4], data[6+i*4]])
                                tool_data[i] = bytes(tool_array[i])
                                self.tool_pos[i] = struct.unpack('f', tool_data[i])[0]

                                rounded = round(self.tool_pos[i], 3)

                                self.pos_out_txt[i].config(state='normal')
                                self.pos_out_txt[i].delete("1.0", tk.END)
                                self.pos_out_txt[i].insert('end', f"{rounded}" + '\n')
                                self.pos_out_txt[i].config(state='disabled')

                        elif data[2] == Data_Command.IK_RESULT:
                            angle_array = [0] * 6   # Toolframe pos byte array
                            angle_data = [0] * 6    # Merged toolframe byte array

                            for i in range(0, 6):
                                self.joint_angle[i] = 0.0

                                angle_array[i] = bytearray([data[3+i*4], data[4+i*4], data[5+i*4], data[6+i*4]])
                                angle_data[i] = bytes(angle_array[i])
                                self.joint_angle[i] = struct.unpack('f', angle_data[i])[0]

                                rounded = round(self.joint_angle[i], 3)

                                self.angle_out_txt[i].config(state='normal')
                                self.angle_out_txt[i].delete("1.0", tk.END)
                                self.angle_out_txt[i].insert('end', f"{rounded}" + '\n')
                                self.angle_out_txt[i].config(state='disabled')
        
            except Exception as e:
                self.append_text(f"[ERROR] {e}")
                
        if self.serial_run:
            self.serial_tab.after(100, self.read_serial)

    # Send forward kinematics data
    def send_fk_data(self):
        float_entry = [0.0] * 6

        for i in range(0, 6):
            float_entry[i] = float(self.fk_entry_input[i].get())

        angle_bytes_array = struct.pack('<6f', *float_entry)

        data_to_send = bytearray([Data_Header.HEADER1, Data_Header.HEADER2, Data_Command.CALC_FK]) + angle_bytes_array
        self.ser.write(data_to_send)

    # Send inverse kinematics data
    def send_ik_data(self):
        float_entry = [0.0] * 6

        for i in range(0, 6):
            float_entry[i] = float(self.ik_entry_input[i].get())

        pos_bytes_array = struct.pack('<6f', *float_entry)

        data_to_send = bytearray([Data_Header.HEADER1, Data_Header.HEADER2, Data_Command.CALC_IK]) + pos_bytes_array
        self.ser.write(data_to_send)

    # Serial com config window
    def com_window(self):
        baud_option = ["9600", "115200"]

        # Style config
        style = ttk.Style()
        style.configure("Custom.TButton", font=("Arial", 10))

        # Port selection
        ttk.Label(self.serial_tab, text="Select Port", font=('Arial', 10)).grid(row=0, column=0, sticky="w")
        self.com_combo = ttk.Combobox(self.serial_tab, values=self.com_port)
        self.com_combo.grid(row=0, column=1, sticky="w")

        # Baud rate selection
        ttk.Label(self.serial_tab, text="Baud Rate", font=('Arial', 10)).grid(row=1, column=0, sticky="w")
        self.baud_combo = ttk.Combobox(self.serial_tab, values=baud_option)
        self.baud_combo.grid(row=1, column=1, sticky="w")

        # Button 
        btn_frame = ttk.Frame(self.serial_tab)
        btn_frame.grid(row=2, column=0, columnspan=3, sticky="w")
        ttk.Button(btn_frame, text="Refresh", style="Custom.TButton", command=self.refresh_port, width=10).grid(row=0, column=0, pady=5, sticky="w")
        ttk.Button(btn_frame, text="Connect", style="Custom.TButton", command=self.connect_serial, width=10).grid(row=0, column=1, pady=5, sticky="w")
        ttk.Button(btn_frame, text="Clear", state="Custom.Tbutton", command=self.clear_monitor, width=12).grid(row=0, column=2, pady=5, sticky="w")

        # Port status
        status_frame = ttk.Frame(self.serial_tab)
        status_frame.grid(row=3, column=0, columnspan=4, sticky="w")
        ttk.Label(status_frame, text="Serial Monitor", font=('Arial', 10)).grid(row=0, column=0, pady=5, sticky="w")
        self.txt_box = tk.Text(status_frame, height=15, width=50, state='disabled')
        self.txt_box.grid(row=1, column=0)

    # Forward kinematic tab window
    def fk_window(self):
        # Joint angle input
        for i in range(0, 6):
            ttk.Label(self.fk_tab, text="Joint " + f"{i+1}", font=('Arial', 10), padding=5).grid(row=i, column=0, sticky="w")
            self.fk_entry_input[i] = ttk.Entry(self.fk_tab, width=10)
            self.fk_entry_input[i].grid(row=i, column=1, sticky="w", padx=5)

        # Tool position output
        axis = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        self.pos_out_txt = [0] * 6
    
        for i in range(0, 6):
            ttk.Label(self.fk_tab, text=axis[i], font=('Arial', 10)).grid(row=i, column=2, sticky="w", padx=5)
            self.pos_out_txt[i] = tk.Text(self.fk_tab, height=1, width=8, state='disabled')
            self.pos_out_txt[i].grid(row=i, column=3, sticky="w")

        # Update fk value button
        btn_frame = ttk.Frame(self.fk_tab)
        btn_frame.grid(row=6, column=0, columnspan=2, sticky="w")
        ttk.Button(btn_frame, text="Update", state="Custom.Tbutton", command=self.send_fk_data).grid(row=6, column=0, pady=10, sticky="w")

    # Inverse kinematic tab window
    def ik_window(self):
        # Tool position input
        axis = ["X", "Y", "Z", "Rx", "Ry", "Rz"]
        self.angle_out_txt = [0] * 6 
        for i in range(0, 6):
            ttk.Label(self.ik_tab, text=axis[i], font=('Arial', 10), padding=5).grid(row=i, column=0, sticky="w")
            self.ik_entry_input[i] = ttk.Entry(self.ik_tab, width=10)
            self.ik_entry_input[i].grid(row=i, column=1, sticky="w")

        # Joint angle output
        for i in range(0, 6):
            ttk.Label(self.ik_tab, text="Joint " + f"{i+1}", font=('Arial', 10)).grid(row=i, column=2, sticky="w", padx=5)
            self.angle_out_txt[i] = tk.Text(self.ik_tab, height=1, width=8, state='disabled')
            self.angle_out_txt[i].grid(row=i, column=3, sticky="w", padx=5)

        # Update fk value button
        btn_frame = ttk.Frame(self.ik_tab)
        btn_frame.grid(row=6, column=0, columnspan=2, sticky="w")
        ttk.Button(btn_frame, text="Update", state="Custom.Tbutton", command=self.send_ik_data).grid(row=6, column=0, pady=10, sticky="w")

    # Main window
    def show_window(self):
        # Set window size
        self.root.geometry("400x300")

        # Tab 
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True)

        # Serial com setting tab
        self.serial_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.serial_tab, text=" Com Setting ")
        self.com_window()

        # Forward kinematics tab
        self.fk_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.fk_tab, text=" Forward ")
        self.fk_window()

        # Inverse kinematics tab
        self.ik_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.ik_tab, text=" Inverse ")
        self.ik_window()

        # Start receive data thread
        if not self.serial_run:
            self.serial_run = True
            receive_data_thread = threading.Thread(target=self.read_serial)
            receive_data_thread.start()

if __name__ == "__main__":
    root = tk.Tk()
    app = KinematicsApp(root)
    root.mainloop()
