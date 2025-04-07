# esp_interface.py
import serial
import time

class ESPInterface:
    def __init__(self, port):
        """
        Initialize ESP interface. given a serial port
        """

        self._state = {}
        self.port_name = port
        self.baud_rate = 115200
        self.read_timeout = 1
        self.write_timeout = 1

        self.serial_port = serial.Serial(port=self.port_name, baudrate=self.baud_rate, timeout=self.read_timeout, write_timeout=self.write_timeout)
        self.data = []

    #Example code
    def get(self, key):
        """
        Generic getter for internal state.
        """
        return self._state.get(key, None)
    
    def set(self, key, value):
        """
        Generic setter for internal state.
        """
        self._state[key] = value

    def aux_function(self, *args, **kwargs):
        """
        Auxiliary function placeholder.
        """
        print("ESPInterface auxiliary function called.")

    #getter methods
    def get_port(self):
        """
        Return the port being used for the ESP32 interface
        """
        return self.port_name
    
    def get_baud(self):
        """
        Return the current baud rate of the ESP32 interface
        """
        return self.baud_rate
    
    #setter method

    #aux methods
    def read_serial(self):
        """
        Method for reading the connected ESP32

        Input: self

        Returns: Boolean, Data. A boolean True if new data was successfully read, or False otherwise.
             Data: a list of float values from the ESP32 or the last read values.
        """
        try:
            if self.serial_port.in_waiting > 0:
                esp32_output = self.serial_port.readline().decode('utf-8').strip()
                vals = esp32_output.split(",")  # Split into a list of strings
                print(vals)  # Debugging output

                self.data = [float(val) for val in vals]
                return True, self.data
            else:
                # No new data, but return previous data
                return True, self.data
        except Exception as e:
            print(f"Failed to read serial data from ESP32: {e}")
            return False, self.data


    def write_serial(self, pickup_state, thrower_state):
        """
        Method for writing pickup and thrower states to the connected ESP32.

        Input:
            pickup_state - int (0 or 1)
            thrower_state - int (0 or 1)

        Returns:
            Boolean - True if data was written to the ESP32, False otherwise
        """

        try:
            # Make sure they are integers
            cmd_string = f"{int(pickup_state)},{int(thrower_state)}\n"
            self.serial_port.write(cmd_string.encode('utf-8'))
            print(f"Writing to ESP32: {cmd_string.strip()}") 
            return True
        except Exception as e:
            print(f"Failed to write serial data to the ESP32: {e}")
            return False

    def disconnect(self):
        """
        Method for cleanly disconnecting from the ESP32
        """

        self.serial_port.close()
