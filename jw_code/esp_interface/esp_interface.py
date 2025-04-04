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

        Returns: Boolean, Data. A boolean True if the data was successfully read or False if it failed. Data: a list of float values from the ESP32
        """
        try:
            # We recommend moving code outside of the try block for testing.
            if self.serial_port.in_waiting > 0:
                esp32_output = str(self.serial_port.readline()) # The ESP32 output should be a series of values seperated by commas and terminated by "\n", e.g. "1,2,3,4,\n".
                                                           # This termination occurs automatically if you use Serial.println();
                
                esp32_output = esp32_output[2:-3]
                vals = esp32_output.split(",") # Split into a list of strings
                print(vals) # Useful for debugging
                
                for i in range(len(self.data)):
                    self.data[i] = float(vals[i])
                
                print(self.data)
                self.serial_port.reset_input_buffer()
            else:
                time.sleep(0.00101) # Sleep for at least 1 ms to give the loop a chance to rest

            return True, self.data
        except:
            print("Failed to read serial data from ESP32")
            return False, self.data

    def write_serial(self, data_to_write):
        """
        Method for writing to the connected ESP32

        Input: self, data_to_write - a list of floats to be sent to the ESP32

        Returns: Boolean - True if data was written to the ESP32. False if otherwise
        """

        try:
            cmd_string = ""
            for data in data_to_write:
                cmd_string += str(data) + ","

            self.serial_port.write(cmd_string[:-1])
            return True
        except:
            print("Failed to write serial data to the ESP32")
            return False

    def disconnect(self):
        """
        Method for cleanly disconnecting from the ESP32
        """

        self.serial_port.close()
