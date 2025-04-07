# test_esp.py

import time
from esp_interface.esp_interface import ESPInterface

def test_esp():
    counter = 0
    
    #Test connect
    try:
        esp = ESPInterface('/dev/ttyACM0')
        data = []
    except Exception as e:
        print("Cannot connect to ESP32:" + e)
        exit()

    while (True):
        got_data, data = esp.read_serial()
        print(got_data)
        print(data)

        if (counter % 3):
            esp.write_serial(1,0)
        elif (counter % 2):
            esp.write_serial(0,1)
        else:
            esp.write_serial(0,0)
            
        time.sleep(1)

        counter += 1
        
    

if __name__ == "__main__":
    test_esp()
