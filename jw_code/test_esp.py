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

    last_send_time = time.time()
    last_print_time = time.time()
    send_interval = 0.5   # seconds between sending data
    print_interval = 0.5  # seconds between prints

    state = 0  # Tracks which state we're sending

    while True:
        got_data, data = esp.read_serial()

        current_time = time.time()

        # Print data at intervals
        if current_time - last_print_time >= print_interval:
            print(data)
            last_print_time = current_time

        # Send commands at intervals
        if current_time - last_send_time >= send_interval:
            if state == 0:
                esp.write_serial(1,0)
            elif state == 1:
                esp.write_serial(0,1)
            else:
                esp.write_serial(0,0)

            state = (state + 1) % 3  # Cycle through 0, 1, 2
            last_send_time = current_time

        time.sleep(0.01)  # Small sleep to prevent 100% CPU usage
        
    

if __name__ == "__main__":
    test_esp()
