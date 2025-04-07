# test_esp.py

from esp_interface.esp_interface import ESPInterface

def test_esp():
    
    #Test connect
    try:
        esp = ESPInterface('/dev/ttyACM0')
        data = []
    except Exception as e:
        print("Cannot connect to ESP32:" + e)
        exit

    while (True):
        got_data, data = esp.read_serial()
        print(got_data)
        print(data)
    

if __name__ == "__main__":
    test_esp()
