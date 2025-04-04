# test_esp.py

from esp_interface.esp_interface import ESPInterface

def test_esp():
    
    #Test connect
    try:
        esp = ESPInterface('/dev/ttyACM0')
    except Exception as e:
        print("Cannot connect to ESP32:" + e)
        exit

    # Test getter method
    assert esp.get_baud() == 115200
    
    #Test read
    try:
        got_data, data = esp.read_serial()
        print("ESP Data: " + data)
    except Exception as e:
        print("Could not read esp data: " + e)

    #Test write
    try:
        esp.write_serial([2.1,34.56,100.0234])
    except Exception as e:
        print("Could not write data to esp: " + e)

    #Test close
    try:
        esp.close()
    except Exception as e:
        print("Could not close esp connection: " + e)

    print("ESP interface test complete.")

if __name__ == "__main__":
    test_esp()
