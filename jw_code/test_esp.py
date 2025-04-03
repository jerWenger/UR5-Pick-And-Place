# test_esp.py

from esp_interface.esp_interface import ESPInterface

def test_esp():
    esp = ESPInterface()

    # Test setting and getting
    esp.set("status", "ready")
    assert esp.get("status") == "ready", "Getter/Setter failed for ESPInterface"

    # Test aux function
    esp.aux_function()

    print("ESPInterface tests passed.")

if __name__ == "__main__":
    test_esp()
