# test_cv.py

from cv_interface.cv_interface import CVInterface

def test_cv():
    cv = CVInterface()

    # Test setting and getting
    cv.set("test_key", "test_value")
    assert cv.get("test_key") == "test_value", "Getter/Setter failed for CVInterface"

    # Test aux function
    cv.aux_function()

    print("CVInterface tests passed.")

if __name__ == "__main__":
    test_cv()
