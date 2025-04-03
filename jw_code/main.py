# main.py

from control.system_controller import SystemController
import time

def main():
    print("[MAIN] Starting system...")
    system = SystemController()

    try:
        while system.running:
            result = system.step()
            print(f"[MAIN] {result}")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("[MAIN] Interrupted by user.")

    finally:
        system.shutdown()
        print("[MAIN] System shutdown complete.")

if __name__ == "__main__":
    main()