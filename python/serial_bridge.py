import serial
import requests
import time
import re
import json
import sys

SERIAL_PORT = '/dev/cu.usbserial-110'
BAUD_RATE = 115200
NODE_RED_URL = 'http://127.0.0.1:1880/sensor_data'

# Regex pattern untuk parsing data ESP32
SENSOR_RE = re.compile(
    r'TILT=([0-9.]+).*?GRAV=([0-9.]+).*?ROT=([0-9.]+).*?PIEZO=([0-9.]+).*?HAZARD=([0-9.]+).*?STATE=([A-Z]+)'
)

def parse_line(line):
    """Parse satu baris data sensor ESP32, return dict atau None."""
    m = SENSOR_RE.search(line)
    if m:
        return {
            "tilt": float(m.group(1)),
            "gravity": float(m.group(2)),
            "rotation": float(m.group(3)),
            "piezo": float(m.group(4)),
            "hazard": float(m.group(5)),
            "state": m.group(6)
        }
    return None

def main():
    print(f"Serial Bridge ESP32 -> Node-RED")
    print(f"Port: {SERIAL_PORT} @ {BAUD_RATE} baud")
    print(f"Target: {NODE_RED_URL}")
    print()

    session = requests.Session()

    while True:
        ser = None
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=3)
            print(f"[OK] Port {SERIAL_PORT} terbuka")
            time.sleep(1)  # beri waktu ESP32 setelah connect

            empty_count = 0

            while True:
                raw = ser.readline()
                if not raw:
                    empty_count += 1
                    if empty_count > 5:  # 5 x 3s timeout = 15 detik tanpa data
                        print("[!] Tidak ada data 15 detik, reconnect...")
                        break
                    continue

                empty_count = 0
                text = raw.decode('utf-8', 'replace').strip()
                if not text:
                    continue

                data = parse_line(text)
                if data:
                    try:
                        session.post(NODE_RED_URL, json=data, timeout=2)
                        sys.stdout.write(f"\r[DATA] Tilt={data['tilt']:.2f} Grav={data['gravity']:.3f} Rot={data['rotation']:.1f} Piezo={data['piezo']:.0f} Hazard={data['hazard']:.0f} {data['state']}   ")
                        sys.stdout.flush()
                    except requests.RequestException as e:
                        print(f"\n[ERR] HTTP: {e}")
                else:
                    # Log non-data lines (boot messages, events, calibration)
                    if 'EVENT' in text or 'CALIBRATION' in text:
                        try:
                            session.post(NODE_RED_URL + '_log', json={"log": text}, timeout=2)
                        except:
                            pass
                        print(f"\n[LOG] {text}")

        except serial.SerialException as e:
            print(f"\n[!] Serial error: {e}")
        except KeyboardInterrupt:
            print("\n[STOP] Dihentikan.")
            break
        except Exception as e:
            print(f"\n[!] Error: {e}")

        if ser and ser.is_open:
            try:
                ser.close()
            except:
                pass

        print("[...] Mencoba ulang dalam 3 detik...")
        time.sleep(3)

if __name__ == "__main__":
    main()
