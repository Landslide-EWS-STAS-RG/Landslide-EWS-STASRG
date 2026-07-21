#!/bin/bash
echo "=========================================="
echo "    MEMULAI SISTEM IOT TANAH LONGSOR    "
echo "=========================================="
echo ""

# 1. Pastikan InfluxDB berjalan
echo "[1/3] Mengecek Database InfluxDB..."
brew services start influxdb@2 >/dev/null 2>&1

# 2. Menjalankan Node-RED (jika belum jalan)
echo "[2/3] Menjalankan Server Node-RED..."
if pgrep -x "node-red" > /dev/null
then
    echo "      Node-RED sudah berjalan."
else
    # Jalankan Node-RED di background
    nohup node-red > /tmp/node-red.log 2>&1 &
    sleep 3
    echo "      Node-RED berhasil dihidupkan."
fi

# 3. Menjalankan Serial Bridge Python
echo "[3/3] Menjalankan Serial Bridge (Koneksi ESP32)..."
pkill -f serial_bridge.py
nohup /Users/tonihandoko/.gemini/antigravity/scratch/venv/bin/python3 -u /Users/tonihandoko/.gemini/antigravity/scratch/serial_bridge.py > /tmp/serial_bridge.log 2>&1 &

echo ""
echo "=========================================="
echo "          SISTEM BERHASIL AKTIF!          "
echo "=========================================="
echo "Silakan buka tautan berikut di browser Anda:"
echo "1. Dashboard Node-RED : http://localhost:1880/ui"
echo "2. Database InfluxDB  : http://localhost:8086"
echo ""
echo "Jendela ini boleh dibiarkan terbuka atau diminimize."
echo "Untuk menghentikan semua sistem, cukup tutup terminal (atau jalankan pkill node-red)."
