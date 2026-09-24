"""
ml_engine.py — Landslide EWS Machine Learning Engine
=====================================================
Engine utama yang menjalankan model ML secara berkala.

Mode operasi:
  1. PRODUCTION: Membaca data real-time dari InfluxDB, prediksi, kirim ke Node-RED
  2. TEST:       Membaca dari CSV, prediksi, print hasil ke terminal

FLEKSIBEL: Engine ini memuat model dari folder `models/`.
Saat Anda melatih ulang model dengan data riil baru, cukup replace file .pkl/.keras
di folder models/ lalu restart engine ini.
"""

import os
import sys
import time
import json
import argparse
import joblib
import numpy as np
import pandas as pd
from datetime import datetime

# Cek dependensi opsional
try:
    import requests
    HAS_REQUESTS = True
except ImportError:
    HAS_REQUESTS = False

try:
    from influxdb_client import InfluxDBClient
    HAS_INFLUX = True
except ImportError:
    HAS_INFLUX = False

try:
    from tensorflow.keras.models import load_model
    HAS_TENSORFLOW = True
except ImportError:
    HAS_TENSORFLOW = False

# === KONFIGURASI ===
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.join(SCRIPT_DIR, "models")
FEATURES = ["tilt", "gravity", "piezo"]

# InfluxDB (untuk mode production)
INFLUX_URL = "http://127.0.0.1:8086"
INFLUX_TOKEN = os.environ.get("INFLUX_TOKEN", "_YHgSXt472J04QqXRmpL_KO33YsZx3gk6TenIvlOx3-BXldU2LWtg-OGx0t8zT4rgcs-D_a8WaObOG1u71QWOg==")
INFLUX_ORG = "iot_project"
INFLUX_BUCKET = "landslide_data"

# Node-RED (untuk mode production)
NODE_RED_URL = "http://127.0.0.1:1880/ml_predict"

# Telegram (opsional)
TELEGRAM_BOT_TOKEN = os.environ.get("TELEGRAM_BOT_TOKEN", "")
TELEGRAM_CHAT_ID = os.environ.get("TELEGRAM_CHAT_ID", "")


class LandslideMLEngine:
    """Engine Machine Learning untuk deteksi anomali dan prediksi longsor."""

    def __init__(self):
        self.scaler = None
        self.if_model = None
        self.ae_model = None
        self.ae_threshold = 0.1
        self.last_alert_time = 0
        self.alert_cooldown = 300  # 5 menit antara alert

        self._load_models()

    def _load_models(self):
        """Memuat model dari folder models/."""
        print("[ENGINE] Memuat model ML...")

        # Scaler
        scaler_path = os.path.join(MODELS_DIR, "scaler.pkl")
        if os.path.exists(scaler_path):
            self.scaler = joblib.load(scaler_path)
            print(f"  ✅ Scaler dimuat: {scaler_path}")
        else:
            print(f"  ❌ Scaler tidak ditemukan: {scaler_path}")
            print("     Jalankan train_model.py terlebih dahulu!")
            sys.exit(1)

        # Isolation Forest
        if_path = os.path.join(MODELS_DIR, "isolation_forest.pkl")
        if os.path.exists(if_path):
            self.if_model = joblib.load(if_path)
            print(f"  ✅ Isolation Forest dimuat: {if_path}")
        else:
            print(f"  ❌ Isolation Forest tidak ditemukan!")
            sys.exit(1)

        # Autoencoder (opsional)
        ae_path = os.path.join(MODELS_DIR, "autoencoder.keras")
        if os.path.exists(ae_path) and HAS_TENSORFLOW:
            self.ae_model = load_model(ae_path)
            print(f"  ✅ Autoencoder dimuat: {ae_path}")

            # Load threshold
            threshold_path = os.path.join(MODELS_DIR, "ae_threshold.txt")
            if os.path.exists(threshold_path):
                with open(threshold_path, "r") as f:
                    self.ae_threshold = float(f.read().strip())
                print(f"  ✅ AE Threshold: {self.ae_threshold:.6f}")
        else:
            print(f"  ⚠️  Autoencoder tidak tersedia (opsional)")

    def predict(self, tilt: float, gravity: float, piezo: float) -> dict:
        """
        Prediksi satu data point.
        
        Returns:
            dict dengan keys:
            - risk_percentage (int): Skor risiko 0-100%
            - is_anomaly (bool): Apakah terdeteksi anomali
            - if_score (float): Skor Isolation Forest
            - ae_error (float): Reconstruction error Autoencoder
            - risk_level (str): NORMAL / WATCH / WARNING / DANGER
        """
        X = np.array([[tilt, gravity, piezo]])
        X_scaled = self.scaler.transform(X)

        # --- Isolation Forest ---
        if_prediction = self.if_model.predict(X_scaled)[0]  # 1=normal, -1=anomali
        if_score = self.if_model.decision_function(X_scaled)[0]

        # --- Autoencoder ---
        ae_error = 0.0
        if self.ae_model is not None:
            reconstruction = self.ae_model.predict(X_scaled, verbose=0)
            ae_error = float(np.mean(np.power(X_scaled - reconstruction, 2)))

        # --- Gabungkan Skor ---
        # Normalisasi IF score ke 0-100 (semakin tinggi = semakin berbahaya)
        if_risk = max(0, min(100, int((0.5 - if_score) * 100)))

        # Normalisasi AE error ke 0-100
        ae_risk = 0
        if self.ae_model is not None:
            ae_risk = max(0, min(100, int((ae_error / self.ae_threshold) * 50)))

        # Skor akhir: rata-rata tertimbang (IF lebih berat karena lebih stabil)
        if self.ae_model is not None:
            risk_percentage = int(if_risk * 0.6 + ae_risk * 0.4)
        else:
            risk_percentage = if_risk

        risk_percentage = max(0, min(100, risk_percentage))

        # Tentukan level risiko
        if risk_percentage >= 75:
            risk_level = "DANGER"
        elif risk_percentage >= 50:
            risk_level = "WARNING"
        elif risk_percentage >= 25:
            risk_level = "WATCH"
        else:
            risk_level = "NORMAL"

        return {
            "risk_percentage": risk_percentage,
            "is_anomaly": bool(if_prediction == -1),
            "if_score": round(float(if_score), 4),
            "ae_error": round(ae_error, 6),
            "risk_level": risk_level,
            "tilt": round(tilt, 2),
            "gravity": round(gravity, 4),
            "piezo": round(piezo, 1),
            "timestamp": datetime.now().isoformat(),
        }

    def send_to_nodered(self, result: dict):
        """Kirim hasil prediksi ke Node-RED via HTTP POST."""
        if not HAS_REQUESTS:
            return
        try:
            requests.post(NODE_RED_URL, json=result, timeout=2)
        except Exception as e:
            print(f"  [ERR] Gagal kirim ke Node-RED: {e}")

    def send_telegram_alert(self, result: dict):
        """Kirim notifikasi Telegram jika risk tinggi."""
        if not TELEGRAM_BOT_TOKEN or not TELEGRAM_CHAT_ID or not HAS_REQUESTS:
            return

        # Cooldown: jangan spam notifikasi
        now = time.time()
        if now - self.last_alert_time < self.alert_cooldown:
            return

        if result["risk_percentage"] < 70:
            return

        self.last_alert_time = now

        emoji = "🔴" if result["risk_level"] == "DANGER" else "🟠"
        message = (
            f"{emoji} *PERINGATAN LONGSOR* {emoji}\n"
            f"━━━━━━━━━━━━━━━━━━━━━\n"
            f"⏰ Waktu: {result['timestamp']}\n"
            f"\n"
            f"⚠️ AI Risk Score: *{result['risk_percentage']}%*\n"
            f"📐 Kemiringan: {result['tilt']}°\n"
            f"📳 Getaran: {result['piezo']}\n"
            f"🔬 Gravity Drift: {result['gravity']}g\n"
            f"\n"
            f"Status: *{result['risk_level']}*\n"
            f"━━━━━━━━━━━━━━━━━━━━━"
        )

        try:
            url = f"https://api.telegram.org/bot{TELEGRAM_BOT_TOKEN}/sendMessage"
            requests.post(url, json={
                "chat_id": TELEGRAM_CHAT_ID,
                "text": message,
                "parse_mode": "Markdown"
            }, timeout=5)
            print(f"  [TG] Notifikasi Telegram terkirim!")
        except Exception as e:
            print(f"  [ERR] Gagal kirim Telegram: {e}")

    def run_production(self, interval: int = 10):
        """Mode production: baca dari InfluxDB, prediksi, kirim ke Node-RED."""
        if not HAS_INFLUX:
            print("[ERR] influxdb-client tidak terinstall!")
            sys.exit(1)

        print(f"\n[ENGINE] Mode PRODUCTION — Interval: {interval}s")
        print(f"[ENGINE] InfluxDB: {INFLUX_URL}/{INFLUX_BUCKET}")
        print(f"[ENGINE] Node-RED: {NODE_RED_URL}")

        client = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
        query_api = client.query_api()

        while True:
            try:
                query = f'''
                from(bucket: "{INFLUX_BUCKET}")
                  |> range(start: -1m)
                  |> filter(fn: (r) => r["_measurement"] == "sensor")
                  |> pivot(rowKey:["_time"], columnKey: ["_field"], valueColumn: "_value")
                  |> last()
                '''
                tables = query_api.query(query)

                for table in tables:
                    for record in table.records:
                        tilt = float(record.values.get("tilt", 0))
                        gravity = float(record.values.get("gravity", 0))
                        piezo = float(record.values.get("piezo", 0))

                        result = self.predict(tilt, gravity, piezo)
                        self._print_result(result)
                        self.send_to_nodered(result)
                        self.send_telegram_alert(result)

            except Exception as e:
                print(f"  [ERR] {e}")

            time.sleep(interval)

    def run_test(self, csv_path: str):
        """Mode test: baca dari CSV, prediksi setiap baris, print hasil."""
        print(f"\n[ENGINE] Mode TEST — File: {csv_path}")

        df = pd.read_csv(csv_path)
        results = []
        
        for _, row in df.iterrows():
            result = self.predict(
                tilt=float(row["tilt"]),
                gravity=float(row["gravity"]),
                piezo=float(row["piezo"])
            )

            # Tambahkan ground truth jika ada
            if "label" in row:
                result["true_label"] = int(row["label"])
                result["true_state"] = row.get("state", "?")

            results.append(result)

        results_df = pd.DataFrame(results)

        # Statistik
        print(f"\n{'='*60}")
        print(f"📊 Hasil Prediksi ML Engine")
        print(f"{'='*60}")
        print(f"Total data: {len(results_df):,}")
        print(f"\nDistribusi Risk Level (prediksi ML):")
        for level, count in results_df["risk_level"].value_counts().items():
            print(f"  {level:10s}: {count:6,} ({count/len(results_df)*100:.1f}%)")

        print(f"\nStatistik Risk Score:")
        print(f"  Mean:   {results_df['risk_percentage'].mean():.1f}%")
        print(f"  Median: {results_df['risk_percentage'].median():.1f}%")
        print(f"  Max:    {results_df['risk_percentage'].max():.1f}%")

        # Akurasi (jika ada label)
        if "true_label" in results_df.columns:
            # Mapping: true_label > 0 = anomali
            y_true = (results_df["true_label"] > 0).astype(int)
            y_pred = results_df["is_anomaly"].astype(int)
            accuracy = (y_true == y_pred).mean()
            print(f"\n🎯 Akurasi deteksi anomali: {accuracy*100:.1f}%")

        # Simpan hasil
        output_path = csv_path.replace(".csv", "_predictions.csv")
        results_df.to_csv(output_path, index=False)
        print(f"\n[SAVE] Hasil disimpan: {output_path}")

    def _print_result(self, result: dict):
        """Print hasil prediksi dengan format rapi."""
        emoji_map = {
            "NORMAL": "🟢",
            "WATCH": "🟡",
            "WARNING": "🟠",
            "DANGER": "🔴",
        }
        emoji = emoji_map.get(result["risk_level"], "⚪")

        sys.stdout.write(
            f"\r{emoji} Risk: {result['risk_percentage']:3}% | "
            f"Level: {result['risk_level']:8s} | "
            f"Tilt: {result['tilt']:6.2f}° | "
            f"Piezo: {result['piezo']:5.1f} | "
            f"IF: {result['if_score']:+.4f} | "
            f"AE: {result['ae_error']:.6f}   "
        )
        sys.stdout.flush()

        # Newline untuk anomali (agar terlihat jelas)
        if result["is_anomaly"]:
            print(f"\n  ⚠️  ANOMALI TERDETEKSI!")


def main():
    parser = argparse.ArgumentParser(description="Landslide EWS ML Engine")
    parser.add_argument("--mode", choices=["production", "test"], default="test",
                        help="Mode operasi: production (InfluxDB) atau test (CSV)")
    parser.add_argument("--data", default="data/dataset_simulated.csv",
                        help="Path CSV untuk mode test")
    parser.add_argument("--interval", type=int, default=10,
                        help="Interval prediksi dalam detik (mode production)")
    args = parser.parse_args()

    engine = LandslideMLEngine()

    if args.mode == "production":
        engine.run_production(interval=args.interval)
    else:
        data_path = os.path.join(SCRIPT_DIR, args.data)
        engine.run_test(data_path)


if __name__ == "__main__":
    main()
