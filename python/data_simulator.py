"""
data_simulator.py — Landslide EWS Data Simulator
=================================================
Menghasilkan data sintetis sensor (tilt, gravity, piezo) untuk melatih model ML.
Data ini mensimulasikan 4 skenario: Normal, Watch, Warning, Danger/Critical.

FLEKSIBEL: File ini hanya dibutuhkan saat belum ada data riil.
Setelah alat di-deploy dan data riil terkumpul di InfluxDB,
cukup jalankan `data_collector.py` untuk mengekspor data riil ke CSV,
lalu latih ulang model menggunakan `train_model.py`.
"""

import numpy as np
import pandas as pd
import os
import argparse
from datetime import datetime, timedelta

# === KONFIGURASI SENSOR (Sesuaikan dengan alat Anda) ===
# Nilai-nilai ini berdasarkan threshold di kode Arduino Anda.
# Saat data riil sudah ada, angka-angka ini bisa di-update.

SENSOR_CONFIG = {
    "tilt": {
        "unit": "derajat",
        "normal": {"mean": 2.0, "std": 0.5},      # Tanah stabil: ~2° ± 0.5°
        "watch": {"mean": 6.0, "std": 1.5},         # Mulai bergeser
        "warning": {"mean": 14.0, "std": 3.0},      # Berbahaya
        "danger": {"mean": 25.0, "std": 5.0},       # Longsor aktif
    },
    "gravity": {
        "unit": "g (deviasi)",
        "normal": {"mean": 0.01, "std": 0.005},
        "watch": {"mean": 0.03, "std": 0.01},
        "warning": {"mean": 0.08, "std": 0.03},
        "danger": {"mean": 0.25, "std": 0.08},
    },
    "piezo": {
        "unit": "vibration index (0-100)",
        "normal": {"mean": 5, "std": 3},
        "watch": {"mean": 25, "std": 10},
        "warning": {"mean": 55, "std": 15},
        "danger": {"mean": 85, "std": 10},
    },
}

# Label mapping
LABELS = {
    "normal": 0,
    "watch": 1,
    "warning": 2,
    "danger": 3,
}


def generate_segment(state: str, duration_seconds: int, interval_seconds: int = 10,
                     start_time: datetime = None, transition_from: str = None) -> pd.DataFrame:
    """
    Generate satu segmen data sensor untuk state tertentu.
    
    Args:
        state: 'normal', 'watch', 'warning', 'danger'
        duration_seconds: Durasi segmen dalam detik
        interval_seconds: Interval antar sample (default 10 detik)
        start_time: Waktu mulai segmen
        transition_from: Jika diisi, data akan bertransisi gradual dari state ini ke target state
    """
    n_samples = duration_seconds // interval_seconds
    if start_time is None:
        start_time = datetime.now()

    timestamps = [start_time + timedelta(seconds=i * interval_seconds) for i in range(n_samples)]

    data = {"timestamp": timestamps}

    for sensor_name, config in SENSOR_CONFIG.items():
        target = config[state]

        if transition_from and transition_from in config:
            # Transisi gradual (simulasi perubahan bertahap)
            source = config[transition_from]
            # Interpolasi linear dari source ke target
            means = np.linspace(source["mean"], target["mean"], n_samples)
            stds = np.linspace(source["std"], target["std"], n_samples)
            values = np.array([np.random.normal(m, s) for m, s in zip(means, stds)])
        else:
            # Data stasioner (tidak berubah drastis)
            values = np.random.normal(target["mean"], target["std"], n_samples)

        # Clamp values (tidak boleh negatif untuk sensor fisik)
        values = np.maximum(values, 0)

        # Piezo di-clamp ke 0-100
        if sensor_name == "piezo":
            values = np.clip(values, 0, 100)

        data[sensor_name] = values

    data["label"] = LABELS[state]
    data["state"] = state.upper()

    return pd.DataFrame(data)


def generate_full_dataset(output_path: str, seed: int = 42):
    """
    Generate dataset lengkap dengan skenario realistis.
    
    Skenario:
    1. 6 jam data NORMAL (baseline)
    2. 30 menit transisi NORMAL → WATCH
    3. 1 jam data WATCH
    4. 15 menit transisi WATCH → WARNING
    5. 30 menit data WARNING
    6. 10 menit transisi WARNING → DANGER
    7. 20 menit data DANGER
    8. 4 jam data NORMAL lagi (setelah longsor mereda / sensor dikalibrasi ulang)
    9. Ulangi skenario longsor sekali lagi dengan variasi
    """
    np.random.seed(seed)
    
    segments = []
    current_time = datetime(2026, 1, 1, 0, 0, 0)  # Waktu simulasi

    # === SKENARIO 1: Hari Normal Pertama ===
    print("[SIM] Generating 6 jam data NORMAL...")
    seg = generate_segment("normal", 6 * 3600, start_time=current_time)
    segments.append(seg)
    current_time = seg["timestamp"].iloc[-1] + timedelta(seconds=10)

    # === SKENARIO 2: Transisi ke Bahaya ===
    print("[SIM] Generating transisi NORMAL → WATCH (30 menit)...")
    seg = generate_segment("watch", 30 * 60, start_time=current_time, transition_from="normal")
    segments.append(seg)
    current_time = seg["timestamp"].iloc[-1] + timedelta(seconds=10)

    print("[SIM] Generating 1 jam data WATCH...")
    seg = generate_segment("watch", 1 * 3600, start_time=current_time)
    segments.append(seg)
    current_time = seg["timestamp"].iloc[-1] + timedelta(seconds=10)

    print("[SIM] Generating transisi WATCH → WARNING (15 menit)...")
    seg = generate_segment("warning", 15 * 60, start_time=current_time, transition_from="watch")
    segments.append(seg)
    current_time = seg["timestamp"].iloc[-1] + timedelta(seconds=10)

    print("[SIM] Generating 30 menit data WARNING...")
    seg = generate_segment("warning", 30 * 60, start_time=current_time)
    segments.append(seg)
    current_time = seg["timestamp"].iloc[-1] + timedelta(seconds=10)

    print("[SIM] Generating transisi WARNING → DANGER (10 menit)...")
    seg = generate_segment("danger", 10 * 60, start_time=current_time, transition_from="warning")
    segments.append(seg)
    current_time = seg["timestamp"].iloc[-1] + timedelta(seconds=10)

    print("[SIM] Generating 20 menit data DANGER...")
    seg = generate_segment("danger", 20 * 60, start_time=current_time)
    segments.append(seg)
    current_time = seg["timestamp"].iloc[-1] + timedelta(seconds=10)

    # === SKENARIO 3: Kembali Normal ===
    print("[SIM] Generating 4 jam data NORMAL (post-event)...")
    seg = generate_segment("normal", 4 * 3600, start_time=current_time)
    segments.append(seg)
    current_time = seg["timestamp"].iloc[-1] + timedelta(seconds=10)

    # === SKENARIO 4: Longsor Kedua (variasi) ===
    np.random.seed(seed + 1)  # Variasi random berbeda
    
    print("[SIM] Generating skenario longsor kedua...")
    seg = generate_segment("normal", 3 * 3600, start_time=current_time)
    segments.append(seg)
    current_time = seg["timestamp"].iloc[-1] + timedelta(seconds=10)

    seg = generate_segment("watch", 45 * 60, start_time=current_time, transition_from="normal")
    segments.append(seg)
    current_time = seg["timestamp"].iloc[-1] + timedelta(seconds=10)

    seg = generate_segment("warning", 20 * 60, start_time=current_time, transition_from="watch")
    segments.append(seg)
    current_time = seg["timestamp"].iloc[-1] + timedelta(seconds=10)

    seg = generate_segment("danger", 15 * 60, start_time=current_time, transition_from="warning")
    segments.append(seg)
    current_time = seg["timestamp"].iloc[-1] + timedelta(seconds=10)

    seg = generate_segment("normal", 2 * 3600, start_time=current_time)
    segments.append(seg)

    # === GABUNGKAN ===
    df = pd.concat(segments, ignore_index=True)

    # Simpan ke CSV
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    df.to_csv(output_path, index=False)

    print(f"\n{'='*50}")
    print(f"✅ Dataset berhasil di-generate!")
    print(f"   File: {output_path}")
    print(f"   Total baris: {len(df):,}")
    print(f"\n   Distribusi label:")
    for state, count in df["state"].value_counts().items():
        print(f"     {state:10s}: {count:6,} baris ({count/len(df)*100:.1f}%)")
    print(f"{'='*50}")

    return df


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Landslide EWS Data Simulator")
    parser.add_argument("--output", default="data/dataset_simulated.csv",
                        help="Path output CSV (default: data/dataset_simulated.csv)")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed untuk reprodusibilitas")
    args = parser.parse_args()

    # Resolve path relatif ke folder python/
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(script_dir, args.output)

    generate_full_dataset(output_path, seed=args.seed)
