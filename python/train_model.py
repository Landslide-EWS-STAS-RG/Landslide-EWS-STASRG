"""
train_model.py — Landslide EWS Model Training
===============================================
Melatih model Machine Learning (Isolation Forest + Autoencoder) dari dataset CSV.

FLEKSIBEL: Script ini menerima file CSV apapun, asalkan memiliki kolom:
  - tilt (float)
  - gravity (float)  
  - piezo (float)
  - label (int, opsional — 0=normal, 1=watch, 2=warning, 3=danger)

Untuk mengganti dataset:
  python train_model.py --data data/dataset_riil_lapangan.csv
"""

import os
import argparse
import joblib
import numpy as np
import pandas as pd
from sklearn.ensemble import IsolationForest
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import classification_report, confusion_matrix

# Cek apakah TensorFlow tersedia (opsional, untuk Autoencoder)
try:
    import tensorflow as tf
    from tensorflow.keras.models import Model, load_model
    from tensorflow.keras.layers import Input, Dense
    from tensorflow.keras.callbacks import EarlyStopping
    HAS_TENSORFLOW = True
except ImportError:
    HAS_TENSORFLOW = False
    print("[WARN] TensorFlow tidak terinstall. Autoencoder akan dilewati.")
    print("       Install dengan: pip install tensorflow")

# === KONFIGURASI ===
FEATURES = ["tilt", "gravity", "piezo"]  # Kolom input untuk ML
CONTAMINATION = 0.05  # Asumsi 5% data training mungkin anomali
RANDOM_STATE = 42


def load_dataset(csv_path: str) -> pd.DataFrame:
    """Load dataset dari CSV. Fleksibel — bisa data simulasi atau data riil."""
    print(f"[LOAD] Membaca dataset: {csv_path}")
    df = pd.read_csv(csv_path)

    # Validasi kolom wajib
    for col in FEATURES:
        if col not in df.columns:
            raise ValueError(f"Kolom '{col}' tidak ditemukan dalam CSV! "
                             f"Kolom yang tersedia: {list(df.columns)}")

    print(f"[LOAD] Total baris: {len(df):,}")
    print(f"[LOAD] Kolom: {list(df.columns)}")

    # Tampilkan statistik dasar
    print(f"\n[STATS] Statistik Deskriptif:")
    print(df[FEATURES].describe().round(4).to_string())

    return df


def train_isolation_forest(X_train: np.ndarray, scaler: StandardScaler,
                           save_dir: str) -> IsolationForest:
    """Latih model Isolation Forest dan simpan ke disk."""
    print("\n" + "=" * 50)
    print("🌲 Melatih Isolation Forest...")
    print("=" * 50)

    model = IsolationForest(
        contamination=CONTAMINATION,
        n_estimators=200,
        max_samples='auto',
        random_state=RANDOM_STATE,
        n_jobs=-1
    )
    model.fit(X_train)

    # Simpan model
    model_path = os.path.join(save_dir, "isolation_forest.pkl")
    joblib.dump(model, model_path)
    print(f"[SAVE] Model disimpan: {model_path}")

    return model


def train_autoencoder(X_train: np.ndarray, save_dir: str):
    """Latih Autoencoder Neural Network dan simpan ke disk."""
    if not HAS_TENSORFLOW:
        print("[SKIP] Autoencoder dilewati (TensorFlow tidak tersedia)")
        return None

    print("\n" + "=" * 50)
    print("🧠 Melatih Autoencoder...")
    print("=" * 50)

    input_dim = X_train.shape[1]

    # Arsitektur Autoencoder
    # Encoder: input_dim → 8 → 4 (bottleneck)
    # Decoder: 4 → 8 → input_dim
    input_layer = Input(shape=(input_dim,))
    encoded = Dense(8, activation='relu')(input_layer)
    encoded = Dense(4, activation='relu')(encoded)    # Bottleneck
    decoded = Dense(8, activation='relu')(encoded)
    decoded = Dense(input_dim, activation='linear')(decoded)

    autoencoder = Model(input_layer, decoded)
    autoencoder.compile(optimizer='adam', loss='mse')

    # Hanya latih dengan data NORMAL (label=0) jika ada label
    early_stop = EarlyStopping(monitor='val_loss', patience=10, restore_best_weights=True)

    autoencoder.fit(
        X_train, X_train,
        epochs=100,
        batch_size=32,
        shuffle=True,
        validation_split=0.2,
        callbacks=[early_stop],
        verbose=1
    )

    # Simpan model
    model_path = os.path.join(save_dir, "autoencoder.keras")
    autoencoder.save(model_path)
    print(f"[SAVE] Model disimpan: {model_path}")

    return autoencoder


def evaluate_models(df: pd.DataFrame, scaler: StandardScaler,
                    if_model: IsolationForest, ae_model=None):
    """Evaluasi performa model menggunakan label (jika tersedia)."""
    if "label" not in df.columns:
        print("\n[EVAL] Kolom 'label' tidak ada. Evaluasi dilewati.")
        print("       Tambahkan kolom 'label' ke CSV untuk evaluasi (0=normal, 1+=anomali)")
        return

    print("\n" + "=" * 50)
    print("📊 Evaluasi Model")
    print("=" * 50)

    X = scaler.transform(df[FEATURES].values)

    # Ground truth: 0 = normal, 1+ = anomali
    y_true = (df["label"] > 0).astype(int)  # Binary: 0=normal, 1=anomali

    # --- Isolation Forest ---
    if_pred = if_model.predict(X)
    # IF output: 1=normal, -1=anomali → konversi ke 0=normal, 1=anomali
    y_pred_if = (if_pred == -1).astype(int)

    print("\n--- Isolation Forest ---")
    print(classification_report(y_true, y_pred_if, target_names=["Normal", "Anomali"]))
    print("Confusion Matrix:")
    print(confusion_matrix(y_true, y_pred_if))

    # --- Autoencoder ---
    if ae_model is not None:
        reconstructions = ae_model.predict(X, verbose=0)
        mse = np.mean(np.power(X - reconstructions, 2), axis=1)
        
        # Tentukan threshold berdasarkan percentile data normal
        normal_mse = mse[y_true == 0]
        threshold = np.percentile(normal_mse, 95)
        
        y_pred_ae = (mse > threshold).astype(int)

        print("\n--- Autoencoder ---")
        print(f"Threshold MSE: {threshold:.6f}")
        print(classification_report(y_true, y_pred_ae, target_names=["Normal", "Anomali"]))
        print("Confusion Matrix:")
        print(confusion_matrix(y_true, y_pred_ae))

        # Simpan threshold
        script_dir = os.path.dirname(os.path.abspath(__file__))
        threshold_path = os.path.join(script_dir, "models", "ae_threshold.txt")
        with open(threshold_path, "w") as f:
            f.write(str(threshold))
        print(f"[SAVE] AE threshold disimpan: {threshold_path}")


def main():
    parser = argparse.ArgumentParser(description="Landslide EWS Model Training")
    parser.add_argument("--data", default="data/dataset_simulated.csv",
                        help="Path ke file CSV dataset (default: data/dataset_simulated.csv)")
    parser.add_argument("--skip-ae", action="store_true",
                        help="Lewati pelatihan Autoencoder")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_path = os.path.join(script_dir, args.data)
    models_dir = os.path.join(script_dir, "models")
    os.makedirs(models_dir, exist_ok=True)

    # 1. Load Dataset
    df = load_dataset(data_path)

    # 2. Preprocessing — StandardScaler (normalisasi)
    scaler = StandardScaler()
    X_all = df[FEATURES].values

    # Latih scaler dengan data NORMAL saja (jika ada label)
    if "label" in df.columns:
        X_normal = df[df["label"] == 0][FEATURES].values
        if len(X_normal) > 0:
            scaler.fit(X_normal)
            print(f"\n[SCALER] Fit pada {len(X_normal):,} baris data NORMAL")
        else:
            scaler.fit(X_all)
    else:
        scaler.fit(X_all)

    # Simpan scaler
    scaler_path = os.path.join(models_dir, "scaler.pkl")
    joblib.dump(scaler, scaler_path)
    print(f"[SAVE] Scaler disimpan: {scaler_path}")

    X_scaled = scaler.transform(X_all)

    # 3. Latih Isolation Forest
    if_model = train_isolation_forest(X_scaled, scaler, models_dir)

    # 4. Latih Autoencoder (opsional)
    ae_model = None
    if not args.skip_ae:
        # Autoencoder dilatih HANYA dengan data normal
        if "label" in df.columns:
            X_normal_scaled = scaler.transform(df[df["label"] == 0][FEATURES].values)
        else:
            X_normal_scaled = X_scaled
        ae_model = train_autoencoder(X_normal_scaled, models_dir)

    # 5. Evaluasi
    evaluate_models(df, scaler, if_model, ae_model)

    print("\n" + "=" * 50)
    print("✅ Training selesai! File yang dihasilkan:")
    print("=" * 50)
    for f in os.listdir(models_dir):
        fpath = os.path.join(models_dir, f)
        size = os.path.getsize(fpath)
        print(f"   📁 models/{f} ({size:,} bytes)")


if __name__ == "__main__":
    main()
