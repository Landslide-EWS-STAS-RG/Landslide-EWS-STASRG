# 🏔️ Landslide Early Warning System — STAS Research Group

> **Sistem Peringatan Dini Longsor** berbasis analisis geospasial data Rupabumi Indonesia (RBI) dari Badan Informasi Geospasial (BIG).

[![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-3776AB?style=flat-square&logo=python&logoColor=white)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg?style=flat-square)](https://opensource.org/licenses/MIT)
[![GeoPandas](https://img.shields.io/badge/GeoPandas-1.1%2B-139C5A?style=flat-square)](https://geopandas.org/)

---

## 📋 Daftar Isi

- [Gambaran Umum](#-gambaran-umum)
- [Arsitektur Sistem](#-arsitektur-sistem)
- [Instalasi](#-instalasi)
- [Komponen Utama](#-komponen-utama)
  - [RBI Engine](#1-rbi-engine---mesin-pemrosesan-data-rbi)
  - [Analisis Longsor](#2-analisis-kemiringan--potensi-longsor)
  - [Visualisasi Peta](#3-visualisasi-peta-gis)
- [Konfigurasi](#-konfigurasi)
- [Penggunaan](#-penggunaan)
- [Metodologi](#-metodologi)
- [Struktur Proyek](#-struktur-proyek)
- [Testing](#-testing)
- [Kontributor](#-kontributor)
- [Lisensi](#-lisensi)

---

## 🌍 Gambaran Umum

Proyek ini merupakan sistem analisis kerentanan longsor yang menggunakan data **Rupabumi Indonesia (RBI) skala 1:25.000** dari BIG sebagai sumber data geospasial utama. Sistem ini dirancang untuk:

1. **Memproses data RBI** — Membaca dan mengkategorisasi layer geospasial (permukiman, hutan, jalan, sungai, dll.) dari format Geodatabase (`.gdb`) maupun Shapefile (`.shp`).
2. **Menganalisis kemiringan lereng** — Menghitung Digital Elevation Model (DEM) dari garis kontur dan spot height, kemudian mengklasifikasikan kemiringan lereng berdasarkan **Van Zuidam (1985)**.
3. **Memetakan zona bahaya longsor** — Menghasilkan peta interaktif (Folium/Leaflet) dan peta statis (Matplotlib) yang menunjukkan sebaran zona curam, profil penampang lereng, dan statistik bahaya.

### Studi Kasus: Desa Tribaktimulya, Pangalengan

Studi kasus utama difokuskan pada **Desa Tribaktimulya, Kecamatan Pangalengan, Kabupaten Bandung** — wilayah pegunungan dengan topografi curam yang rawan longsor.

#### 🗺️ Peta Analisis Zona Curam & Risiko Longsor

![Peta Analisis Zona Curam & Risiko Longsor — Desa Tribaktimulya, Pangalengan](docs/images/peta_zona_curam_tribaktimulya.png)

> **Panel A.** Sebaran zona curam (>13°) seluruh desa — **Panel B.** Detail topografi area pengamatan — **Panel C.** Profil penampang melintang lereng A–A' — **Panel D.** Distribusi statistik kemiringan & bahaya longsor.

---

## 🏗 Arsitektur Sistem

```
┌─────────────────────────────────────────────────────────┐
│                    DATA SOURCES                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │   RBI GDB    │  │  Shapefile   │  │   GeoJSON    │  │
│  │  (BIG 25K)   │  │    (.shp)    │  │  (Boundary)  │  │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  │
└─────────┼──────────────────┼──────────────────┼─────────┘
          │                  │                  │
          ▼                  ▼                  ▼
┌─────────────────────────────────────────────────────────┐
│                    RBI ENGINE                           │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │
│  │   Config     │  │   Loaders   │  │    Core      │    │
│  │  (YAML)      │  │ GDB / SHP   │  │  RBIEngine   │    │
│  └─────────────┘  └─────────────┘  └─────────────┘    │
└─────────────────────────┬───────────────────────────────┘
                          │
          ┌───────────────┼───────────────┐
          ▼               ▼               ▼
┌─────────────┐  ┌──────────────┐  ┌──────────────┐
│  Landslide  │  │  Visualize   │  │   Run Demo   │
│  Analysis   │  │     Map      │  │              │
│ (Slope/DEM) │  │  (Folium +   │  │  (Synthetic  │
│             │  │  Matplotlib) │  │   Data OK)   │
└──────┬──────┘  └──────┬───────┘  └──────────────┘
       │                │
       ▼                ▼
┌─────────────────────────────────────────────────────────┐
│                    OUTPUT                               │
│  📊 Peta Interaktif (.html)   📈 Peta Statis (.png)    │
│  📋 Statistik Kemiringan      🗺️ Profil Penampang      │
└─────────────────────────────────────────────────────────┘
```

---

## ⚙️ Instalasi

### Prasyarat

- Python ≥ 3.8
- `pip` atau `uv`

### Langkah Instalasi

```bash
# Clone repository
git clone https://github.com/Landslide-EWS-STAS-RG/Landslide-EWS-STASRG.git
cd Landslide-EWS-STASRG

# Buat virtual environment
python -m venv .venv
source .venv/bin/activate   # Linux/macOS
# .venv\Scripts\activate    # Windows

# Install package beserta dependensi
pip install -e ".[dev]"
```

### Dependensi Utama

| Package       | Versi Minimum | Kegunaan                                     |
|---------------|:--------------|----------------------------------------------|
| `geopandas`   | ≥ 1.1.2       | Manipulasi data geospasial (GeoDataFrame)    |
| `pandas`      | ≥ 3.0.0       | Tabel data dan analisis                      |
| `fiona`       | ≥ 1.10.1      | Membaca layer dari Geodatabase (`.gdb`)      |
| `shapely`     | ≥ 2.1.2       | Operasi geometri (buffer, clip, contains)    |
| `pyproj`      | ≥ 3.7.2       | Transformasi proyeksi koordinat (CRS)        |
| `numpy`       | ≥ 2.4.1       | Komputasi numerik (DEM, slope)               |
| `scipy`       | —             | Interpolasi grid elevasi (`griddata`)        |
| `folium`      | —             | Peta web interaktif (Leaflet.js)             |
| `matplotlib`  | —             | Peta statis dan grafik statistik             |
| `PyYAML`      | ≥ 6.0.3       | Parsing konfigurasi YAML                     |

---

## 🧩 Komponen Utama

### 1. RBI Engine — Mesin Pemrosesan Data RBI

**Package:** [`rbi_engine/`](rbi_engine/)

Engine modular untuk membaca, mengkategorisasi, dan memuat data Rupabumi Indonesia (RBI) dari berbagai format sumber.

#### Modul

| File                                  | Deskripsi                                                          |
|---------------------------------------|--------------------------------------------------------------------|
| [`config.py`](rbi_engine/config.py)   | Dataclass konfigurasi (`RBIConfig`, `RBISource`, `LayerPattern`, `LandUseCategory`, `RoadBuffer`) |
| [`core.py`](rbi_engine/core.py)       | `RBIEngine` — Mesin utama: catalog, load category, harmonize      |
| [`loaders.py`](rbi_engine/loaders.py) | Loader abstrak + implementasi `GDBLoader` dan `ShapefileLoader`   |

#### Fitur Utama

- **Dual-format loader** — Mendukung Geodatabase (`.gdb`) dan Shapefile (`.shp`) secara transparan
- **Pattern-based categorization** — Mengenali kategori layer secara otomatis berdasarkan nama dan pola (regex)
- **Layer cataloging** — Inventarisasi otomatis semua layer yang tersedia beserta metadata
- **Field harmonization** — Menyamakan nama kolom antar-sumber data yang berbeda
- **Road buffering** — Konversi garis jalan menjadi polygon berdasarkan kelas jalan (Tol, Arteri, Kolektor, Lokal, Setapak)
- **CRS management** — Transformasi otomatis ke proyeksi target (WGS 84 / UTM 48S)

#### Contoh Penggunaan

```python
from rbi_engine import RBIEngine

# Inisialisasi dari file konfigurasi
engine = RBIEngine.from_yaml("rbi_config.yaml")

# Lihat semua layer yang tersedia
catalog = engine.catalog_layers()
print(catalog[["source", "layer", "category", "geometry_type"]])

# Muat data permukiman
settlements = engine.load_category("settlement")
print(settlements[["NAMOBJ", "geometry"]].head())

# Muat jalan dengan buffer otomatis
roads = engine.load_roads_buffered(default_buffer=3.0)
print(roads[["NAMOBJ", "road_class", "buffer_width"]].head())
```

---

### 2. Analisis Kemiringan & Potensi Longsor

**Script:** [`analisis_longsor_tribaktimulya.py`](analisis_longsor_tribaktimulya.py)

Analisis geomorfologi mendalam untuk Desa Tribaktimulya dengan fitur:

#### Pipeline Analisis

1. **Ekstraksi Data Topografi**
   - Batas administrasi desa dari layer `ADMINISTRASI_AR_DESAKEL`
   - Garis kontur 25K (`KONTUR_LN_25K`) dan titik tinggi (`SPOTHEIGHT_PT_25K`)
   - Jaringan sungai, jalan, dan tutupan lahan

2. **Pembuatan DEM & Perhitungan Slope**
   - Grid elevasi beresolusi tinggi (220×220 piksel) via interpolasi linear (`scipy.griddata`)
   - Kemiringan lereng dalam derajat via gradient metrik (UTM 48S)
   - Masking otomatis ke batas desa

3. **Klasifikasi Kemiringan (Van Zuidam, 1985)**

   | Kelas              | Kemiringan (°) | Tingkat Bahaya   |
   |---------------------|:--------------:|:-----------------|
   | Datar               | 0 – 2         | Sangat Rendah    |
   | Landai              | 2 – 7         | Rendah           |
   | Agak Curam          | 7 – 13        | Sedang           |
   | Curam               | 13 – 35       | **Tinggi**       |
   | Sangat Curam/Terjal | > 35          | **Sangat Tinggi**|

4. **Analisis Titik Pengamatan** (`-7.142425, 107.567892`)
   - Elevasi, kemiringan, arah lereng (aspect)
   - Tutupan lahan, jarak ke sungai dan jalan
   - Kemiringan maksimum dalam radius 200m

5. **Output Visualisasi**
   - **Peta interaktif** (Folium/Leaflet) dengan overlay zona curam, kontur, sungai, jalan
   - **Peta statis 4-panel** (Matplotlib):
     - A. Sebaran zona curam seluruh desa
     - B. Detail topografi area pengamatan (radius 450m)
     - C. Profil penampang melintang lereng A – A'
     - D. Distribusi statistik kemiringan & bahaya longsor

#### Hasil Analisis Potensi Bahaya Longsor

![Analisis Potensi Bahaya Longsor — Desa Tribaktimulya](docs/images/peta_tribaktimulya_analisis.png)

> **Kiri:** Peta kontur & elevasi — **Tengah:** Peta kemiringan lereng (Van Zuidam) — **Kanan:** Peta tutupan lahan.

---

### 3. Visualisasi Peta GIS

**Script:** [`visualize_map.py`](visualize_map.py)

Visualisasi data RBI multi-layer untuk skala kabupaten:

- **Peta interaktif** (Folium) — Layer permukiman, hutan, perairan, jalan dengan basemap satelit Esri dan OpenTopoMap
- **Peta statis** (Matplotlib) — Overlay penutup lahan dan jaringan jalan Kabupaten Bandung

![Peta Penutup Lahan & Jaringan Jalan — Kabupaten Bandung (RBI)](docs/images/peta_kab_bandung.png)

> Peta penutup lahan Kabupaten Bandung menampilkan overlay hutan, perkebunan, danau/waduk, permukiman, dan jaringan jalan dari data RBI 25K.

---

## 🔧 Konfigurasi

Konfigurasi engine dikelola melalui file **YAML** ([`rbi_config.yaml`](rbi_config.yaml)):

```yaml
# Sumber data RBI
sources:
  - name: "kab_bandung"
    region: "KAB_BANDUNG"
    type: "gdb"                    # "gdb" atau "shapefile"
    path: "./data/KAB BANDUNG/RBI25K_KAB_BANDUNG_KUGI50_20221231.gdb"

# Pola pencocokan layer per kategori
layer_patterns:
  settlement:
    category: "settlement"
    layer_names: ["PERMUKIMAN_AR_25K", "RBI25K_PEMUKIMAN_AR"]
    name_patterns: ["PEMUKIMAN", "PERMUKIMAN", "RESIDENTIAL"]
    geometry_type: "polygon"
  # ... (forest, shrub, plantation, paddy, farmland, water, road, etc.)

# Kategori tutupan lahan dengan bobot & prioritas
landuse_categories:
  - name: "water"
    weight: 1.00
    patterns: ["SUNGAI", "DANAU", "WADUK"]
    priority: 100
    description: "Water bodies"
  # ...

# Buffer jalan berdasarkan kelas
road_buffers:
  - class_name: "highway"
    width_meters: 12.0
    patterns: ["TOL", "HIGHWAY", "MOTORWAY"]
  # ...

# Proyeksi koordinat
target_crs: "EPSG:4326"    # WGS 84
metric_crs: "EPSG:32748"   # UTM Zone 48S
```

---

## 🚀 Penggunaan

### Quick Start (Tanpa Data RBI)

Script demo dapat berjalan tanpa data RBI asli — otomatis membuat data sintetis:

```bash
python run_demo.py
```

### Dengan Data RBI Asli

1. **Unduh data RBI** dari [Portal Tana Air BIG](https://tanahair.indonesia.go.id/portal-web/unduh)
2. Letakkan file `.gdb` atau `.shp` di folder `data/`
3. Sesuaikan `rbi_config.yaml` dengan path data Anda
4. Jalankan:

```bash
# Visualisasi peta Kabupaten Bandung
python visualize_map.py

# Analisis longsor Desa Tribaktimulya
python analisis_longsor_tribaktimulya.py
```

### Output

| Output | File | Deskripsi |
|--------|------|-----------|
| Peta interaktif kabupaten | `peta_kab_bandung.html` | Peta web multi-layer |
| Peta statis kabupaten | `peta_kab_bandung.png` | Gambar PNG penutup lahan |
| Peta interaktif zona curam | `peta_tribaktimulya_interaktif.html` | Zona curam + kontur + overlay |
| Peta analisis statis | `peta_zona_curam_tribaktimulya.png` | 4-panel analisis lengkap |

---

## 📐 Metodologi

### Sumber Data

- **Rupabumi Indonesia (RBI) 1:25.000** — BIG (Badan Informasi Geospasial)
  - Format: ESRI File Geodatabase (`.gdb`)
  - Wilayah: Kabupaten Bandung (Edisi 2022)
  - Layer: Kontur, spot height, administrasi, permukiman, hutan, sungai, jalan, dll.

### Perhitungan DEM & Slope

1. Ekstraksi titik elevasi dari garis kontur (`KONTUR_LN_25K`) dan titik tinggi (`SPOTHEIGHT_PT_25K`)
2. Interpolasi linear menggunakan `scipy.griddata` pada grid metrik UTM 48S
3. Perhitungan gradient kemiringan: `slope = arctan(√(dz/dx² + dz/dy²))`
4. Konversi ke derajat dan masking ke boundary desa

### Klasifikasi Kemiringan Lereng

Mengacu pada **Van Zuidam (1985)** — standar klasifikasi geomorfologi yang umum digunakan di Indonesia untuk pemetaan kerentanan gerakan tanah.

### Analisis Bahaya Longsor

Faktor yang dipertimbangkan:
- **Kemiringan lereng** (slope angle)
- **Arah kemiringan** (aspect/downslope direction)
- **Tutupan lahan** (land cover — vegetasi vs tegalan)
- **Kedekatan dengan alur sungai** (proximity to drainage)
- **Kedekatan dengan infrastruktur jalan** (cut-slope hazard)

---

## 📁 Struktur Proyek

```
Landslide-EWS-STASRG/
│
├── rbi_engine/                         # Package utama RBI Engine
│   ├── __init__.py                     # Public API exports
│   ├── config.py                       # Dataclass konfigurasi
│   ├── core.py                         # RBIEngine (catalog, load, harmonize)
│   └── loaders.py                      # GDBLoader, ShapefileLoader
│
├── tests/
│   └── test_engine.py                  # Unit tests (pytest)
│
├── analisis_longsor_tribaktimulya.py   # Analisis kemiringan & longsor
├── visualize_map.py                    # Visualisasi peta GIS multi-layer
├── run_demo.py                         # Demo script (dengan data sintetis)
│
├── rbi_config.yaml                     # Konfigurasi sumber data & layer
├── tribaktimulya_boundary.geojson      # Batas desa Tribaktimulya
├── pyproject.toml                      # Konfigurasi package Python
├── .gitignore                          # Git ignore rules
└── README.md                           # Dokumentasi ini
```

---

## 🧪 Testing

```bash
# Install dependensi dev
pip install -e ".[dev]"

# Jalankan test suite
pytest tests/ -v
```

Test mencakup:
- Parsing konfigurasi YAML
- Loading layer dari shapefile sintetis
- Katalogisasi layer otomatis
- Loading per kategori dan verifikasi atribut
- Buffering jalan berdasarkan kelas

---

## 👥 Kontributor

| Nama | Peran |
|------|-------|
| **STAS Research Group** | Tim Penelitian |
| Rahman Hakim | Pengembang RBI Engine |

---

## 📜 Lisensi

Proyek ini dilisensikan di bawah [MIT License](https://opensource.org/licenses/MIT).

---

## 📚 Referensi

- **Van Zuidam, R. A.** (1985). *Aerial Photo-Interpretation in Terrain Analysis and Geomorphologic Mapping*. Smits Publishers, The Hague.
- **Badan Informasi Geospasial (BIG)** — [Portal Tana Air](https://tanahair.indonesia.go.id/portal-web/unduh)
- **BNPB** — Data Informasi Bencana Indonesia ([DIBI](https://dibi.bnpb.go.id/))
