import sys
import webbrowser
from pathlib import Path
import folium
from folium import LayerControl
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D

from rbi_engine import RBIEngine


def create_interactive_map(output_html="peta_kab_bandung.html"):
    print("=" * 60)
    print("   Membuat Peta GIS Interaktif (Leaflet Web Map)")
    print("=" * 60)

    engine = RBIEngine.from_yaml("rbi_config.yaml")

    # Inisialisasi peta berpusat di Kabupaten Bandung
    m = folium.Map(
        location=[-7.05, 107.55],
        zoom_start=10,
        tiles="OpenStreetMap",
        control_scale=True
    )

    # Tambahkan layer satelit Esri dan Peta Topo
    folium.TileLayer(
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Esri World Imagery",
        name="Citra Satelit (Esri)",
        overlay=False
    ).add_to(m)

    folium.TileLayer(
        tiles="https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png",
        attr="OpenTopoMap",
        name="Peta Topografi (OpenTopoMap)",
        overlay=False
    ).add_to(m)

    # 1. Permukiman (Settlement)
    print("[*] Memuat layer Permukiman...")
    settlements = engine.load_category("settlement")
    if not settlements.empty:
        settlements_sub = settlements.head(500)
        fg_settlement = folium.FeatureGroup(name="Permukiman (Settlement)", show=True)
        folium.GeoJson(
            settlements_sub,
            style_function=lambda x: {
                "fillColor": "#e74c3c",
                "color": "#c0392b",
                "weight": 1,
                "fillOpacity": 0.5
            },
            tooltip=folium.GeoJsonTooltip(fields=["NAMOBJ", "REMARK"], aliases=["Nama Objek:", "Keterangan:"])
        ).add_to(fg_settlement)
        fg_settlement.add_to(m)
        print(f"    -> {len(settlements_sub)} fitur permukiman dimuat.")

    # 2. Hutan (Forest)
    print("[*] Memuat layer Hutan...")
    forest = engine.load_category("forest")
    if not forest.empty:
        forest_sub = forest.head(300)
        fg_forest = folium.FeatureGroup(name="Hutan (Forest)", show=True)
        folium.GeoJson(
            forest_sub,
            style_function=lambda x: {
                "fillColor": "#27ae60",
                "color": "#1e8449",
                "weight": 1,
                "fillOpacity": 0.5
            },
            tooltip=folium.GeoJsonTooltip(fields=["NAMOBJ", "REMARK"], aliases=["Nama Objek:", "Keterangan:"])
        ).add_to(fg_forest)
        fg_forest.add_to(m)
        print(f"    -> {len(forest_sub)} fitur hutan dimuat.")

    # 3. Air / Danau / Waduk (Water)
    print("[*] Memuat layer Perairan (Water)...")
    water = engine.load_category("water", geometry_type="polygon")
    if not water.empty:
        water_sub = water.head(300)
        fg_water = folium.FeatureGroup(name="Danau / Waduk (Water)", show=True)
        folium.GeoJson(
            water_sub,
            style_function=lambda x: {
                "fillColor": "#3498db",
                "color": "#2980b9",
                "weight": 1,
                "fillOpacity": 0.6
            },
            tooltip=folium.GeoJsonTooltip(fields=["NAMOBJ", "REMARK"], aliases=["Nama Objek:", "Keterangan:"])
        ).add_to(fg_water)
        fg_water.add_to(m)
        print(f"    -> {len(water_sub)} fitur perairan dimuat.")

    # 4. Jalan (Roads)
    print("[*] Memuat layer Jaringan Jalan...")
    roads = engine.load_category("road", geometry_type="line")
    if not roads.empty:
        roads_sub = roads.head(400)
        fg_road = folium.FeatureGroup(name="Jaringan Jalan", show=True)
        folium.GeoJson(
            roads_sub,
            style_function=lambda x: {
                "color": "#f39c12",
                "weight": 2.5,
                "opacity": 0.8
            },
            tooltip=folium.GeoJsonTooltip(fields=["NAMOBJ", "REMARK"], aliases=["Nama Jalan:", "Kelas:"])
        ).add_to(fg_road)
        fg_road.add_to(m)
        print(f"    -> {len(roads_sub)} garis jalan dimuat.")

    # Kontrol layer
    LayerControl(position="topright", collapsed=False).add_to(m)

    output_path = Path(output_html).resolve()
    m.save(str(output_path))
    print(f"[+] Peta interaktif berhasil disimpan di: {output_path}")

    return output_path


def create_static_plot(output_png="peta_kab_bandung.png"):
    print("=" * 60)
    print("   Membuat Gambar Peta Statis (Matplotlib PNG)")
    print("=" * 60)

    engine = RBIEngine.from_yaml("rbi_config.yaml")
    fig, ax = plt.subplots(figsize=(11, 9))

    legend_elements = []

    # 1. Hutan
    print("[*] Plotting layer Hutan...")
    forest = engine.load_category("forest")
    if not forest.empty:
        forest.plot(ax=ax, color="#27ae60", alpha=0.6)
        legend_elements.append(Patch(facecolor="#27ae60", edgecolor="#1e8449", alpha=0.6, label="Hutan"))

    # 2. Perkebunan / Ladang
    print("[*] Plotting layer Perkebunan & Ladang...")
    plantation = engine.load_category("plantation")
    if not plantation.empty:
        plantation.plot(ax=ax, color="#f1c40f", alpha=0.5)
        legend_elements.append(Patch(facecolor="#f1c40f", edgecolor="#d4ac0d", alpha=0.5, label="Perkebunan"))

    # 3. Danau / Waduk
    print("[*] Plotting layer Perairan...")
    water = engine.load_category("water", geometry_type="polygon")
    if not water.empty:
        water.plot(ax=ax, color="#3498db", alpha=0.8)
        legend_elements.append(Patch(facecolor="#3498db", edgecolor="#2980b9", alpha=0.8, label="Danau/Waduk"))

    # 4. Permukiman
    print("[*] Plotting layer Permukiman...")
    settlements = engine.load_category("settlement")
    if not settlements.empty:
        settlements.plot(ax=ax, color="#e74c3c", alpha=0.5)
        legend_elements.append(Patch(facecolor="#e74c3c", edgecolor="#c0392b", alpha=0.5, label="Permukiman"))

    # 5. Jalan
    print("[*] Plotting layer Jalan...")
    roads = engine.load_category("road", geometry_type="line")
    if not roads.empty:
        roads.plot(ax=ax, color="#2c3e50", linewidth=0.7, alpha=0.8)
        legend_elements.append(Line2D([0], [0], color="#2c3e50", lw=1.2, label="Jalan"))

    ax.set_title("Peta Penutup Lahan & Jaringan Jalan - Kabupaten Bandung (RBI)", fontsize=13, fontweight="bold", pad=12)
    ax.set_xlabel("Bujur / Longitude (°E)", fontsize=10)
    ax.set_ylabel("Lintang / Latitude (°S)", fontsize=10)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(handles=legend_elements, loc="lower left", framealpha=0.9, fontsize=9)

    plt.tight_layout()
    output_path = Path(output_png).resolve()
    plt.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close()
    print(f"[+] Peta statis berhasil disimpan di: {output_path}")

    return output_path


def main():
    html_file = create_interactive_map("peta_kab_bandung.html")
    png_file = create_static_plot("peta_kab_bandung.png")

    print("\n" + "=" * 60)
    print("Semua visualisasi berhasil dibuat!")
    print(f"1. Peta Web Interaktif : {html_file}")
    print(f"2. Gambar Peta Statis  : {png_file}")
    print("=" * 60)

    # Otomatis buka file HTML di browser
    try:
        webbrowser.open(f"file://{html_file}")
        print("[+] Membuka peta interaktif di browser Anda...")
    except Exception:
        pass


if __name__ == "__main__":
    main()
