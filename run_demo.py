import sys
from pathlib import Path
import geopandas as gpd
from shapely.geometry import Polygon, LineString

from rbi_engine import RBIEngine
from rbi_engine.config import RBIConfig, RBISource, LayerPattern, RoadBuffer


def main():
    print("=" * 60)
    print("       RBI Engine (Rupa Bumi Indonesia) - Demo")
    print("=" * 60)

    config_path = Path("rbi_config.yaml")

    if not config_path.exists():
        print(f"[ERROR] Config file '{config_path}' not found.")
        sys.exit(1)

    print(f"[*] Reading configuration: {config_path}")
    default_config = RBIConfig.from_yaml(config_path)

    # Check if configured sources exist
    found_sources = [s for s in default_config.sources if s.path.exists()]

    if found_sources:
        print(f"[+] Found {len(found_sources)} configured data source(s). Running with real data...")
        engine = RBIEngine(default_config)
    else:
        print("[-] Real RBI data not detected in configured paths.")
        print("    (Download .gdb/.shp from: https://tanahair.indonesia.go.id/portal-web/unduh)")
        print("[*] Creating synthetic RBI sample data in './sample_data' for demonstration...")

        sample_dir = Path("sample_data")
        sample_dir.mkdir(exist_ok=True)

        # Create sample settlement polygon
        settlement_shp = sample_dir / "PERMUKIMAN_AR_25K.shp"
        poly = Polygon([(107.50, -7.00), (107.55, -7.00), (107.55, -7.05), (107.50, -7.05), (107.50, -7.00)])
        gdf_settlement = gpd.GeoDataFrame(
            [{
                "NAMOBJ": "Permukiman Contoh",
                "REMARK": "Permukiman Penduduk",
                "geometry": poly
            }],
            crs="EPSG:4326"
        )
        gdf_settlement.to_file(settlement_shp)

        # Create sample road line
        road_shp = sample_dir / "JALAN_LN_25K.shp"
        line = LineString([(107.49, -7.02), (107.56, -7.02)])
        gdf_road = gpd.GeoDataFrame(
            [{
                "NAMOBJ": "Jl. Raya Utama",
                "REMARK": "Jalan Kolektor Primer",
                "geometry": line
            }],
            crs="EPSG:4326"
        )
        gdf_road.to_file(road_shp)

        # Create demo config pointing to sample data
        demo_config = RBIConfig(
            sources=[
                RBISource(
                    name="sample_region",
                    region="DEMO_KAB_BANDUNG",
                    type="shapefile",
                    path=sample_dir
                )
            ],
            layer_patterns=default_config.layer_patterns,
            landuse_categories=default_config.landuse_categories,
            road_buffers=default_config.road_buffers,
            target_crs=default_config.target_crs,
            metric_crs=default_config.metric_crs
        )
        engine = RBIEngine(demo_config)

    print("\n1. Cataloging available layers:")
    catalog = engine.catalog_layers()
    print(catalog[["source", "region", "layer", "category", "geometry_type"]])

    print("\n2. Loading 'settlement' category:")
    settlements = engine.load_category("settlement")
    if not settlements.empty:
        print(settlements[["NAMOBJ", "REMARK", "_source", "geometry"]].head())
    else:
        print("No settlement layers loaded.")

    print("\n3. Loading 'road' buffered:")
    roads = engine.load_roads_buffered()
    if not roads.empty:
        print(roads[["NAMOBJ", "road_class", "buffer_width", "geometry"]].head())
    else:
        print("No road layers loaded.")

    print("\n" + "=" * 60)
    print("Demo executed successfully!")
    print("=" * 60)


if __name__ == "__main__":
    main()
