import pytest
import geopandas as gpd
from shapely.geometry import Polygon, LineString
from pathlib import Path
import tempfile
import yaml

from rbi_engine.config import RBIConfig, RBISource, LayerPattern, RoadBuffer
from rbi_engine.core import RBIEngine


def test_config_from_yaml():
    config_path = Path("rbi_config.yaml")
    config = RBIConfig.from_yaml(config_path)
    assert len(config.sources) > 0
    assert "settlement" in config.layer_patterns
    assert config.target_crs == "EPSG:4326"


def test_rbi_engine_with_synthetic_shapefile(tmp_path):
    shp_dir = tmp_path / "shp_data"
    shp_dir.mkdir()

    # Create synthetic settlement polygon shapefile: PERMUKIMAN_AR_25K.shp
    poly = Polygon([(107.5, -7.0), (107.6, -7.0), (107.6, -7.1), (107.5, -7.1), (107.5, -7.0)])
    gdf_settlement = gpd.GeoDataFrame(
        [{
            "NAMOBJ": "Permukiman Test",
            "REMARK": "Pemukiman",
            "geometry": poly
        }],
        crs="EPSG:4326"
    )
    gdf_settlement.to_file(shp_dir / "PERMUKIMAN_AR_25K.shp")

    # Create synthetic road line shapefile: JALAN_LN_25K.shp
    line = LineString([(107.5, -7.0), (107.6, -7.1)])
    gdf_road = gpd.GeoDataFrame(
        [{
            "NAMOBJ": "Jalan Utama",
            "REMARK": "Jalan Kolektor",
            "geometry": line
        }],
        crs="EPSG:4326"
    )
    gdf_road.to_file(shp_dir / "JALAN_LN_25K.shp")

    # Create test configuration
    config = RBIConfig(
        sources=[
            RBISource(
                name="test_region",
                region="TEST_REGION",
                type="shapefile",
                path=shp_dir
            )
        ],
        layer_patterns={
            "settlement": LayerPattern(
                category="settlement",
                layer_names=["PERMUKIMAN_AR_25K"],
                name_patterns=["PEMUKIMAN", "PERMUKIMAN"],
                geometry_type="polygon"
            ),
            "road": LayerPattern(
                category="road",
                layer_names=["JALAN_LN_25K"],
                name_patterns=["JALAN"],
                geometry_type="line"
            )
        },
        landuse_categories=[],
        road_buffers=[
            RoadBuffer(
                class_name="kolektor",
                width_meters=7.5,
                patterns=["KOLEKTOR"]
            )
        ],
        target_crs="EPSG:4326",
        metric_crs="EPSG:32748"
    )

    engine = RBIEngine(config)

    # 1. Test catalog_layers
    catalog = engine.catalog_layers()
    assert len(catalog) == 2
    layers = set(catalog['layer'])
    assert "PERMUKIMAN_AR_25K" in layers
    assert "JALAN_LN_25K" in layers

    # 2. Test load_category
    settlements = engine.load_category("settlement")
    assert not settlements.empty
    assert len(settlements) == 1
    assert settlements.iloc[0]["NAMOBJ"] == "Permukiman Test"
    assert settlements.iloc[0]["_source"] == "TEST_REGION"

    # 3. Test load_roads_buffered
    roads_buffered = engine.load_roads_buffered()
    assert not roads_buffered.empty
    assert len(roads_buffered) == 1
    assert roads_buffered.iloc[0]["road_class"] == "kolektor"
    assert roads_buffered.iloc[0]["buffer_width"] == 7.5
