import streamlit as st
import pandas as pd
import plotly.graph_objects as go
import numpy as np
import pydeck as pdk
from heartbeat_sim import DroneHeartbeatSimulator
import time
from datetime import datetime
import math

# ========== 坐标系转换（WGS-84 ↔ GCJ-02）==========
def wgs84_to_gcj02(lat, lon):
    a = 6378245.0
    ee = 0.00669342162296594323
    def transform_lat(lat, lon):
        ret = -100.0 + 2.0 * lon + 3.0 * lat + 0.2 * lat * lat + 0.1 * lon * lat + 0.2 * math.sqrt(abs(lat))
        ret += (20.0 * math.sin(6.0 * lat * math.pi) + 20.0 * math.sin(2.0 * lat * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lon * math.pi) + 40.0 * math.sin(lon / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(lon / 12.0 * math.pi) + 320 * math.sin(lon * math.pi / 30.0)) * 2.0 / 3.0
        return ret
    def transform_lon(lat, lon):
        ret = 300.0 + lon + 2.0 * lat + 0.1 * lon * lon + 0.1 * lon * lat + 0.1 * math.sqrt(abs(lon))
        ret += (20.0 * math.sin(6.0 * lon * math.pi) + 20.0 * math.sin(2.0 * lon * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lon * math.pi) + 40.0 * math.sin(lon / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(lon / 12.0 * math.pi) + 300.0 * math.sin(lon / 30.0 * math.pi)) * 2.0 / 3.0
        return ret
    dlat = transform_lat(lat - 35.0, lon - 105.0)
    dlon = transform_lon(lat - 35.0, lon - 105.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlon = (dlon * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    return lat + dlat, lon + dlon

def gcj02_to_wgs84(lat, lon):
    a = 6378245.0
    ee = 0.00669342162296594323
    def transform_lat(lat, lon):
        ret = -100.0 + 2.0 * lon + 3.0 * lat + 0.2 * lat * lat + 0.1 * lon * lat + 0.2 * math.sqrt(abs(lat))
        ret += (20.0 * math.sin(6.0 * lat * math.pi) + 20.0 * math.sin(2.0 * lat * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lon * math.pi) + 40.0 * math.sin(lon / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(lon / 12.0 * math.pi) + 320 * math.sin(lon * math.pi / 30.0)) * 2.0 / 3.0
        return ret
    def transform_lon(lat, lon):
        ret = 300.0 + lon + 2.0 * lat + 0.1 * lon * lon + 0.1 * lon * lat + 0.1 * math.sqrt(abs(lon))
        ret += (20.0 * math.sin(6.0 * lon * math.pi) + 20.0 * math.sin(2.0 * lon * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lon * math.pi) + 40.0 * math.sin(lon / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(lon / 12.0 * math.pi) + 300.0 * math.sin(lon / 30.0 * math.pi)) * 2.0 / 3.0
        return ret
    dlat = transform_lat(lat - 35.0, lon - 105.0)
    dlon = transform_lon(lat - 35.0, lon - 105.0)
    radlat = lat / 180.0 * math.pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
    dlon = (dlon * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
    return lat - dlat, lon - dlon

def convert_coordinate(lat, lon, from_crs, to_crs="WGS-84"):
    if from_crs == to_crs:
        return lat, lon
    if from_crs == "WGS-84" and to_crs == "GCJ-02":
        return wgs84_to_gcj02(lat, lon)
    if from_crs == "GCJ-02" and to_crs == "WGS-84":
        return gcj02_to_wgs84(lat, lon)
    return lat, lon

def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

# ========== 页面配置 ==========
st.set_page_config(page_title="无人机智能化应用系统", page_icon="🚁", layout="wide")

# 初始化
if 'a_point' not in st.session_state:
    st.session_state.a_point = {"lat": 32.2322, "lon": 118.7490, "set": True}
if 'b_point' not in st.session_state:
    st.session_state.b_point = {"lat": 32.2345, "lon": 118.7505, "set": True}
if 'coord_system' not in st.session_state:
    st.session_state.coord_system = "GCJ-02"
if 'simulator' not in st.session_state:
    st.session_state.simulator = DroneHeartbeatSimulator()
    st.session_state.running = False
if 'flight_height' not in st.session_state:
    st.session_state.flight_height = 50

# ========== 页面标题 ==========
st.title("🗺️ 无人机航线规划系统 - 南京科技职业学院")
st.markdown("---")

# ========== 左右两列布局 ==========
col_left, col_right = st.columns([1, 2], gap="large")

# ========== 左侧控制面板 ==========
with col_left:
    st.markdown("### 🎮 控制面板")
    st.markdown("---")
    
    # 起点A
    st.markdown("#### 📍 起点A")
    st.caption("输入坐标：GCJ-02（高德地图）")
    col_a1, col_a2 = st.columns(2)
    with col_a1:
        a_lat = st.number_input("纬度", value=st.session_state.a_point["lat"], format="%.6f", key="a_lat")
    with col_a2:
        a_lon = st.number_input("经度", value=st.session_state.a_point["lon"], format="%.6f", key="a_lon")
    
    if st.button("📍 设置A点", key="set_a", use_container_width=True):
        display_lat, display_lon = convert_coordinate(a_lat, a_lon, "GCJ-02", "WGS-84")
        st.session_state.a_point = {"lat": display_lat, "lon": display_lon, "set": True}
        st.success(f"✅ A点已设置 (GCJ-02: {a_lat:.6f}, {a_lon:.6f})")
        st.rerun()
    
    st.markdown("---")
    
    # 终点B
    st.markdown("#### 📍 终点B")
    st.caption("输入坐标：GCJ-02（高德地图）")
    col_b1, col_b2 = st.columns(2)
    with col_b1:
        b_lat = st.number_input("纬度", value=st.session_state.b_point["lat"], format="%.6f", key="b_lat")
    with col_b2:
        b_lon = st.number_input("经度", value=st.session_state.b_point["lon"], format="%.6f", key="b_lon")
    
    if st.button("📍 设置B点", key="set_b", use_container_width=True):
        display_lat, display_lon = convert_coordinate(b_lat, b_lon, "GCJ-02", "WGS-84")
        st.session_state.b_point = {"lat": display_lat, "lon": display_lon, "set": True}
        st.success(f"✅ B点已设置 (GCJ-02: {b_lat:.6f}, {b_lon:.6f})")
        st.rerun()
    
    st.markdown("---")
    
    # 飞行参数
    st.markdown("#### ✈️ 飞行参数")
    st.session_state.flight_height = st.slider("设定飞行高度(m)", 20, 200, 50)
    
    st.markdown("---")
    
    # 系统状态
    st.markdown("#### 📊 系统状态")
    if st.session_state.a_point["set"]:
        st.success("✅ A点已设")
    else:
        st.info("⚪ A点未设")
    
    if st.session_state.b_point["set"]:
        st.success("✅ B点已设")
    else:
        st.info("⚪ B点未设")
    
    # 显示距离
    if st.session_state.a_point["set"] and st.session_state.b_point["set"]:
        gcj_a_lat, gcj_a_lon = convert_coordinate(
            st.session_state.a_point["lat"], st.session_state.a_point["lon"],
            "WGS-84", "GCJ-02"
        )
        gcj_b_lat, gcj_b_lon = convert_coordinate(
            st.session_state.b_point["lat"], st.session_state.b_point["lon"],
            "WGS-84", "GCJ-02"
        )
        distance = calculate_distance(
            st.session_state.a_point["lat"], st.session_state.a_point["lon"],
            st.session_state.b_point["lat"], st.session_state.b_point["lon"]
        )
        st.info(f"📏 AB点直线距离: **{distance:.0f} 米**")
        st.caption(f"A点(GCJ-02): {gcj_a_lat:.6f}, {gcj_a_lon:.6f}")
        st.caption(f"B点(GCJ-02): {gcj_b_lat:.6f}, {gcj_b_lon:.6f}")

# ========== 右侧3D地图 ==========
with col_right:
    st.markdown("### 🗺️ 3D卫星地图 - 南京科技职业学院")
    st.caption("🛰️ 真实卫星影像 | 绿色:起点A | 红色:终点B | 橙色柱:障碍物 | 青色线:规划航线")
    
    if st.session_state.a_point["set"] and st.session_state.b_point["set"]:
        # 障碍物（南京科技职业学院校园内建筑 - 高德地图GCJ-02坐标）
        obstacles_gcj02 = [
            (32.2330, 118.7495, "📚 图书馆", 25),
            (32.2335, 118.7500, "🏫 八号教学楼", 30),
            (32.2338, 118.7488, "🏛️ 行政楼", 28),
            (32.2328, 118.7480, "🔬 化工实验楼", 22),
            (32.2345, 118.7505, "🛏️ 学生宿舍区", 20),
            (32.2325, 118.7498, "🍽️ 学生食堂", 18),
            (32.2332, 118.7483, "⚡ 配电房", 12),
        ]
        
        # 将障碍物坐标从GCJ-02转换为WGS-84用于地图显示
        obstacles_wgs84 = []
        for lat, lon, name, height in obstacles_gcj02:
            wgs_lat, wgs_lon = convert_coordinate(lat, lon, "GCJ-02", "WGS-84")
            obstacles_wgs84.append((wgs_lat, wgs_lon, name, height))
        
        # 起点终点的显示坐标
        gcj_a_lat, gcj_a_lon = convert_coordinate(
            st.session_state.a_point["lat"], st.session_state.a_point["lon"],
            "WGS-84", "GCJ-02"
        )
        gcj_b_lat, gcj_b_lon = convert_coordinate(
            st.session_state.b_point["lat"], st.session_state.b_point["lon"],
            "WGS-84", "GCJ-02"
        )
        
        # 准备地图数据
        start_data = pd.DataFrame({
            'lat': [st.session_state.a_point["lat"]], 
            'lon': [st.session_state.a_point["lon"]], 
            'name': ['🟢 起点 A (南门)'],
            'gcj_coord': [f"{gcj_a_lat:.6f}, {gcj_a_lon:.6f}"]
        })
        
        end_data = pd.DataFrame({
            'lat': [st.session_state.b_point["lat"]], 
            'lon': [st.session_state.b_point["lon"]], 
            'name': ['🔴 终点 B (宿舍区)'],
            'gcj_coord': [f"{gcj_b_lat:.6f}, {gcj_b_lon:.6f}"]
        })
        
        obstacles_data = pd.DataFrame(obstacles_wgs84, columns=['lat', 'lon', 'name', 'height'])
        
        # 绘制航线点
        num_points = 30
        route_lats = np.linspace(st.session_state.a_point["lat"], st.session_state.b_point["lat"], num_points)
        route_lons = np.linspace(st.session_state.a_point["lon"], st.session_state.b_point["lon"], num_points)
        route_heights = [st.session_state.flight_height] * num_points
        route_data = pd.DataFrame({
            'lat': route_lats,
            'lon': route_lons,
            'height': route_heights
        })
        
        # 起点图层
        start_layer = pdk.Layer(
            "ScatterplotLayer",
            data=start_data,
            get_position=["lon", "lat"],
            get_color=[0, 255, 0, 255],
            get_radius=50,
            pickable=True,
        )
        
        # 终点图层
        end_layer = pdk.Layer(
            "ScatterplotLayer",
            data=end_data,
            get_position=["lon", "lat"],
            get_color=[255, 0, 0, 255],
            get_radius=50,
            pickable=True,
        )
        
        # 障碍物图层（3D柱状）
        obstacle_layer = pdk.Layer(
            "ColumnLayer",
            data=obstacles_data,
            get_position=["lon", "lat"],
            get_elevation="height",
            elevation_scale=5,
            radius=35,
            get_fill_color=[255, 165, 0, 200],
            pickable=True,
        )
        
        # 航线图层
        line_layer = pdk.Layer(
            "LineLayer",
            data=route_data,
            get_source_position=["lon", "lat"],
            get_target_position=["lon", "lat"],
            get_color=[0, 200, 255, 200],
            get_width=4,
        )
        
        # 视图中心
        center_lat = (st.session_state.a_point["lat"] + st.session_state.b_point["lat"]) / 2
        center_lon = (st.session_state.a_point["lon"] + st.session_state.b_point["lon"]) / 2
        
        view_state = pdk.ViewState(
            latitude=center_lat,
            longitude=center_lon,
            zoom=16.5,
            pitch=55,
            bearing=0,
        )
        
        # ========== 🔥 关键修改：使用卫星影像地图 ==========
        deck = pdk.Deck(
            layers=[start_layer, end_layer, obstacle_layer, line_layer],
            initial_view_state=view_state,
            tooltip={"text": "{name}\n坐标(GCJ-02): {gcj_coord}"},
            map_style="mapbox://styles/mapbox/satellite-streets-v12",  # 🛰️ 卫星图+路名
            # 如果想用纯卫星图（无路名），改成下面这行：
            # map_style="mapbox://styles/mapbox/satellite-v9",
        )
        
        st.pydeck_chart(deck, use_container_width=True, height=600)
        
        # 障碍物列表
        with st.expander("📋 南京科技职业学院校园内障碍物列表"):
            st.caption("坐标：GCJ-02（高德地图坐标系）")
            for lat, lon, name, height in obstacles_gcj02:
                st.write(f"- **{name}**: 纬度 {lat:.6f}, 经度 {lon:.6f}, 高度 {height}m")
        
        # 航线信息
        with st.expander("✈️ 航线信息"):
            st.write(f"**起点A (南门) GCJ-02:** {gcj_a_lat:.6f}, {gcj_a_lon:.6f}")
            st.write(f"**终点B (宿舍区) GCJ-02:** {gcj_b_lat:.6f}, {gcj_b_lon:.6f}")
            st.write(f"**直线距离:** {distance:.0f} 米")
            st.write(f"**飞行高度:** {st.session_state.flight_height} 米")
            st.write(f"**障碍物数量:** {len(obstacles_gcj02)} 个")
    else:
        st.warning("⚠️ 请先在左侧设置 A 点和 B 点")

# 页脚
st.markdown("---")
st.caption(f"🕒 最后更新: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | 南京科技职业学院 | 底图: Mapbox 卫星影像 | 坐标转换: GCJ-02 ↔ WGS-84")
