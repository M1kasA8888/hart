import streamlit as st
import pandas as pd
import plotly.graph_objects as go
import numpy as np
import pydeck as pdk
from heartbeat_sim import DroneHeartbeatSimulator
import time
from datetime import datetime
import math

# ========== 坐标系转换 ==========
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
    st.session_state.a_point = {"lat": 32.2320, "lon": 118.7480, "set": True}
if 'b_point' not in st.session_state:
    st.session_state.b_point = {"lat": 32.2355, "lon": 118.7490, "set": True}
if 'coord_system' not in st.session_state:
    st.session_state.coord_system = "GCJ-02"
if 'simulator' not in st.session_state:
    st.session_state.simulator = DroneHeartbeatSimulator()
    st.session_state.running = False
if 'flight_height' not in st.session_state:
    st.session_state.flight_height = 50
if 'current_page' not in st.session_state:
    st.session_state.current_page = "航线规划"

# 简单障碍物列表（不依赖 obstacle_manager）
if 'obstacles' not in st.session_state:
    st.session_state.obstacles = [
        {"id": 1, "name": "🏢 博业楼", "lat": 32.2335, "lon": 118.7485, "height": 25},
        {"id": 2, "name": "📚 图书馆", "lat": 32.2338, "lon": 118.7492, "height": 28},
        {"id": 3, "name": "🏫 教学楼", "lat": 32.2342, "lon": 118.7498, "height": 30},
    ]

# ========== 侧边栏 ==========
with st.sidebar:
    st.header("📋 导航")
    page = st.radio("功能页面", ["航线规划", "飞行监控"], index=0)
    st.session_state.current_page = page
    
    st.markdown("---")
    st.header("⚙️ 坐标系统设置")
    coord_system = st.radio("输入坐标", ["GCJ-02 (高德/腾讯)", "WGS-84 (GPS)"], index=0)
    st.session_state.coord_system = "GCJ-02" if "GCJ-02" in coord_system else "WGS-84"
    
    st.markdown("---")
    st.header("📊 系统状态")
    col_status1, col_status2 = st.columns(2)
    with col_status1:
        if st.session_state.a_point["set"]:
            st.success("✅ A点已设")
        else:
            st.info("⚪ A点未设")
    with col_status2:
        if st.session_state.b_point["set"]:
            st.success("✅ B点已设")
        else:
            st.info("⚪ B点未设")

# ========== 主页面 ==========
st.title("🗺️ 无人机智能化应用系统 - 南京科技职业学院")
st.caption("📍 起点A: 操场 | 终点B: 一食堂 | 🛰️ 卫星影像 | 障碍物: 博业楼、图书馆、教学楼")

st.markdown("---")

# ========== 左右两列布局 ==========
col_left, col_right = st.columns([1, 1.5], gap="large")

# ========== 左侧控制面板 ==========
with col_left:
    if page == "航线规划":
        st.markdown("### 🎮 航线规划")
        
        st.markdown("#### 📍 起点A (操场)")
        col_a1, col_a2 = st.columns(2)
        with col_a1:
            a_lat = st.number_input("纬度", value=32.2320, format="%.6f", key="a_lat")
        with col_a2:
            a_lon = st.number_input("经度", value=118.7480, format="%.6f", key="a_lon")
        
        if st.button("✅ 设置A点", key="set_a", use_container_width=True):
            if st.session_state.coord_system == "GCJ-02":
                wgs_lat, wgs_lon = convert_coordinate(a_lat, a_lon, "GCJ-02", "WGS-84")
            else:
                wgs_lat, wgs_lon = a_lat, a_lon
            st.session_state.a_point = {"lat": wgs_lat, "lon": wgs_lon, "set": True}
            st.success("✅ A点已设置")
            st.rerun()
        
        st.markdown("---")
        
        st.markdown("#### 📍 终点B (一食堂)")
        col_b1, col_b2 = st.columns(2)
        with col_b1:
            b_lat = st.number_input("纬度", value=32.2355, format="%.6f", key="b_lat")
        with col_b2:
            b_lon = st.number_input("经度", value=118.7490, format="%.6f", key="b_lon")
        
        if st.button("✅ 设置B点", key="set_b", use_container_width=True):
            if st.session_state.coord_system == "GCJ-02":
                wgs_lat, wgs_lon = convert_coordinate(b_lat, b_lon, "GCJ-02", "WGS-84")
            else:
                wgs_lat, wgs_lon = b_lat, b_lon
            st.session_state.b_point = {"lat": wgs_lat, "lon": wgs_lon, "set": True}
            st.success("✅ B点已设置")
            st.rerun()
        
        st.markdown("---")
        
        st.markdown("#### ✈️ 飞行参数")
        st.session_state.flight_height = st.slider("设定飞行高度(m)", 10, 200, 50)
        
        if st.session_state.a_point["set"] and st.session_state.b_point["set"]:
            distance = calculate_distance(
                st.session_state.a_point["lat"], st.session_state.a_point["lon"],
                st.session_state.b_point["lat"], st.session_state.b_point["lon"]
            )
            st.info(f"📏 AB点距离: **{distance:.0f} 米**")
        
        st.markdown("---")
        
        st.markdown("#### 🧱 障碍物列表")
        for obs in st.session_state.obstacles:
            col_d1, col_d2 = st.columns([3, 1])
            with col_d1:
                st.write(f"**{obs['name']}** - {obs['lat']:.6f}, {obs['lon']:.6f}")
            with col_d2:
                if st.button(f"🗑️", key=f"del_{obs['id']}"):
                    st.session_state.obstacles = [o for o in st.session_state.obstacles if o['id'] != obs['id']]
                    st.rerun()
        
        # 添加障碍物
        st.markdown("#### ➕ 添加障碍物")
        col_obs1, col_obs2 = st.columns(2)
        with col_obs1:
            new_lat = st.number_input("纬度", value=32.2335, format="%.6f", key="new_lat")
        with col_obs2:
            new_lon = st.number_input("经度", value=118.7490, format="%.6f", key="new_lon")
        new_name = st.text_input("名称", value="新建筑", key="new_name")
        
        if st.button("➕ 添加", use_container_width=True):
            new_id = max([o['id'] for o in st.session_state.obstacles]) + 1 if st.session_state.obstacles else 1
            st.session_state.obstacles.append({
                "id": new_id, "name": new_name, 
                "lat": new_lat, "lon": new_lon, "height": 30
            })
            st.success(f"已添加: {new_name}")
            st.rerun()
    
    else:  # 飞行监控
        st.markdown("### 📡 飞行监控")
        
        col_btn1, col_btn2 = st.columns(2)
        with col_btn1:
            if not st.session_state.running:
                if st.button("▶️ 启动心跳监测", use_container_width=True):
                    st.session_state.simulator.start()
                    st.session_state.running = True
                    st.rerun()
            else:
                if st.button("⏹️ 停止心跳监测", use_container_width=True):
                    st.session_state.simulator.stop()
                    st.session_state.running = False
                    st.rerun()
        
        st.markdown("---")
        col1, col2, col3 = st.columns(3)
        
        if st.session_state.running:
            latest = st.session_state.simulator.get_latest_heartbeat()
            history = st.session_state.simulator.get_history()
            
            if st.session_state.simulator.offline:
                st.error("🚨 警报！无人机已掉线超过3秒！")
            
            with col1:
                if latest and latest.get('status') == 'alive':
                    st.metric("📡 无人机状态", "🟢 在线")
                else:
                    st.metric("📡 无人机状态", "🔴 掉线")
            with col2:
                st.metric("⏱️ 最后心跳", latest.get('time_str', '--') if latest else "--")
            with col3:
                heartbeat_count = len([h for h in history if h.get('status') == 'alive'])
                st.metric("💗 累计心跳", f"{heartbeat_count}")
            
            if history:
                df_data = []
                for h in history[-50:]:
                    df_data.append({
                        '序号': h['heartbeat_id'],
                        '时间': h['time_str'],
                        '呼吸时间': h['breath_time'] if h.get('status') == 'alive' else None,
                    })
                df = pd.DataFrame(df_data)
                fig = go.Figure()
                online_df = df[df['呼吸时间'].notna()]
                if len(online_df) > 0:
                    fig.add_trace(go.Scatter(
                        x=online_df['序号'], y=online_df['呼吸时间'],
                        mode='lines+markers', name='心跳信号',
                        line=dict(color='green', width=2)
                    ))
                fig.update_layout(title="心跳信号实时监控", xaxis_title="序号", yaxis_title="呼吸时间(秒)", height=350)
                st.plotly_chart(fig, use_container_width=True)
        else:
            st.info("👈 点击「启动心跳监测」开始监控")
        
        if st.session_state.running:
            time.sleep(0.5)
            st.rerun()

# ========== 右侧地图 ==========
with col_right:
    if page == "航线规划":
        st.markdown("### 🗺️ 卫星地图 - 南京科技职业学院全景")
        
        if st.session_state.a_point["set"] and st.session_state.b_point["set"]:
            # 转换显示坐标
            show_a_lat, show_a_lon = convert_coordinate(
                st.session_state.a_point["lat"], st.session_state.a_point["lon"],
                "WGS-84", "GCJ-02"
            )
            show_b_lat, show_b_lon = convert_coordinate(
                st.session_state.b_point["lat"], st.session_state.b_point["lon"],
                "WGS-84", "GCJ-02"
            )
            
            # 地图数据
            start_data = pd.DataFrame({
                'lat': [st.session_state.a_point["lat"]], 'lon': [st.session_state.a_point["lon"]],
                'name': ['🟢 起点A (操场)']
            })
            end_data = pd.DataFrame({
                'lat': [st.session_state.b_point["lat"]], 'lon': [st.session_state.b_point["lon"]],
                'name': ['🔴 终点B (一食堂)']
            })
            
            # 障碍物数据
            obstacles_data = pd.DataFrame(st.session_state.obstacles) if st.session_state.obstacles else pd.DataFrame()
            
            # 航线点
            num_points = 30
            route_lats = np.linspace(st.session_state.a_point["lat"], st.session_state.b_point["lat"], num_points)
            route_lons = np.linspace(st.session_state.a_point["lon"], st.session_state.b_point["lon"], num_points)
            route_data = pd.DataFrame({'lat': route_lats, 'lon': route_lons})
            
            # 起点图层
            start_layer = pdk.Layer(
                "ScatterplotLayer", data=start_data,
                get_position=["lon", "lat"], get_color=[0, 255, 0, 255], get_radius=40
            )
            # 终点图层
            end_layer = pdk.Layer(
                "ScatterplotLayer", data=end_data,
                get_position=["lon", "lat"], get_color=[255, 0, 0, 255], get_radius=40
            )
            # 障碍物图层
            if not obstacles_data.empty:
                obstacle_layer = pdk.Layer(
                    "ColumnLayer", data=obstacles_data,
                    get_position=["lon", "lat"], get_elevation="height",
                    elevation_scale=5, radius=25, disk_resolution=12,
                    get_fill_color=[255, 165, 0, 200]
                )
            else:
                obstacle_layer = None
            # 航线图层
            line_layer = pdk.Layer(
                "LineLayer", data=route_data,
                get_source_position=["lon", "lat"], get_target_position=["lon", "lat"],
                get_color=[0, 150, 255, 200], get_width=4
            )
            
            # 视图中心
            center_lat = (st.session_state.a_point["lat"] + st.session_state.b_point["lat"]) / 2
            center_lon = (st.session_state.a_point["lon"] + st.session_state.b_point["lon"]) / 2
            view_state = pdk.ViewState(latitude=center_lat, longitude=center_lon, zoom=16.5, pitch=50)
            
            # 组装图层
            layers = [start_layer, end_layer, line_layer]
            if obstacle_layer:
                layers.append(obstacle_layer)
            
            # 卫星地图
            deck = pdk.Deck(
                layers=layers,
                initial_view_state=view_state,
                tooltip={"text": "{name}"},
                map_style="mapbox://styles/mapbox/satellite-streets-v12",
            )
            
            st.pydeck_chart(deck, use_container_width=True, height=550)
            
            st.info(f"""
            📍 **校园位置说明**：
            - **起点A (操场)**：{show_a_lat:.6f}, {show_a_lon:.6f}
            - **终点B (一食堂)**：{show_b_lat:.6f}, {show_b_lon:.6f}
            - **AB点距离**: {distance:.0f} 米
            - **飞行高度**: {st.session_state.flight_height} 米
            """)
            
        else:
            st.warning("⚠️ 请先在左侧设置 A 点和 B 点")

# 页脚
st.markdown("---")
st.caption(f"🕒 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | 南京科技职业学院 | 地图: Mapbox 卫星影像")
