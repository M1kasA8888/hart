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
    st.session_state.a_point = {"lat": 32.2057, "lon": 118.7178, "set": False}
if 'b_point' not in st.session_state:
    st.session_state.b_point = {"lat": 32.2100, "lon": 118.7250, "set": False}
if 'coord_system' not in st.session_state:
    st.session_state.coord_system = "WGS-84"
if 'simulator' not in st.session_state:
    st.session_state.simulator = DroneHeartbeatSimulator()
    st.session_state.running = False

# ========== 侧边栏 ==========
with st.sidebar:
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
    
    st.markdown("---")
    st.header("⚙️ 坐标系设置")
    coord_system = st.radio(
        "输入坐标系",
        ["WGS-84", "GCJ-02"],
        index=0 if st.session_state.coord_system == "WGS-84" else 1
    )
    st.session_state.coord_system = coord_system
    st.caption("💡 地图显示使用 WGS-84")

# ========== 主页面 Tab ==========
tab1, tab2 = st.tabs(["🗺️ 航线规划", "📡 飞行监控"])

# ========== Tab1: 航线规划 ==========
with tab1:
    st.subheader("🗺️ 航线规划 - 3D地图")
    col_left, col_right = st.columns([1, 2])
    with col_left:
        st.markdown("### 📍 坐标设置")
        st.markdown("**起点 A**")
        col_a1, col_a2 = st.columns(2)
        with col_a1:
            a_lat_input = st.number_input("纬度", value=32.2057, format="%.6f", key="a_lat")
        with col_a2:
            a_lon_input = st.number_input("经度", value=118.7178, format="%.6f", key="a_lon")
        if st.button("📍 设置A点"):
            display_lat, display_lon = convert_coordinate(a_lat_input, a_lon_input, st.session_state.coord_system, "WGS-84")
            st.session_state.a_point = {"lat": display_lat, "lon": display_lon, "set": True}
            st.success(f"✅ A点已设置 ({display_lat:.6f}, {display_lon:.6f})")
            st.rerun()
        
        st.markdown("---")
        st.markdown("**终点 B**")
        col_b1, col_b2 = st.columns(2)
        with col_b1:
            b_lat_input = st.number_input("纬度", value=32.2100, format="%.6f", key="b_lat")
        with col_b2:
            b_lon_input = st.number_input("经度", value=118.7250, format="%.6f", key="b_lon")
        if st.button("📍 设置B点"):
            display_lat, display_lon = convert_coordinate(b_lat_input, b_lon_input, st.session_state.coord_system, "WGS-84")
            st.session_state.b_point = {"lat": display_lat, "lon": display_lon, "set": True}
            st.success(f"✅ B点已设置 ({display_lat:.6f}, {display_lon:.6f})")
            st.rerun()
        
        st.markdown("---")
        st.markdown("### ✈️ 飞行参数")
        flight_height = st.slider("飞行高度 (m)", 20, 200, 50)
        
        if st.session_state.a_point["set"] and st.session_state.b_point["set"]:
            distance = calculate_distance(st.session_state.a_point["lat"], st.session_state.a_point["lon"], st.session_state.b_point["lat"], st.session_state.b_point["lon"])
            st.info(f"📏 AB点距离: {distance:.0f} 米")
    
    with col_right:
        if st.session_state.a_point["set"] and st.session_state.b_point["set"]:
            obstacles = [
                (32.2070, 118.7195, "🏢 图书馆", 30),
                (32.2082, 118.7210, "📚 教学楼", 35),
                (32.2078, 118.7230, "🏛️ 行政楼", 28),
                (32.2090, 118.7185, "🗼 水塔", 40),
            ]
            start_data = pd.DataFrame({'lat': [st.session_state.a_point["lat"]], 'lon': [st.session_state.a_point["lon"]], 'name': ['🟢 起点 A']})
            end_data = pd.DataFrame({'lat': [st.session_state.b_point["lat"]], 'lon': [st.session_state.b_point["lon"]], 'name': ['🔴 终点 B']})
            obstacles_data = pd.DataFrame(obstacles, columns=['lat', 'lon', 'name', 'height'])
            
            start_layer = pdk.Layer("ScatterplotLayer", data=start_data, get_position=["lon", "lat"], get_color=[0,255,0,255], get_radius=40)
            end_layer = pdk.Layer("ScatterplotLayer", data=end_data, get_position=["lon", "lat"], get_color=[255,0,0,255], get_radius=40)
            obstacle_layer = pdk.Layer("ColumnLayer", data=obstacles_data, get_position=["lon", "lat"], get_elevation="height", elevation_scale=5, radius=50, get_fill_color=[255,165,0,200])
            
            view_state = pdk.ViewState(latitude=(st.session_state.a_point["lat"]+st.session_state.b_point["lat"])/2, longitude=(st.session_state.a_point["lon"]+st.session_state.b_point["lon"])/2, zoom=15, pitch=50)
            deck = pdk.Deck(layers=[start_layer, end_layer, obstacle_layer], initial_view_state=view_state, tooltip={"text": "{name}"}, map_style="mapbox://styles/mapbox/satellite-streets-v12")
            st.pydeck_chart(deck, use_container_width=True)
            
            with st.expander("📋 障碍物列表"):
                for obs in obstacles:
                    st.write(f"- {obs[2]}: {obs[0]:.6f}, {obs[1]:.6f}")
        else:
            st.warning("⚠️ 请先设置 A 点和 B 点")

# ========== Tab2: 飞行监控 ==========
with tab2:
    st.subheader("📡 飞行监控 - 心跳监测")
    
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
    
    with col_btn2:
        if st.session_state.a_point["set"] and st.session_state.b_point["set"]:
            st.success(f"📍 A: {st.session_state.a_point['lat']:.4f}")
            st.info(f"📍 B: {st.session_state.b_point['lat']:.4f}")
    
    st.markdown("---")
    col1, col2, col3 = st.columns(3)
    
    if st.session_state.running:
        latest = st.session_state.simulator.get_latest_heartbeat()
        history = st.session_state.simulator.get_history()
        
        if st.session_state.simulator.offline:
            st.error("🚨 **警报！无人机已掉线超过3秒！**")
        
        with col1:
            if latest and latest.get('status') == 'alive':
                st.metric("📡 无人机状态", "🟢 在线飞行中")
            else:
                st.metric("📡 无人机状态", "🔴 已掉线")
        with col2:
            if latest:
                st.metric("⏱️ 最后心跳", latest.get('time_str', '--'))
            else:
                st.metric("⏱️ 最后心跳", "--")
        with col3:
            heartbeat_count = len([h for h in history if h.get('status') == 'alive'])
            st.metric("💗 累计心跳", f"{heartbeat_count} 次")
        
        if history and len(history) > 0:
            df_data = []
            for h in history:
                df_data.append({
                    '序号': h['heartbeat_id'],
                    '时间': h['time_str'],
                    '呼吸时间(秒)': h['breath_time'] if h.get('status') == 'alive' else None,
                    '状态': '在线' if h.get('status') == 'alive' else '掉线'
                })
            df = pd.DataFrame(df_data)
            
            fig = go.Figure()
            online_df = df[df['呼吸时间(秒)'].notna()]
            if len(online_df) > 0:
                fig.add_trace(go.Scatter(
                    x=online_df['序号'],
                    y=online_df['呼吸时间(秒)'],
                    mode='lines+markers',
                    name='🟢 心跳信号',
                    line=dict(color='green', width=2)
                ))
            fig.update_layout(
                title="心跳信号实时监控",
                xaxis_title="心跳序号",
                yaxis_title="呼吸时间 (秒)",
                height=400,
                template='plotly_white'
            )
            st.plotly_chart(fig, use_container_width=True)
            
            with st.expander("📋 最近心跳记录"):
                st.dataframe(df.tail(10), use_container_width=True)
    else:
        st.info("👈 请点击「启动心跳监测」开始监控")
    
    if st.session_state.running:
        time.sleep(0.5)
        st.rerun()

st.markdown("---")
st.caption(f"🕒 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | 坐标系: {st.session_state.coord_system} | v3.0")
