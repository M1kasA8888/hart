import streamlit as st
import pandas as pd
import plotly.graph_objects as go
import numpy as np
import pydeck as pdk
from heartbeat_sim import DroneHeartbeatSimulator
from obstacle_manager import ObstacleManager
import time
from datetime import datetime
import math
import json

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
if 'obstacle_manager' not in st.session_state:
    st.session_state.obstacle_manager = ObstacleManager()
if 'polygon_points' not in st.session_state:
    st.session_state.polygon_points = []  # 临时存储圈选的点

# ========== 页面标题 ==========
st.title("🗺️ 无人机智能化应用系统 - 南京科技职业学院")
st.markdown("---")

# ========== 左右两列布局 ==========
col_left, col_right = st.columns([1, 2], gap="large")

# ========== 左侧控制面板 ==========
with col_left:
    st.markdown("### 🎮 控制面板")
    
    # 导航
    st.markdown("#### 📋 功能页面")
    page = st.radio("", ["航线规划", "飞行监控"], horizontal=True)
    
    st.markdown("---")
    
    # 坐标系设置
    st.markdown("#### ⚙️ 坐标系设置")
    coord_system = st.radio(
        "输入坐标系",
        ["WGS-84", "GCJ-02 (高德/百度)"],
        index=1 if st.session_state.coord_system == "GCJ-02" else 0
    )
    st.session_state.coord_system = "GCJ-02" if "GCJ-02" in coord_system else "WGS-84"
    
    st.markdown("---")
    
    # 系统状态
    st.markdown("#### 📊 系统状态")
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
    
    # 起点A
    st.markdown("#### 📍 起点A")
    st.caption("输入坐标：GCJ-02")
    col_a1, col_a2 = st.columns(2)
    with col_a1:
        a_lat = st.number_input("纬度", value=32.2323, format="%.6f", key="a_lat")
    with col_a2:
        a_lon = st.number_input("经度", value=118.7490, format="%.6f", key="a_lon")
    
    if st.button("📍 设置A点", key="set_a", use_container_width=True):
        wgs_lat, wgs_lon = convert_coordinate(a_lat, a_lon, "GCJ-02", "WGS-84")
        st.session_state.a_point = {"lat": wgs_lat, "lon": wgs_lon, "set": True}
        st.success(f"✅ A点已设置 (GCJ-02: {a_lat:.6f}, {a_lon:.6f})")
        st.rerun()
    
    st.markdown("---")
    
    # 终点B
    st.markdown("#### 📍 终点B")
    st.caption("输入坐标：GCJ-02")
    col_b1, col_b2 = st.columns(2)
    with col_b1:
        b_lat = st.number_input("纬度", value=32.2344, format="%.6f", key="b_lat")
    with col_b2:
        b_lon = st.number_input("经度", value=118.7490, format="%.6f", key="b_lon")
    
    if st.button("📍 设置B点", key="set_b", use_container_width=True):
        wgs_lat, wgs_lon = convert_coordinate(b_lat, b_lon, "GCJ-02", "WGS-84")
        st.session_state.b_point = {"lat": wgs_lat, "lon": wgs_lon, "set": True}
        st.success(f"✅ B点已设置 (GCJ-02: {b_lat:.6f}, {b_lon:.6f})")
        st.rerun()
    
    st.markdown("---")
    
    # 飞行参数
    st.markdown("#### ✈️ 飞行参数")
    st.session_state.flight_height = st.slider("设定飞行高度(m)", 10, 200, 50)
    
    st.markdown("---")
    
    # 障碍物管理
    st.markdown("#### 🧱 障碍物配置")
    
    # 多边形圈选说明
    st.info("💡 在地图上点击鼠标左键圈选障碍物区域，双击完成圈选")
    
    col_btn1, col_btn2 = st.columns(2)
    with col_btn1:
        if st.button("💾 保存到文件", use_container_width=True):
            if st.session_state.obstacle_manager.save():
                st.success(f"已保存 {st.session_state.obstacle_manager.get_count()} 个障碍物")
            else:
                st.error("保存失败")
    
    with col_btn2:
        if st.button("📂 从文件加载", use_container_width=True):
            if st.session_state.obstacle_manager.load():
                st.success(f"已加载 {st.session_state.obstacle_manager.get_count()} 个障碍物")
                st.rerun()
            else:
                st.warning("无配置文件或加载失败")
    
    col_btn3, col_btn4 = st.columns(2)
    with col_btn3:
        if st.button("🗑️ 清除全部", use_container_width=True):
            st.session_state.obstacle_manager.clear_all()
            st.success("已清除所有障碍物")
            st.rerun()
    
    with col_btn4:
        # 下载配置文件
        config_json = json.dumps({
            "version": "v12.2",
            "save_time": datetime.now().isoformat(),
            "obstacles": st.session_state.obstacle_manager.obstacles
        }, ensure_ascii=False, indent=2)
        st.download_button(
            label="📥 下载配置文件",
            data=config_json,
            file_name="obstacle_config.json",
            mime="application/json",
            use_container_width=True
        )
    
    # 显示障碍物数量
    obstacle_count = st.session_state.obstacle_manager.get_count()
    save_time = st.session_state.obstacle_manager.get_save_time()
    st.caption(f"📊 共 {obstacle_count} 个障碍物 | 保存时间: {save_time}")

# ========== 右侧3D地图 ==========
with col_right:
    if page == "航线规划":
        st.markdown("### 🗺️ 3D卫星地图 - 多边形圈选障碍物")
        st.caption("🛰️ OpenStreetMap 卫星影像 | 🟢 起点A | 🔴 终点B | 🟠 障碍物 | 🔵 航线")
        
        if st.session_state.a_point["set"] and st.session_state.b_point["set"]:
            # 获取障碍物
            obstacles = st.session_state.obstacle_manager.get_obstacles_for_map()
            
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
                'name': ['🟢 起点 A'],
                'coord': [f"{gcj_a_lat:.6f}, {gcj_a_lon:.6f}"]
            })
            
            end_data = pd.DataFrame({
                'lat': [st.session_state.b_point["lat"]], 
                'lon': [st.session_state.b_point["lon"]], 
                'name': ['🔴 终点 B'],
                'coord': [f"{gcj_b_lat:.6f}, {gcj_b_lon:.6f}"]
            })
            
            obstacles_data = pd.DataFrame(obstacles) if obstacles else pd.DataFrame()
            
            # 绘制航线点
            num_points = 30
            route_lats = np.linspace(st.session_state.a_point["lat"], st.session_state.b_point["lat"], num_points)
            route_lons = np.linspace(st.session_state.a_point["lon"], st.session_state.b_point["lon"], num_points)
            route_data = pd.DataFrame({'lat': route_lats, 'lon': route_lons})
            
            # 起点图层
            start_layer = pdk.Layer(
                "ScatterplotLayer",
                data=start_data,
                get_position=["lon", "lat"],
                get_color=[0, 255, 0, 255],
                get_radius=40,
                pickable=True,
            )
            
            # 终点图层
            end_layer = pdk.Layer(
                "ScatterplotLayer",
                data=end_data,
                get_position=["lon", "lat"],
                get_color=[255, 0, 0, 255],
                get_radius=40,
                pickable=True,
            )
            
            # 障碍物图层（圆柱体）
            obstacle_layer = pdk.Layer(
                "ColumnLayer",
                data=obstacles_data,
                get_position=["lon", "lat"],
                get_elevation="height",
                elevation_scale=5,
                radius=25,
                disk_resolution=12,
                get_fill_color=[255, 165, 0, 200],
                pickable=True,
            ) if not obstacles_data.empty else None
            
            # 航线图层
            line_layer = pdk.Layer(
                "LineLayer",
                data=route_data,
                get_source_position=["lon", "lat"],
                get_target_position=["lon", "lat"],
                get_color=[0, 150, 255, 200],
                get_width=4,
            )
            
            # 视图中心
            center_lat = (st.session_state.a_point["lat"] + st.session_state.b_point["lat"]) / 2
            center_lon = (st.session_state.a_point["lon"] + st.session_state.b_point["lon"]) / 2
            
            view_state = pdk.ViewState(
                latitude=center_lat,
                longitude=center_lon,
                zoom=16,
                pitch=50,
                bearing=0,
            )
            
            # 组装图层（只添加非None的图层）
            layers = [start_layer, end_layer, line_layer]
            if obstacle_layer:
                layers.append(obstacle_layer)
            
            # ========== 使用 OpenStreetMap 卫星图 ==========
            deck = pdk.Deck(
                layers=layers,
                initial_view_state=view_state,
                tooltip={"text": "{name}\n坐标: {coord}"},
                map_style="mapbox://styles/mapbox/satellite-streets-v12",  # 卫星影像
            )
            
            st.pydeck_chart(deck, use_container_width=True, height=550)
            
            # 多边形圈选说明
            st.markdown("---")
            st.markdown("### ✏️ 多边形圈选障碍物")
            st.caption("在地图上点击鼠标左键添加顶点，双击完成圈选")
            
            # 显示现有障碍物列表
            with st.expander("📋 当前障碍物列表"):
                if obstacles:
                    for obs in obstacles:
                        col_del1, col_del2 = st.columns([3, 1])
                        with col_del1:
                            st.write(f"**{obs['name']}** - 中心: {obs['lat']:.6f}, {obs['lon']:.6f}")
                        with col_del2:
                            if st.button(f"🗑️ 删除", key=f"del_{obs['id']}"):
                                st.session_state.obstacle_manager.remove_obstacle(obs['id'])
                                st.rerun()
                else:
                    st.info("暂无障碍物，请在地图上圈选添加")
            
            # 距离信息
            distance = calculate_distance(
                st.session_state.a_point["lat"], st.session_state.a_point["lon"],
                st.session_state.b_point["lat"], st.session_state.b_point["lon"]
            )
            st.info(f"📏 AB点直线距离: **{distance:.0f} 米** | 飞行高度: **{st.session_state.flight_height} 米** | 障碍物数量: **{len(obstacles)}**")
            
        else:
            st.warning("⚠️ 请先在左侧设置 A 点和 B 点")
    
    else:  # 飞行监控页面
        st.markdown("### 📡 飞行监控 - 心跳监测")
        
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
        else:
            st.info("👈 请点击「启动心跳监测」开始监控")
        
        if st.session_state.running:
            time.sleep(0.5)
            st.rerun()

# 页脚
st.markdown("---")
st.caption(f"🕒 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | 南京科技职业学院 | 坐标系: {st.session_state.coord_system} | 障碍物配置持久化 v12.2")
