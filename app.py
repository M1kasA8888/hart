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
import requests

# ========== 高德卫星地图瓦片配置 ==========
# style=6: 纯卫星图
# style=8: 卫星图 + 道路/地名标注
AMAP_SATELLITE_URL = "https://webst01.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}"
AMAP_SATELLITE_LABEL_URL = "https://webst01.is.autonavi.com/appmaptile?style=8&x={x}&y={y}&z={z}"

# ========== 坐标系转换（WGS-84 ↔ GCJ-02）==========
def wgs84_to_gcj02(lat, lon):
    """WGS-84 转 GCJ-02（高德坐标系）"""
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
    """GCJ-02 转 WGS-84"""
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
    """坐标转换主函数"""
    if from_crs == to_crs:
        return lat, lon
    if from_crs == "WGS-84" and to_crs == "GCJ-02":
        return wgs84_to_gcj02(lat, lon)
    if from_crs == "GCJ-02" and to_crs == "WGS-84":
        return gcj02_to_wgs84(lat, lon)
    return lat, lon


def calculate_distance(lat1, lon1, lat2, lon2):
    """计算两点间距离（米）"""
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c


# ========== 高德地图瓦片图层 ==========
def create_amap_tile_layer():
    """创建高德卫星地图瓦片图层"""
    return {
        "@@type": "TileLayer",
        "data": AMAP_SATELLITE_URL,
        "tileSize": 256,
        "maxZoom": 19,
        "minZoom": 1,
        "opacity": 1.0
    }


# ========== 页面配置 ==========
st.set_page_config(page_title="无人机智能化应用系统", page_icon="🚁", layout="wide")

# 初始化 session state
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
if 'current_page' not in st.session_state:
    st.session_state.current_page = "航线规划"

# ========== 侧边栏 ==========
with st.sidebar:
    st.header("📋 导航")
    
    # 功能页面选择
    page = st.radio(
        "功能页面",
        ["航线规划", "飞行监控"],
        index=0 if st.session_state.current_page == "航线规划" else 1
    )
    st.session_state.current_page = page
    
    st.markdown("---")
    
    st.header("⚙️ 坐标系统设置")
    coord_system = st.radio(
        "输入坐标",
        ["GCJ-02 (高德/腾讯)", "WGS-84 (GPS)"],
        index=0 if st.session_state.coord_system == "GCJ-02" else 1
    )
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
st.title("🗺️ 无人机智能化应用系统")
st.caption("高德卫星地图 | 支持多边形圈选障碍物 | 配置持久化保存")

st.markdown("---")

# ========== 左右两列布局 ==========
col_left, col_right = st.columns([1, 1.5], gap="large")

# ========== 左侧控制面板 ==========
with col_left:
    if st.session_state.current_page == "航线规划":
        st.markdown("### 🎮 航线规划")
        
        # 起点 A
        st.markdown("#### 📍 起点A")
        col_a1, col_a2 = st.columns(2)
        with col_a1:
            a_lat = st.number_input("纬度", value=32.2322, format="%.6f", key="a_lat")
        with col_a2:
            a_lon = st.number_input("经度", value=118.7490, format="%.6f", key="a_lon")
        
        if st.button("✅ 设置A点", key="set_a", use_container_width=True):
            if st.session_state.coord_system == "GCJ-02":
                # 输入的是GCJ-02，需要转成WGS-84存储
                wgs_lat, wgs_lon = convert_coordinate(a_lat, a_lon, "GCJ-02", "WGS-84")
            else:
                wgs_lat, wgs_lon = a_lat, a_lon
            st.session_state.a_point = {"lat": wgs_lat, "lon": wgs_lon, "set": True}
            st.success(f"✅ A点已设置")
            st.rerun()
        
        st.markdown("---")
        
        # 终点 B
        st.markdown("#### 📍 终点B")
        col_b1, col_b2 = st.columns(2)
        with col_b1:
            b_lat = st.number_input("纬度", value=32.2345, format="%.6f", key="b_lat")
        with col_b2:
            b_lon = st.number_input("经度", value=118.7505, format="%.6f", key="b_lon")
        
        if st.button("✅ 设置B点", key="set_b", use_container_width=True):
            if st.session_state.coord_system == "GCJ-02":
                wgs_lat, wgs_lon = convert_coordinate(b_lat, b_lon, "GCJ-02", "WGS-84")
            else:
                wgs_lat, wgs_lon = b_lat, b_lon
            st.session_state.b_point = {"lat": wgs_lat, "lon": wgs_lon, "set": True}
            st.success(f"✅ B点已设置")
            st.rerun()
        
        st.markdown("---")
        
        # 飞行参数
        st.markdown("#### ✈️ 飞行参数")
        st.session_state.flight_height = st.slider("设定飞行高度(m)", 10, 200, 50)
        
        # 显示距离
        if st.session_state.a_point["set"] and st.session_state.b_point["set"]:
            distance = calculate_distance(
                st.session_state.a_point["lat"], st.session_state.a_point["lon"],
                st.session_state.b_point["lat"], st.session_state.b_point["lon"]
            )
            st.info(f"📏 AB点距离: **{distance:.0f} 米**")
        
        st.markdown("---")
        
        # 障碍物管理
        st.markdown("#### 🧱 障碍物配置")
        st.info("💡 提示：障碍物配置自动保存，下次打开自动加载")
        
        # 显示障碍物数量
        obstacle_count = st.session_state.obstacle_manager.get_count()
        st.caption(f"📊 当前障碍物数量: **{obstacle_count}** 个")
        
        col_btn1, col_btn2 = st.columns(2)
        with col_btn1:
            if st.button("💾 保存配置", use_container_width=True):
                if st.session_state.obstacle_manager.save():
                    st.success("已保存")
                else:
                    st.error("保存失败")
        with col_btn2:
            if st.button("🗑️ 清除全部", use_container_width=True):
                st.session_state.obstacle_manager.clear_all()
                st.success("已清除")
                st.rerun()
        
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
        
        # 显示保存时间
        save_time = st.session_state.obstacle_manager.get_save_time()
        st.caption(f"📅 上次保存: {save_time}")
    
    else:  # 飞行监控页面
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
        
        # 状态显示
        col1, col2, col3 = st.columns(3)
        
        if st.session_state.running:
            latest = st.session_state.simulator.get_latest_heartbeat()
            history = st.session_state.simulator.get_history()
            
            if st.session_state.simulator.offline:
                st.error("🚨 **警报！无人机已掉线超过3秒！**")
            
            with col1:
                if latest and latest.get('status') == 'alive':
                    st.metric("📡 无人机状态", "🟢 在线")
                else:
                    st.metric("📡 无人机状态", "🔴 掉线")
            with col2:
                st.metric("⏱️ 最后心跳", latest.get('time_str', '--') if latest else "--")
            with col3:
                heartbeat_count = len([h for h in history if h.get('status') == 'alive'])
                st.metric("💗 累计心跳", f"{heartbeat_count} 次")
            
            # 心跳趋势图
            if history and len(history) > 0:
                df_data = []
                for h in history[-50:]:  # 只显示最近50条
                    df_data.append({
                        '序号': h['heartbeat_id'],
                        '时间': h['time_str'],
                        '呼吸时间(秒)': h['breath_time'] if h.get('status') == 'alive' else None,
                    })
                df = pd.DataFrame(df_data)
                
                fig = go.Figure()
                online_df = df[df['呼吸时间(秒)'].notna()]
                if len(online_df) > 0:
                    fig.add_trace(go.Scatter(
                        x=online_df['序号'],
                        y=online_df['呼吸时间(秒)'],
                        mode='lines+markers',
                        name='心跳信号',
                        line=dict(color='green', width=2)
                    ))
                fig.update_layout(
                    title="心跳信号实时监控",
                    xaxis_title="心跳序号",
                    yaxis_title="呼吸时间 (秒)",
                    height=300,
                    template='plotly_white'
                )
                st.plotly_chart(fig, use_container_width=True)
        else:
            st.info("👈 点击「启动心跳监测」开始监控")
        
        # 自动刷新
        if st.session_state.running:
            time.sleep(0.5)
            st.rerun()

# ========== 右侧地图 ==========
with col_right:
    if st.session_state.current_page == "航线规划":
        st.markdown("### 🗺️ 高德卫星地图 - 可绘制多边形圈选障碍物")
        
        if st.session_state.a_point["set"] and st.session_state.b_point["set"]:
            # 获取障碍物
            obstacles = st.session_state.obstacle_manager.get_obstacles_for_map()
            
            # 获取显示坐标（GCJ-02用于显示）
            display_a_lat, display_a_lon = convert_coordinate(
                st.session_state.a_point["lat"], st.session_state.a_point["lon"],
                "WGS-84", "GCJ-02"
            )
            display_b_lat, display_b_lon = convert_coordinate(
                st.session_state.b_point["lat"], st.session_state.b_point["lon"],
                "WGS-84", "GCJ-02"
            )
            
            # 准备地图数据（地图显示用WGS-84）
            start_data = pd.DataFrame({
                'lat': [st.session_state.a_point["lat"]],
                'lon': [st.session_state.a_point["lon"]],
                'name': ['🟢 起点 A'],
                'display_coord': [f"{display_a_lat:.6f}, {display_a_lon:.6f}"]
            })
            
            end_data = pd.DataFrame({
                'lat': [st.session_state.b_point["lat"]],
                'lon': [st.session_state.b_point["lon"]],
                'name': ['🔴 终点 B'],
                'display_coord': [f"{display_b_lat:.6f}, {display_b_lon:.6f}"]
            })
            
            obstacles_data = pd.DataFrame(obstacles) if obstacles else pd.DataFrame()
            
            # 航线点
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
            if not obstacles_data.empty:
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
                )
            else:
                obstacle_layer = None
            
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
            
            # 组装图层
            layers = [create_amap_tile_layer(), start_layer, end_layer, line_layer]
            if obstacle_layer:
                layers.append(obstacle_layer)
            
            # 创建地图
            deck = pdk.Deck(
                layers=layers,
                initial_view_state=view_state,
                tooltip={"text": "{name}"},
                map_provider=None,
                map_style=None,
            )
            
            st.pydeck_chart(deck, use_container_width=True, height=550)
            
            # 多边形圈选说明
            st.markdown("---")
            st.markdown("### ✏️ 多边形圈选障碍物使用说明")
            st.caption("由于 pydeck 限制，目前通过手动输入坐标的方式来添加障碍物多边形")
            
            # 手动添加障碍物
            st.markdown("#### 🆕 添加障碍物")
            col_p1, col_p2, col_p3 = st.columns(3)
            with col_p1:
                obs_lat = st.number_input("障碍物纬度", value=32.2330, format="%.6f", key="obs_lat")
            with col_p2:
                obs_lon = st.number_input("障碍物经度", value=118.7495, format="%.6f", key="obs_lon")
            with col_p3:
                obs_name = st.text_input("障碍物名称", value="新障碍物", key="obs_name")
            
            if st.button("➕ 添加障碍物", use_container_width=True):
                # 转换坐标
                if st.session_state.coord_system == "GCJ-02":
                    wgs_lat, wgs_lon = convert_coordinate(obs_lat, obs_lon, "GCJ-02", "WGS-84")
                else:
                    wgs_lat, wgs_lon = obs_lat, obs_lon
                st.session_state.obstacle_manager.add_obstacle(
                    [[wgs_lat, wgs_lon]],
                    name=obs_name
                )
                st.success(f"已添加障碍物: {obs_name}")
                st.rerun()
            
            # 显示现有障碍物列表
            with st.expander("📋 当前障碍物列表"):
                if obstacles:
                    for obs in obstacles:
                        col_del1, col_del2 = st.columns([3, 1])
                        with col_del1:
                            # 显示GCJ-02坐标
                            gcj_lat, gcj_lon = convert_coordinate(obs['lat'], obs['lon'], "WGS-84", "GCJ-02")
                            st.write(f"**{obs['name']}** - 坐标: {gcj_lat:.6f}, {gcj_lon:.6f}")
                        with col_del2:
                            if st.button(f"🗑️", key=f"del_{obs['id']}"):
                                st.session_state.obstacle_manager.remove_obstacle(obs['id'])
                                st.rerun()
                else:
                    st.info("暂无障碍物，请在上方添加")
            
        else:
            st.warning("⚠️ 请先在左侧设置 A 点和 B 点")

# ========== 页脚 ==========
st.markdown("---")
st.caption(f"🕒 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} | 南京科技职业学院 | 坐标系: {st.session_state.coord_system} | 障碍物配置持久化 v12.2 | 地图: 高德卫星影像")
