# heartbeat_monitor.py
import time
import pandas as pd
import streamlit as st
from datetime import datetime

# 初始化数据
if "heartbeats" not in st.session_state:
    st.session_state.heartbeats = []
    st.session_state.last_time = time.time()
    st.session_state.running = False

st.title("🚁 无人机心跳监测可视化")

# 侧边栏控制
with st.sidebar:
    st.header("控制面板")
    if st.button("▶️ 开始模拟心跳"):
        st.session_state.running = True
    
    if st.button("⏹️ 停止模拟"):
        st.session_state.running = False
    
    if st.button("🗑️ 清空数据"):
        st.session_state.heartbeats = []
        st.session_state.last_time = time.time()
        st.session_state.running = False

# 生成心跳包
def generate_heartbeat():
    seq = len(st.session_state.heartbeats) + 1
    now = datetime.now()
    st.session_state.heartbeats.append({
        "序号": seq,
        "时间": now,
        "延迟(秒)": round(time.time() - st.session_state.last_time, 3)
    })
    st.session_state.last_time = time.time()

# 自动生成心跳（每秒一个）
if st.session_state.running:
    current_time = time.time()
    if current_time - st.session_state.last_time >= 1:
        generate_heartbeat()
        st.rerun()

# 掉线检测
if len(st.session_state.heartbeats) > 0:
    last_beat_time = st.session_state.heartbeats[-1]["时间"].timestamp()
    current_time = time.time()
    seconds_since_last = current_time - last_beat_time
    
    if seconds_since_last > 3:
        st.error(f"⚠️ 无人机掉线！已 {seconds_since_last:.1f} 秒未收到心跳包！")
    else:
        st.success(f"✅ 在线中 | 最后心跳: {seconds_since_last:.1f} 秒前")

# 显示最新心跳信息
if st.session_state.heartbeats:
    latest = st.session_state.heartbeats[-1]
    st.info(f"📡 最新心跳 | 序号: {latest['序号']} | 时间: {latest['时间'].strftime('%H:%M:%S')} | 间隔: {latest['延迟(秒)']}秒")

# 可视化折线图
df = pd.DataFrame(st.session_state.heartbeats)
if not df.empty:
    st.subheader("📊 心跳序号变化趋势")
    st.line_chart(df.set_index("时间")["序号"], use_container_width=True)
    
    # 显示数据表格
    with st.expander("查看详细数据"):
        st.dataframe(df)
else:
    st.info("点击「开始模拟心跳」启动监测")
