import json
import os
from datetime import datetime

CONFIG_FILE = "obstacle_config.json"

class ObstacleManager:
    """障碍物管理器 - 支持持久化存储"""
    
    def __init__(self):
        self.obstacles = []
        self.load()
    
    def add_obstacle(self, polygon_coords, name="障碍物", height=30):
        """添加多边形障碍物"""
        obstacle = {
            "id": len(self.obstacles) + 1,
            "name": name,
            "type": "polygon",
            "coordinates": polygon_coords,
            "created_at": datetime.now().isoformat(),
            "height": height
        }
        self.obstacles.append(obstacle)
        self.save()
        return obstacle
    
    def remove_obstacle(self, obstacle_id):
        """删除障碍物"""
        self.obstacles = [o for o in self.obstacles if o["id"] != obstacle_id]
        self.save()
    
    def clear_all(self):
        """清除所有障碍物"""
        self.obstacles = []
        self.save()
    
    def save(self):
        """保存到文件"""
        try:
            with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
                json.dump({
                    "version": "v12.2",
                    "save_time": datetime.now().isoformat(),
                    "obstacles": self.obstacles
                }, f, ensure_ascii=False, indent=2)
            return True
        except Exception as e:
            print(f"保存失败: {e}")
            return False
    
    def load(self):
        """从文件加载"""
        try:
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    self.obstacles = data.get("obstacles", [])
                return True
        except Exception as e:
            print(f"加载失败: {e}")
        return False
    
    def get_obstacles_for_map(self):
        """获取用于地图显示的障碍物数据"""
        map_obstacles = []
        for obs in self.obstacles:
            if obs["type"] == "polygon":
                coords = obs["coordinates"]
                if coords and len(coords) > 0:
                    center_lat = sum(p[0] for p in coords) / len(coords)
                    center_lon = sum(p[1] for p in coords) / len(coords)
                    map_obstacles.append({
                        "id": obs["id"],
                        "name": obs["name"],
                        "lat": center_lat,
                        "lon": center_lon,
                        "height": obs.get("height", 30)
                    })
        return map_obstacles
    
    def get_count(self):
        return len(self.obstacles)
    
    def get_save_time(self):
        try:
            if os.path.exists(CONFIG_FILE):
                with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    return data.get("save_time", "未知")
        except:
            pass
        return "未保存"
