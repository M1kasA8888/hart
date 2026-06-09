import time
import threading
import queue
from datetime import datetime
import random

class DroneHeartbeatSimulator:
    def __init__(self):
        self.running = False
        self.heartbeat_queue = queue.Queue()
        self.last_heartbeat_time = None
        self.offline = False
        self.heartbeat_history = []
        
    def start(self):
        self.running = True
        self.offline = False
        self.heartbeat_history = []
        self.send_thread = threading.Thread(target=self._send_heartbeat)
        self.send_thread.daemon = True
        self.send_thread.start()
        self.detect_thread = threading.Thread(target=self._detect_offline)
        self.detect_thread.daemon = True
        self.detect_thread.start()
        
    def _send_heartbeat(self):
        while self.running:
            breath_time = round(random.uniform(0.8, 1.2), 2)
            heartbeat = {
                'timestamp': datetime.now(),
                'time_str': datetime.now().strftime("%H:%M:%S"),
                'breath_time': breath_time,
                'heartbeat_id': len(self.heartbeat_history) + 1,
                'status': 'alive'
            }
            self.heartbeat_queue.put(heartbeat)
            self.last_heartbeat_time = time.time()
            self.heartbeat_history.append(heartbeat)
            if len(self.heartbeat_history) > 100:
                self.heartbeat_history.pop(0)
            time.sleep(1)
            
    def _detect_offline(self):
        while self.running:
            if self.last_heartbeat_time is not None:
                elapsed = time.time() - self.last_heartbeat_time
                if elapsed > 3 and not self.offline:
                    self.offline = True
                    offline_mark = {
                        'timestamp': datetime.now(),
                        'time_str': datetime.now().strftime("%H:%M:%S"),
                        'breath_time': 0,
                        'heartbeat_id': len(self.heartbeat_history) + 1,
                        'status': 'offline',
                        'offline_after': round(elapsed, 1)
                    }
                    self.heartbeat_queue.put(offline_mark)
                    self.heartbeat_history.append(offline_mark)
            time.sleep(0.5)
            
    def stop(self):
        self.running = False
        
    def get_latest_heartbeat(self):
        try:
            return self.heartbeat_queue.get_nowait()
        except queue.Empty:
            return None
            
    def get_history(self):
        return self.heartbeat_history.copy()
