import paho.mqtt.client as mqtt
import json
import time

# MQTT事件回调
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    # 订阅CSI数据主题
    client.subscribe("csi/data")  # 假设ESP32发布到这个主题
    client.subscribe("csi/results")  # 处理结果主题

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        print(f"Topic: {msg.topic}")
        print(f"Message: {json.dumps(payload, indent=2)}")
        
        if msg.topic == "csi/results":
            print(f"Motion detected: {payload.get('motion_detected')}")
            print(f"Breathing rate: {payload.get('breathing_rate')} breaths/min")
    except Exception as e:
        print(f"Error processing message: {e}")
        print(f"Raw payload: {msg.payload}")

# 创建MQTT客户端
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# 连接到本地MQTT代理
client.connect("localhost", 1883, 60)  # 本地代理地址和端口

# 开始接收消息
print("Listening for CSI data...")
client.loop_forever()