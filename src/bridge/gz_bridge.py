import os
import yaml

def main():
    config_path = os.path.join(os.path.dirname(__file__), 'bridge_config.yml')
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
        
    topic_args = []
    for entry in config.get('topics', []):
        name = entry['topic_name']
        ros_type = entry['ros_type_name']
        gz_type = entry['gz_type_name']
        topic_args.append(f"{name}@{ros_type}[{gz_type}")
        
    bridge_args = " ".join(topic_args)
    cmd = f"bash -c 'source ~/ardu_ws/install/setup.bash && ros2 run ros_gz_bridge parameter_bridge {bridge_args}'"
    
    print(f"[*] Command: {cmd}")
    
    try:
        os.system(cmd)
    except KeyboardInterrupt:
        print("\n[*] Bridge stopped.")

if __name__ == '__main__':
    main()
