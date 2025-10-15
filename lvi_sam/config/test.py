import yaml

with open("c:\Honda_Intern\LVI_SAM_forked\lvi_sam\config\params_camera.yaml") as f:
    config = yaml.safe_load(f)

assert "/**" in config, "ROS 2 param YAML must start with /**:"
assert "ros__parameters" in config["/**"], "Missing ros__parameters block"
