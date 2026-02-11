import rclpy
from rclpy.node import Node
from fastapi import FastAPI
from fastapi_bridge.routes.nodes import router as nodes_router
from fastapi_bridge.routes.logs import router as logs_router
from fastapi_bridge.routes.launch import router as launch_router
from fastapi_bridge.routes.config import router as config_router
import uvicorn
import threading
import os

app = FastAPI(title="ROS2 Node Description API")

class ROS2BridgeNode(Node):
    def __init__(self):
        super().__init__('ros2_bridge')

        self.declare_all_parameters()
        self.load_all_parameters()

        app.state.ws_root = os.path.expanduser(self.ws_root)
        app.state.log_file_path = os.path.expanduser(self.log_file_path)
        app.state.launch_dir = self.launch_dir.replace("[ROS2-WS]", app.state.ws_root)
        app.state.rec_dir = self.rec_dir.replace("[ROS2-WS]", app.state.ws_root)

        self.register_routes()

    def declare_all_parameters(self):
        self.declare_parameter('ws_root.value', '')
        self.declare_parameter('api_host.value', '0.0.0.0')
        self.declare_parameter('api_port.value', 8000)
        self.declare_parameter('log_file_path.value', '')
        self.declare_parameter('launch_dir.value', '')
        self.declare_parameter('rec_dir.value', '')

    def load_all_parameters(self):
        self.ws_root = self.get_parameter('ws_root.value').value
        self.api_host = self.get_parameter('api_host.value').value
        self.api_port = self.get_parameter('api_port.value').value
        self.log_file_path = self.get_parameter('log_file_path.value').value
        self.launch_dir = self.get_parameter('launch_dir.value').value
        self.rec_dir = self.get_parameter('rec_dir.value').value

    def register_routes(self):
        app.include_router(nodes_router)
        app.include_router(logs_router)
        app.include_router(launch_router)
        app.include_router(config_router)

    def initialize_app(self):
        uvicorn.run(app, host=self.api_host, port=self.api_port, log_level="info")

    def destroy(self):
        app.routes.clear()
        self.destroy_node()

def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)

    ros_node = ROS2BridgeNode()
    
    executor_thread = threading.Thread(target=lambda: rclpy.spin(ros_node), daemon=True)
    executor_thread.start()

    print(f"Starting FastAPI server at http://{ros_node.api_host}:{ros_node.api_port}")
    try:
        ros_node.initialize_app()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
