import rclpy
from rclpy.node import Node
from fastapi import FastAPI
from fastapi_bridge.routes.nodes import router as nodes_router
from fastapi_bridge.routes.logs import router as logs_router
from fastapi_bridge.routes.launch import router as launch_router
from fastapi_bridge.routes.config import router as config_router
import uvicorn
import threading

app = FastAPI(title="ROS2 Node Description API")

class ROS2BridgeNode(Node):
    def __init__(self):
        super().__init__('ros2_bridge')
        self.declare_parameter('ws_root', '/home/malekpdas/ros2-ws')
        self.declare_parameter('api_host', '0.0.0.0')
        self.declare_parameter('api_port', 8000)
        self.declare_parameter('log_file_path', '/home/malekpdas/.ros/log/latest/launch.log')
        self.declare_parameter('launch_dir', '/home/malekpdas/ros2-ws/src/launch')

        self.ws_root = self.get_parameter('ws_root').get_parameter_value().string_value
        self.api_host = self.get_parameter('api_host').get_parameter_value().string_value
        self.api_port = self.get_parameter('api_port').get_parameter_value().integer_value
        self.log_file_path = self.get_parameter('log_file_path').get_parameter_value().string_value
        self.launch_dir = self.get_parameter('launch_dir').get_parameter_value().string_value

        app.state.ws_root = self.ws_root
        app.state.log_file_path = self.log_file_path
        app.state.launch_dir = self.launch_dir
        app.state.is_shutting_down = False

        # Add event handlers for graceful shutdown
        @app.on_event("startup")
        async def startup_event():
            app.state.is_shutting_down = False

        @app.on_event("shutdown")
        async def shutdown_event():
            app.state.is_shutting_down = True

        self.register_routes()

    def register_routes(self):
        app.include_router(nodes_router)
        app.include_router(logs_router)
        app.include_router(launch_router)
        app.include_router(config_router)

    def initialize_app(self):
        uvicorn.run(app, host=self.api_host, port=self.api_port, log_level="info")

    def destroy(self):
        app.should_exit = True
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
