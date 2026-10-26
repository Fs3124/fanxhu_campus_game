#!/usr/bin/env python

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from ros_tcp_endpoint import TcpServer


def main(args=None):
    rclpy.init(args=args)
    tcp_server = TcpServer("UnityEndpoint")
    executor = MultiThreadedExecutor(num_threads=2)
    # TCP registration must find the executor even on the first connection.
    tcp_server.executor = executor
    executor.add_node(tcp_server)
    try:
        tcp_server.start()
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        tcp_server.destroy_nodes()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
