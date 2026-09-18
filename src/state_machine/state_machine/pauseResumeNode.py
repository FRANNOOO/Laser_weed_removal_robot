import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool  # swap for actual detection msg type


RESUME_TIMEOUT_SEC = 5.0


class PauseResumeSupervisor(Node):
    def __init__(self):
        super().__init__('pause_resume_node')

        self._paused = False
        self._last_detection_time = None

        self.pause_client = self.create_client(
            Trigger, '/navigate_complete_coverage/pause')
        self.resume_client = self.create_client(
            Trigger, '/navigate_complete_coverage/resume')

        for client, name in [(self.pause_client, 'pause'),
                              (self.resume_client, 'resume')]:
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'Waiting for {name} service...')

        # Detection topic -> triggers pause, resets the resume timer
        self.create_subscription(
            Bool, '/detection_topic', self.detection_cb, 10)

        # Checks every 0.5s whether enough time has passed to resume
        self.create_timer(0.5, self.check_resume_timeout)

    def detection_cb(self, msg: Bool):
        if not msg.data:
            return  # only care about positive detections

        self._last_detection_time = self.get_clock().now()

        if not self._paused:
            self._paused = True
            self.call_service(self.pause_client, 'pause')

    def check_resume_timeout(self):
        if not self._paused or self._last_detection_time is None:
            return

        elapsed = (self.get_clock().now() - self._last_detection_time).nanoseconds / 1e9
        if elapsed >= RESUME_TIMEOUT_SEC:
            self._paused = False
            self.call_service(self.resume_client, 'resume')

    def call_service(self, client, label):
        request = Trigger.Request()
        future = client.call_async(request)
        future.add_done_callback(
            lambda f, label=label: self._on_response(f, label))

    def _on_response(self, future, label):
        try:
            result = future.result()
            self.get_logger().info(
                f'{label} -> success={result.success}, msg={result.message}')
        except Exception as e:
            self.get_logger().error(f'{label} service call failed: {e}')


def main():
    rclpy.init()
    node = PauseResumeSupervisor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()