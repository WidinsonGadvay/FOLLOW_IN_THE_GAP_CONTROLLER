#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math

class LapCounterNode(Node):
    def __init__(self, max_laps=2):
        super().__init__('lap_counter_node')

        # Suscripción a odometría
        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        # Publicador para detener vehículo
        self.stop_pub = self.create_publisher(Bool, '/stop_flag', 10)

        # Variables internas
        self.start_pos = None
        self.prev_dist = 999.0
        self.lap_count = 0
        self.lap_zone_active = True
        self.last_lap_time = None
        self.total_time = 0.0
        self.max_laps = max_laps

        self.get_logger().info("LapCounterNode listo. Esperando primera posición...")

    # Formatear tiempo total en MM:SS
    def format_time(self, seconds_total):
        minutes = int(seconds_total // 60)
        seconds = seconds_total % 60
        return f"{minutes:02d}:{seconds:05.2f}"

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Registrar posición inicial
        if self.start_pos is None:
            self.start_pos = (x, y)
            self.get_logger().info(f"Posición inicial: x={x:.2f}, y={y:.2f}")
            return

        x0, y0 = self.start_pos
        dist = math.sqrt((x - x0)**2 + (y - y0)**2)
        is_at_start = dist < 3.5  # tolerancia de zona de inicio

        # 1) Mientras está en la zona inicial no contamos
        if is_at_start and self.lap_zone_active:
            self.prev_dist = dist
            return

        # 2) Sale de la zona inicial → iniciar vuelta
        if not is_at_start and self.lap_zone_active:
            self.lap_zone_active = False
            self.last_lap_time = self.get_clock().now()
            next_lap = self.lap_count + 1
            self.get_logger().info(
                f"\n----------------------------\n"
                f"  Iniciando vuelta {next_lap}\n"
                f"----------------------------"
            )
            self.prev_dist = dist
            return

        # 3) Regresa a la zona inicial → terminar vuelta
        if is_at_start and not self.lap_zone_active:
            self.lap_count += 1
            now = self.get_clock().now()
            lap_time = (now - self.last_lap_time).nanoseconds / 1e9
            self.total_time += lap_time
            self.last_lap_time = now
            total_time_str = self.format_time(self.total_time)

            self.get_logger().info(
                f"\n=====================================\n"
                f"             LAP {self.lap_count}\n"
                f"     Tiempo vuelta : {lap_time:.2f} s\n"
                f"     Tiempo total  : {total_time_str} (MM:SS)\n"
                f"====================================="
            )

            # Publicar stop_flag si alcanza máximo de vueltas
            if self.lap_count >= self.max_laps:
                stop_msg = Bool()
                stop_msg.data = True
                self.stop_pub.publish(stop_msg)
                self.get_logger().info("Número máximo de vueltas alcanzado. Deteniendo vehículo...")

            self.lap_zone_active = True

        self.prev_dist = dist


def main(args=None):
    rclpy.init(args=args)
    node = LapCounterNode(max_laps=10)  # Cambia 3 por el número de vueltas deseado
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

