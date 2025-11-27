### CONTROLADOR FOLLOW IN THE GAP
En este proyecto se a diseñado el controaldor follow in the gap para controalr el vehiculo del simulador F1TENTH , ademas ,de un nodo para contar con el nùmero de vueltas que y el tiempo que ha tomado en realziar cada vuelta 

------------


#### 1. Descripción general del controlador
El FollowTheGapNode es un controlador de navegación autónoma que utiliza tecnología LIDAR, diseñado especialmente para vehículos tipo F1Tenth o robots de carrera que siguen trayectorias mientras evitan obstáculos.
####  Señales de entrada y salida:
- **LaserScan :** Un sensor LIDAR que realiza un escaneo láser, este mensaje incluye:
- - Lista de distancias medidas por el LiDAR,
- - Àngulo de cada rayo
- - Límites mínimo/máximo del láser,

- **AckermannDriveStamped :** Envia mensajes  regulan la velocidad lineal y el ángulo de dirección.
- - **Odómetro (Odometry):**  Para medir la velocidad actual y un indicador de parada (Bool) para detener el vehículo.

------------


#### 2.Características técnicas:
Este controlador contiene algunos aspectos tècnicos que se han considero añadirlos , por lo cual , se presenta a continuaciòn:

|  Característica |  Descripción | Ventaja  |
| ------------ | ------------ | ------------ |
| Disparity Extender + Blur gaussiano circular  | Aplica un filtro gaussiano únicamente a los puntos que estén a más de 0.3 m de distancia, de manera que se conserven los obstáculos pequeños y se suavicen las transiciones.  |   Reduce ruido del LIDAR y evita gaps falsos por puntos aislados
|  Burbuja de seguridad dinámica |  Radio configurable (bubble_radius = 110 índices) alrededor del obstáculo más cercano | Evita colisiones frontales y laterales  |
|  Seguridad frontal adicional | Si hay algo a ±30 rayos del centro que esté a menos de  safety_margin_front (1.2 m), entonces frena  |  Evita colisiones frontales en espacios reducidos. |
|Selección ponderada del punto objetivo   |Combina 70% del punto más lejano + 30% promedio ponderado por d² dentro del gap   |   Mejor seguimiento del centro del hueco grande|
|Control de velocidad adaptativo|   Tres niveles de velocidad base más un factor de curvatura lateral.|  Permite ir muy rápido en recta y reducir drásticamente en curvas cerradas |

------------


### 3.Configuraciòn para la simulacion y lanzamiento de los nodos:
En esta secciòn se detallara el procedimiento para instalar el paquete necesario conjunto con sus dependencias , cambio de pista requerida en el simulador , configuracion de los nodos:

1.Crear una carpeta en donde se encontraràn todos los archivos del proyecto ,en este caso ,estara dicha carpeta en la parte de Documentos:

 	mkdir PROYECTO1
2.Ingrese a la carpeta creada :

	cd PROYECTO1/
3.Crear otra carpeta llamada scr :

	mkdir scr

4.Instalaciòn de las dependencia :Ingresar a la carpeta scr

	cd scr
5.Ejecutar este comando para instalar todas la sdependencias a usarse:

	ros2 pkg create follow_the_gap_node --build-type ament_python --dependencies rclpy sensor_msgs nav_msgs ackermann_msgs numpy scipy

5.Una vez isntaladas las dependencias , ejecutar los siguientes comandos para construir el paquete

	PROYECTO1$ colcon build

Nota:Debe de ingresar a su carpeta del proyecto desde la terminal ,luego ejecutar el comando anterior

6.Ejecutar el siguiente comando:

	source install/setup.bash

 7.Para simular en  F1TENTH con la pista respectiva se procede a cambiar el nombre del mapa en el archivo sim.yaml:

	    map_path: '/home/widinsong/F1Tenth-Repository/src/f1tenth_gym_ros/maps/SaoPaulo_map'

8.Àdemas , se debe toman en cuenta tener los archivos de la pista en formato png y yaml en la carpeta de maps del simulador F1TENTH 

------------


### 4.Parámetros  importantes y sus valores :
Mediante esta tabla se presenta a continuaciòn los parametros que se usan para este controlador considerando que su  velocidad màxima alcanzada es de  10 m/s.
Estos valores son los siguientes:

|  Parámetro | Valor    |  Efecto |
| ------------ | ------------ | ------------ |
|   max_steer_deg| 33.0°  | Límite físico del actuador de dirección   |
|bubble_radius   |  110 índices (~±45-50° típico) | Tamaño de la burbuja de exclusión   |
|   safety_margin_front|  1.2 m | Distancia mínima frontal para frenar |
|  safety_margin_side |  1.4 m | Distancia lateral mínima para reducir velocidad   |
| max_straight_speed  | 10.0 m/s  |   Velocidad máxima en recta despejada |
| min_curve_speed  |  2.0 m/s |Velocidad mínima en curvas cerradas    |
|   curve_slowdown_factor|  0.35 | Factor usado internamente (no directamente)   |
|   disparity_blur_kernel| 9  |  - |  
|   disparity_blur_sigma| 2.0  |controla la fuerza del suavizado|  |

### 5.Explicación del código por secciones-FollowTheGapNode:

#### Còdigo del nodo :

```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
import numpy as np
import math
from scipy.ndimage import gaussian_filter1d

class FollowTheGapNode(Node):
    def __init__(self):
        super().__init__('follow_the_gap_node')

        # Parámetros
        self.declare_parameter('max_steer_deg', 33.0)
        self.declare_parameter('bubble_radius', 110) #110
        self.declare_parameter('safety_margin_front', 1.2)
        self.declare_parameter('safety_margin_side', 1.4)
        self.declare_parameter('curve_slowdown_factor', 0.35)
        
        self.declare_parameter('max_straight_speed', 10.0)
        self.declare_parameter('min_curve_speed', 2.0)
        
        self.declare_parameter('enable_disparity_blur', True)
        self.declare_parameter('disparity_blur_kernel', 9.0)
        self.declare_parameter('disparity_blur_sigma', 2.0)

        self.max_steer = math.radians(self.get_parameter('max_steer_deg').value)
        self.bubble_radius = self.get_parameter('bubble_radius').value
        self.safety_front = self.get_parameter('safety_margin_front').value
        self.safety_side = self.get_parameter('safety_margin_side').value
        self.curve_factor_param = self.get_parameter('curve_slowdown_factor').value
        self.max_straight_speed = self.get_parameter('max_straight_speed').value
        self.min_curve_speed = self.get_parameter('min_curve_speed').value
        self.enable_blur = self.get_parameter('enable_disparity_blur').value
        self.blur_kernel = self.get_parameter('disparity_blur_kernel').value
        self.blur_sigma = self.get_parameter('disparity_blur_sigma').value

        # Suscripciones y publicadores
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.stop_sub = self.create_subscription(Bool, '/stop_flag', self.stop_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.current_velocity = 0.0
        self.angles = None
        self.stop_vehicle = False

    def stop_callback(self, msg):
        self.stop_vehicle = msg.data
        if self.stop_vehicle:
            self.get_logger().info("STOP_FLAG recibido. Vehículo detenido.")

    def odom_callback(self, msg):
        self.current_velocity = msg.twist.twist.linear.x

    # =============================
    # Métodos de procesamiento LIDAR
    # =============================
    def preprocess_lidar(self, ranges):
        proc = np.array(ranges)
        proc[np.isinf(proc)] = 30.0
        return np.clip(proc, 0.01, 30.0)

    def apply_disparity_blur(self, ranges):
        if not self.enable_blur or self.blur_kernel <= 1:
            return ranges
        disparity = np.copy(ranges)
        disparity[disparity < 0.3] = 0.0
        blurred = gaussian_filter1d(disparity, sigma=self.blur_sigma, mode='wrap')
        blurred[disparity < 0.3] = 0.0
        blurred = np.minimum(blurred, disparity)
        return blurred

    def find_max_gap(self, free_space):
        masked = np.ma.masked_where(free_space == 0, free_space)
        slices = np.ma.notmasked_contiguous(masked)
        max_len = 0
        largest = None
        for s in slices:
            size = s.stop - s.start
            if size > max_len:
                max_len = size
                largest = s
        return largest.start, largest.stop if largest else (0, len(free_space))

    def find_best_point(self, start_i, end_i, ranges):
        if self.angles is None:
            return len(ranges) // 2
        d = np.array(ranges[start_i:end_i])
        a = self.angles[start_i:end_i]
        valid = d > 0.5
        if not np.any(valid):
            return len(ranges) // 2
        d = d[valid]
        a = a[valid]
        farthest_i = np.argmax(d)
        best_angle = a[farthest_i]
        weights = d ** 2
        avg_angle = np.average(a, weights=weights)
        final_angle = 0.7 * best_angle + 0.3 * avg_angle
        best_idx = np.argmin(np.abs(self.angles - final_angle))
        return best_idx

    def get_curvature_speed_factor(self, target_angle, ranges):
        n = len(ranges)
        idx_45L = int(n * (135 + 45) / 270)
        idx_90L = int(n * (135 + 90) / 270)
        idx_45R = int(n * (135 - 45) / 270)
        idx_90R = int(n * (135 - 90) / 270)
        d45L = ranges[idx_45L] if 0 <= idx_45L < n else 10.0
        d90L = ranges[idx_90L] if 0 <= idx_90L < n else 10.0
        d45R = ranges[idx_45R] if 0 <= idx_45R < n else 10.0
        d90R = ranges[idx_90R] if 0 <= idx_90R < n else 10.0

        if abs(target_angle) > math.radians(18):
            if target_angle > 0:
                lat = min(d45L, d90L)
            else:
                lat = min(d45R, d90R)
            if lat < self.safety_side:
                return max(0.2, lat / self.safety_side)
            elif lat < self.safety_side * 1.8:
                return 0.55
        return 1.0

    # =============================
    # Callback principal de LIDAR
    # =============================
    def scan_callback(self, scan_msg):
        if self.stop_vehicle:
            self.publish_drive(0.0, 0.0)
            return

        ranges = self.preprocess_lidar(scan_msg.ranges)
        ranges = self.apply_disparity_blur(ranges)

        if self.angles is None:
            n = len(ranges)
            self.angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, n)

        closest_idx = np.argmin(ranges)
        L = max(0, closest_idx - self.bubble_radius)
        R = min(len(ranges)-1, closest_idx + self.bubble_radius)
        ranges[L:R+1] = 0

        front = ranges[len(ranges)//2 - 30 : len(ranges)//2 + 30]
        if len(front) > 0 and np.min(front) < self.safety_front:
            self.publish_drive(0.0, 0.0)
            return

        start, end = self.find_max_gap(ranges)
        best_idx = self.find_best_point(start, end, ranges)
        target_angle = self.angles[best_idx]

        max_gap = np.max(ranges[start:end]) if end > start else 0

        if abs(target_angle) < math.radians(10) and max_gap > 8.0:
            base_speed = self.max_straight_speed
        elif abs(target_angle) < math.radians(20):
            base_speed = 0.4* (self.max_straight_speed + self.min_curve_speed)
        else:
            base_speed = self.min_curve_speed

        factor = self.get_curvature_speed_factor(target_angle, ranges)
        target_speed = base_speed * factor
        target_speed = max(self.min_curve_speed, target_speed)
        steer = np.clip(target_angle, -self.max_steer, self.max_steer)
        self.publish_drive(target_speed, steer)

    # =============================
    # Publicar velocidad y dirección
    # =============================
    def publish_drive(self, speed, steer):
        if self.stop_vehicle:
            speed = 0.0
            steer = 0.0
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)
        self.get_logger().info(
            f"Speed: {speed:5.2f} m/s | Steer: {math.degrees(steer):+6.1f}°",
            throttle_duration_sec=0.2
        )

def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_drive(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```
En este apartado se describe cada segmento del còdigo del controlador que esta conformado ,para una mejor comprenciòn del mismo:
- **Inicialización y parámetros:**

```
self.declare_parameter('max_steer_deg', 33.0)
self.declare_parameter('bubble_radius', 110)
...
self.max_steer = math.radians(self.get_parameter('max_steer_deg').value)
```
- **Suscripciones y publicadores:**

```
self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
self.stop_sub = self.create_subscription(Bool, '/stop_flag', self.stop_callback, 10)
self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

```
- - **LaserScan:**recibe distancias del LIDAR.
- - **Odometry:**Recibe la velocidad actual del vehículo.
- - **Bool:**Flag externo para detener el vehículo de forma segura.
- - **AckermannDriveStamped:**Publica comandos de dirección y velocidad.
- **Callbacks de control básico:**
- - **stop_callback:**Aactualiza un flag que detiene el vehículo si es necesario.
- - **odom_callback:** actualiza la velocidad actual para cálculos de control de velocidad dinámica.

------------


### Explicaciòn de las partes del còdigo del nodo:

- **Procesamiento de LIDAR:**
1. **Preprocesamiento:**

```
def preprocess_lidar(self, ranges):
    proc = np.array(ranges)
    proc[np.isinf(proc)] = 30.0
    return np.clip(proc, 0.01, 30.0)

```
- Convierte los datos LIDAR a un array NumPy.

- Reemplaza valores infinitos por 30 m.

- Limita los rangos para evitar valores muy pequeños o muy grandes.

2.**Suavizado por disparidad:**

```
def apply_disparity_blur(self, ranges):
    ...
```
- Mejora la detección de huecos aplicando un filtro gaussiano a las diferencias de distancia entre puntos.
- Reduce ruido de LIDAR y evita cambios bruscos en el cálculo de huecos.

3.**Eliminación de burbuja alrededor del obstáculo**:
```
closest_idx = np.argmin(ranges)
L = max(0, closest_idx - self.bubble_radius)
R = min(len(ranges)-1, closest_idx + self.bubble_radius)
ranges[L:R+1] = 0
```
- Se identifica el punto más cercano (closest_idx) 
- Se establece un área de seguridad alrededor de él. De esta manera, el vehículo evita dirigirse directamente hacia el obstáculo.

4.**Identificación del mayor hueco libre:**
```
start, end = self.find_max_gap(ranges)
best_idx = self.find_best_point(start, end, ranges)
target_angle = self.angles[best_idx]

```
- **find_max_gap:** Descubre la secuencia continua más larga de distancias que no sean cero.
- **find_best_point:** Elige el punto más adecuado dentro del espacio, teniendo en cuenta la distancia al obstáculo.
- **target_angle:**Àngulo relativo hacia el que el vehículo debe dirigirse.

5.**Cálculo de velocidad según curvatura:**
```
def get_curvature_speed_factor(self, target_angle, ranges):
        n = len(ranges)
        idx_45L = int(n * (135 + 45) / 270)
        idx_90L = int(n * (135 + 90) / 270)
        idx_45R = int(n * (135 - 45) / 270)
        idx_90R = int(n * (135 - 90) / 270)
        d45L = ranges[idx_45L] if 0 <= idx_45L < n else 10.0
        d90L = ranges[idx_90L] if 0 <= idx_90L < n else 10.0
        d45R = ranges[idx_45R] if 0 <= idx_45R < n else 10.0
        d90R = ranges[idx_90R] if 0 <= idx_90R < n else 10.0

        if abs(target_angle) > math.radians(18):
            if target_angle > 0:
                lat = min(d45L, d90L)
            else:
                lat = min(d45R, d90R)
            if lat < self.safety_side:
                return max(0.2, lat / self.safety_side)
            elif lat < self.safety_side * 1.8:
                return 0.55
        return 1.0
```
- Mide la distancia lateral a 45° y 90° en ambos lados.
- Si la curva es pronunciada (target_angle > 18°) y hay poco espacio lateral:
- - Reduce la velocidad de manera proporcional.
- Si hay más espacio lateral:
- - Aplica un factor intermedio de 0.55.
- Si la curva no es pronunciada, mantén la velocidad máxima.
6.**Determinación de velocidad y ángulo:**

```
if abs(target_angle) < math.radians(10) and max_gap > 8.0:
    base_speed = self.max_straight_speed
elif abs(target_angle) < math.radians(20):
    base_speed = 0.4* (self.max_straight_speed + self.min_curve_speed)
else:
    base_speed = self.min_curve_speed
```
- **Curvas suaves / línea recta:** velocidad alta.
- ** Curvas medias:** velocidad intermedia.
- **Curvas cerradas: **Velocidad mínima.
- Se combina con get_curvature_speed_factor para ajustar según riesgo lateral.

7.**Publicación de comandos:**

```
 def publish_drive(self, speed, steer):
        if self.stop_vehicle:
            speed = 0.0
            steer = 0.0
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)
        self.get_logger().info(
            f"Speed: {speed:5.2f} m/s | Steer: {math.degrees(steer):+6.1f}°",
            throttle_duration_sec=0.2
        )
```
- Crea un mensaje ROS2 de tipo AckermannDrive.
- Publica velocidad y ángulo de dirección.
- También imprime un log para depuración.

------------


### 6. Flujo general de ejecución-Nodo FollowTheGaP
**1. **Nodo se inicia y se suscribe a LIDAR, odometría y el flag de stop.
**2.** Cada vez que se recibe un mensaje de LIDAR (scan_callback):
 - Primero, se preprocesan los datos.
 - Si está habilitado, se aplica un suavizado.
 - Luego, se elimina la burbuja alrededor del obstáculo más cercano.
 - Se verifica si hay algún obstáculo cerca del frente.
 - Se identifica el hueco libre más grande y se determina el punto óptimo dentro de él.
 - Se calcula el ángulo de dirección (steer) y la velocidad (speed) en función de la curvatura y el espacio lateral.
 - Se publica el comando de conducción.
 
**3.**El vehículo se ajusta en tiempo real a los cambios en el entorno.

------------


### 7.Descripciòn Nodo LapCounterNode

LapCounterNode es un nodo reactivo que se encarga de contar las vueltas de un vehículo autónomo

- **Rol:** Identificar cuándo el vehículo completa una vuelta en la pista y, si se desea, enviar una señal (/stop_flag) para detener el vehículo al alcanzar un número máximo de vueltas (max_laps).

- **Tipo:** Detector que se basa en la posición (odometría), sin utilizar mapas ni visión, es geo-espacial y se fundamenta en un umbral de distancia desde una posición inicial.

- **Entradas y salidas:**
**1. Entrada:** nav_msgs/Odometry (topic /ego_racecar/odom) — nos da la posición (x,y) del vehículo y la velocidad disponible si es necesario.
**2.Salida: **std_msgs/Bool (topic /stop_flag) — un flag que permite a otros nodos (como el controlador de movimiento) detener el vehículo. 

------------


### 8.Método de detección del nodo LapCounterNode

- Registra la posición inicial la primera vez que recibes un mensaje de odometría. Esa posición se usará como referencia para la "línea de meta".

- En cada mensaje de odometría, calcula la distancia euclidiana entre la posición actual y la posición inicial.

- Define una zona de inicio: un círculo alrededor de la posición inicial con un radio de 3.5 m, que se utiliza como umbral para:

    - Salir de la zona: esto indica que comienza una vuelta (se marca el inicio de la vuelta).

    - Regresar a la zona: se considera que la vuelta ha terminado.

- Se mantiene un estado llamado lap_zone_active para evitar contar múltiples transiciones débiles (debounce lógico):

    - lap_zone_active = True significa que estamos en la zona de inicio (esperando salir para iniciar la vuelta).

    - lap_zone_active = False significa que estamos fuera (esperando regresar para finalizar la vuelta).

- Cuando se termina una vuelta, incrementa lap_count, calcula lap_time usando timestamps de ROS2 (get_clock().now()), acumula total_time y publica stop_flag si lap_count es mayor o igual a max_laps.

------------


### 9.Còdigo del nodo completo:

```
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

```

------------


### 10.Explicación del còdigo:
1.**Cabecera e imports:**

```
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math

```
Importa ROS2 Python (rclpy), los tipos de mensajes y la biblioteca math para realizar cálculos euclidianos.

**2.Clase y constructor:**

```
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

```

- Declara el nodo ROS2 llamado lap_counter_node. 

- El argumento max_laps se utiliza durante la construcción (por defecto es 2, pero en main se cambia a 10)

**3.Suscripción a odometría:**

```
self.subscription = self.create_subscription(
    Odometry,
    '/ego_racecar/odom',
    self.odom_callback,
    10
)

```

- Se suscríbe a /ego_racecar/odom con una profundidad de QoS depth 10 (el valor predeterminado). 
- La función odom_callback gestionará cada mensaje.

**4.Publicador stop_flag:**

```
self.stop_pub = self.create_publisher(Bool, '/stop_flag', 10)

```

Un publicador que avisa al controlador para que detenga el vehículo. Se publica una variable tipo Bool con el dato = True

**5.Variables internas:**

```
self.start_pos = None
self.prev_dist = 999.0
self.lap_count = 0
self.lap_zone_active = True
self.last_lap_time = None
self.total_time = 0.0
self.max_laps = max_laps

```

------------

**6.Varibles del nodo de LapCounterNode**

|   Variablee|Descripciòn  |
| ------------ | ------------ |
|  start_pos |  posición de referencia (x0,y0), se asigna la primera vez que llega un Odometry |
|  prev_dist |El estado anterior (se inicializa en grande). En el código actual, se guarda, pero no se utiliza para una lógica de detección robusta (está listo para filtros o detección de cruce con conciencia de dirección).   |
|  lap_count |  contador de vueltas completadas (inicio y fin). |
|  lap_zone_active |   estado de la zona de inicio|
| last_lap_time  | timestamp del inicio de la vuelta actual  |
| total_time  | Suma acumulada de los tiempos de vuelta realizadas  |
| max_laps  | N_maximo de vueltas permitidas antes de enviar |   |

------------


**7.Mensaje de readiness:**

```
self.get_logger().info("LapCounterNode listo. Esperando primera posición...")

```

Log inicial.

------------


**8.Método utilitario: formateo de tiempo:**

```
def format_time(self, seconds_total):
    minutes = int(seconds_total // 60)
    seconds = seconds_total % 60
    return f"{minutes:02d}:{seconds:05.2f}"

```
Usado para mostrar por la terminal el tiempo en formato de minutos y segundos (MM:SS) el timepo de cada vuelta y el tiempo total 

------------


**9.Callback principal: odom_callback:**

```
def odom_callback(self, msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

```
Obtiene  x,y de la pose.

------------


**10.Registrar posición inicial:**

```
if self.start_pos is None:
    self.start_pos = (x, y)
    self.get_logger().info(f"Posición inicial: x={x:.2f}, y={y:.2f}")
    return

```
- La primera lectura define start_pos. 
- Retorna sin más procesamiento (no inicia ni finaliza vuelta en el primer mensaje).

------------


**11.Cálculo de distancia y zona de inicio:**

```
x0, y0 = self.start_pos
dist = math.sqrt((x - x0)**2 + (y - y0)**2)
is_at_start = dist < 3.5  # tolerancia de zona de inicio

```

------------


**12.Cálculo de distancia y zona de inicio:**

```
x0, y0 = self.start_pos
dist = math.sqrt((x - x0)**2 + (y - y0)**2)
is_at_start = dist < 3.5  # tolerancia de zona de inicio

```

- dist=la distancia euclidiana hasta la línea de meta. 

- El parámetro is_at_start utiliza un umbral de 3.5 metros. Este valor establece el radio de la zona de meta, y puedes ajustarlo según la escala de la pista y la precisión de la odometría.

------------


###11. Lògica de ejecuciòn del nodo:

------------



- Mientras está en la zona inicial no contamos

```
if is_at_start and self.lap_zone_active:
    self.prev_dist = dist
    return
```
Si estamos en la zona y lap_zone_active es True, simplemente actualiza prev_dist y sale: todavía no ha comenzado nueva vuelta.
- Inica la vuelta para salir de la zona inicial

```
if not is_at_start and self.lap_zone_active:
    self.lap_zone_active = False
    self.last_lap_time = self.get_clock().now()
    next_lap = self.lap_count + 1
    self.get_logger().info(...)
    self.prev_dist = dist
    return

```
- Detecta cuando se hace la transición de salida (es decir, cuando el vehiculo deja la zona de inicio). 
- -Cambia lap_zone_active a False y toma last_lap_time como el momento en que comenzó la vuelta. Registra el log y sal

------------


- Regreso a la zona inicial para terminal la vuelta:

```
if is_at_start and not self.lap_zone_active:
    self.lap_count += 1
    now = self.get_clock().now()
    lap_time = (now - self.last_lap_time).nanoseconds / 1e9
    self.total_time += lap_time
    self.last_lap_time = now
    total_time_str = self.format_time(self.total_time)
    self.get_logger().info(...)
    if self.lap_count >= self.max_laps:
        stop_msg = Bool()
        stop_msg.data = True
        self.stop_pub.publish(stop_msg)
        self.get_logger().info("Número máximo de vueltas alcanzado. Deteniendo vehículo...")
    self.lap_zone_active = True

```
Al regresar a la zona cuando lap_zone_active era False, se cuenta una vuelta:
Por lo cual algunas de las variable se comportan de la siguiente manera:
   - lap_count: Incrementa 
   - Calcula lap_time como diferencia entre now y last_lap_time
   - Usa nanoseconds / 1e9 para convertir a segundos.
   - total_time: Suma lso tiempos de cada vuelta terminada
   - Publica log con tiempos de cada vuelta y el tiempo total
   - Si la variable lap_count >= max_laps ,entonces la variable stop_flag = True, esto para mandar a detener al vehiculo
   - La variable lap_zone_active = True se reactiva para inicar un proxima vuelta

------------


- Finalmente actualiza prev_dist

```
self.prev_dist = dist

```
Usado para guardar la distancia previa 

------------


- main:

```
def main(args=None):
    rclpy.init(args=args)
    node = LapCounterNode(max_laps=10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

```
Inicia ROS2, crea el nodo con max_laps=10 y lo ejecuta hasta que termine el nùmero de vueltas asignadas. Al final, destruye el nodo y cierra rclpy.


------------
### 12.Comandos para ejecutar los nodos 

- Ir a la carpeta donde tenga  instalado el  repositorio para ejecutar los siguientes comandos:
Para ejecutar los siguientes comandos debera de estar en la carpeta de su proyecto creado :

	cd F1Tenth-Repository

ejemplo:

	widinsong@widinsongadvay:~$ cd F1Tenth-Repository
	widinsong@widinsongadvay:~/F1Tenth-Repository$ colcon build
	widinsong@widinsongadvay:~/F1Tenth-Repository$ source install/setup.bash
	widinsong@widinsongadvay:~/F1Tenth-Repository$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py

- Dirigirse al directorio de su carpeta del proyecto creado para ejecutar los siguientes comandos:

ejemplo de ejecuciòn:

	widinsong@widinsongadvay:~$ cd Documentos/
	widinsong@widinsongadvay:~/Documentos$ cd PROYECTO1/
	widinsong@widinsongadvay:~/Documentos/PROYECTO1$
	widinsong@widinsongadvay:~/Documentos/PROYECTO1$ colcon build
        Starting >>> f1tenth_follow_gap
        Finished <<< f1tenth_follow_gap [1.09s]          
       Summary: 1 package finished [1.34s]
	widinsong@widinsongadvay:~/Documentos/PROYECTO1$ source install/setup.bash

-Comando para ejecutar el nodo del controlador:

	widinsong@widinsongadvay:~/Documentos/PROYECTO1$ ros2 run f1tenth_follow_gap FollowTheGapNode

-Comando para ejecutar el nodo del contador y cronometro:

	widinsong@widinsongadvay:~/Documentos/PROYECTO1$ ros2 run f1tenth_follow_gap LapCounterNode 

------------

#### Enlace del funcionamiento del controlador conjunto con sus dos nodos:
https://youtu.be/EXUDsh8eUD8

- Imagen de  funcionamiento:

    [![EJECUCION](S "EJECUCION")](https://github.com/WidinsonGadvay/FOLLOW_IN_THE_GAP_CONTROLLER/blob/main/EJECUCI%C3%92N%20DE%20SIMULADOR%20Y%20NODOS.png "EJECUCION")








###End
