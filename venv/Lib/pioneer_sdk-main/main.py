import math
import traceback
import time
from datetime import datetime
from threading import Lock, Thread

from pioneer_sdk import Pioneer

USE_SIMULATOR = False

CONFIG = {
    "drones": {
        "Drone-0": {"ip": "127.0.0.1", "port": 8000, "start": 0},
        "Drone-1": {"ip": "127.0.0.1", "port": 8001, "start": 1},
        "Drone-2": {"ip": "127.0.0.1", "port": 8002, "start": 2},
        "Drone-3": {"ip": "127.0.0.1", "port": 8003, "start": 3}
    },
    "polygon": {
        "size": (10.5, 10.5),
        "start-points": [
            (-2.0, -4.0),
            (-1.0, -4.0),
            (1.0, -4.0),
            (2.0, -4.0)
        ]
    },
    "area": {
        "start": (1, 1.5),
        "end": (9.5, 9.5),
        "line-step": 0.3,
        "fire-min-temp": 30,
        "fire-radius": 1.1,
        "scan-height": 1,
        "max-overhead": 0.2  # расстояние, которое можно без проблем наложить
    },
}

# генерация зон сканирования
try:
    start = CONFIG["area"]["start"]
    end = CONFIG["area"]["end"]
    area_width = (end[1] - start[1]) / 4
    CONFIG["areas"] = [
        {
            "start": (start[0], start[1] + area_width * i),
            "size": (end[0] - start[0], area_width)
        } for i in range(0, 4)
    ]
except Exception as e:
    raise e

# ================ ДЛЯ РАБОТЫ С ДРОНАМИ ================


PIONEER_LOGGING = False
PIONEER_METHOD = 2

# точки представляют собой 3d точку - x (red, width), y (green, height), z (blue, depth)
# это класс-адаптер для таких точек
class PointAdapter:
    def __init__(self, x: float, y: float, z: float):
        self._x = round(x, 1)
        self._y = round(y, 1)
        self._z = round(z, 1)

    @staticmethod
    def from_2d(point: tuple | list, height: float):
        assert len(point) == 2
        return PointAdapter(point[0], height, point[1])

    @classmethod
    def array_from_2d(cls, points: list, height: float):
        res = []
        for p in points:
            res.append(PointAdapter.from_2d(p, height))
        return res

    @staticmethod
    def from_3d(point: tuple | list):
        # в списке будет X Y Z
        # чтобы соответсвовало моим кординатам нужно поставить X Z Y
        return PointAdapter(point[0], point[2], point[1])

    def __str__(self):
        return f"Point (x={self._x}, y={self._y}, z={self._z})"

    def x(self):
        return self._x

    def y(self):
        return self._z  # в симуляторе, как и в жизни оси слегка попутаны, проще всего работать с высотой как Y

    def z(self):
        return self._y

# это класс базового функционала дрона (он же раб, не надо удивляться названию (а этот комп вообще-то master))
class BasicDrone(Thread):
    def __init__(self, pioneer: Pioneer = None, ip: str = "127.0.0.1", port: int = 8000,
                 drone_name: str = "drone", enable_log: bool = True):
        Thread.__init__(self)
        self._drone_name = drone_name
        self._enable_log = enable_log
        if pioneer is not None:
            self._drone = pioneer
        else:
            if ip is None or port is None:
                raise Exception("ip or port not defined")
            self._drone = Pioneer(logger=PIONEER_LOGGING, method=PIONEER_METHOD, pioneer_ip=ip,
                                  pioneer_mavlink_port=port)

    __log_lock = Lock()

    def _log(self, message):
        if self._enable_log:
            with BasicSlave.__log_lock:
                print(f"{datetime.now().strftime('%H:%M:%S.%f')[:-3]} {self._drone_name}: {message}")

    def do_waypoint_fly(self, points: list):
        """
        пролететь по точкам
        Note: считается что дрон уже взлетел
        :param points: <code>list</code> of <code>Point</code>
        """
        for point in points:
            # направляемся в новую точку
            self._log(f"go to {point}")
            self._drone.go_to_local_point(x=point.x(), y=point.y(), z=point.z(), yaw=0)

            # теперь надо ждать пока дрон не прилетит
            start_time = time.time()
            self._log("wait for point reached event...")

            while True:
                res = self._drone.point_reached()
                if res is not None:
                    if res:
                        break  # прилетел

            end_time = time.time()
            self._log(f"point reached on {end_time - start_time} sec")

class ScannerDrone(BasicDrone):
    _lock = Lock()
    _fire_points = []

    @staticmethod
    def _is_new_fire(point: tuple | list):
        assert len(point) == 2
        with ScannerDrone._lock:
            for fire in ScannerDrone._fire_points:
                vec_len = math.sqrt((point[0] - fire[0]) ** 2 + (point[1] - fire[1]) ** 2)
                if vec_len <= 1:
                    return False
            return True

    @staticmethod
    def _add_fire_point(point: tuple | list):
        assert len(point) == 2
        with ScannerDrone._lock:
            ScannerDrone._fire_points.append(point)

    @staticmethod
    def get_fire_points():
        with ScannerDrone._lock:
            return ScannerDrone._fire_points.copy()

    def __init__(self, name, config):
        super().__init__(drone_name=name, ip=config["ip"], port=config["port"])
        self._config = config
        area = CONFIG["area"]
        self._height = area["scan-height"]
        self._line_step = area["line-step"]
        self._min_fire_temp = area["fire-min-temp"]
        self._home_point = CONFIG["polygon"]["start-points"][config["start"]]

    def _fire_scan_area(self, point: list | tuple):
        # и так, квадрат (малевича)

        # y   это
        # y   оси)
        # 0xxxxxx

        offsets = [
            (0.50, -0.50), (-0.50, -0.50),
            (-0.50, -0.25), (0.50, -0.25),
            (0.50, 0.00), (-0.50, 0.00),
            (-0.50, 0.25), (0.50, 0.25),
            (0.50, 0.50), (-0.50, 0.50)
        ]
        fire_points = []
        have_piro_data = False
        is_fire = False
        piro_data = None
        for offset in offsets:
            target = PointAdapter.from_2d((point[0] + offset[0], point[1] + offset[1]), self._height)
            self._log(f"go to {point} with scan")
            self._drone.go_to_local_point(x=target.x(), y=target.y(), z=target.z(), yaw=0)
            while True:
                if not have_piro_data:
                    tmp = self._drone.get_piro_sensor_data(blocking=False)
                    if tmp is not None:
                        if tmp > self._min_fire_temp:
                            have_piro_data = True
                            piro_data = tmp
                            if not is_fire:
                                is_fire = True

                if have_piro_data:
                    # точку надо получить блокирующим методом, чтоб уж наверняка
                    tmp = self._drone.get_local_position_lps(blocking=True)
                    if tmp is not None:
                        have_piro_data = False
                        point_2d = (tmp[0], tmp[1])
                        self._log(f"(sub area scan) point fire: piro={piro_data}, point={point_2d}")
                        fire_points.append(point_2d)

                tmp = self._drone.point_reached(blocking=False)
                if tmp is not None:
                    if tmp:
                        break  # прилетел

        return fire_points

    def _fire_point_received(self, point: list | tuple):
        # теперь проверим, не принадлежит этот набор точек обнаруженному пожару
        if ScannerDrone._is_new_fire(point):
            # ага, принадлежит, значит сканируем дальше
            # а пока просто тормозим
            # self.do_waypoint_fly([PointAdapter.from_2d(point, self._height)])

            # выполним сканирование ближайшей зоны
            points = self._fire_scan_area(point)
            # дальше просто сделаем усреднение координат, центр скорее всего будет где-то близко
            x, y = 0, 0
            for p in points:
                x += p[0]
                y += p[1]
            x, y = x / len(points), y / len(points)

            # записываем пожар
            ScannerDrone._add_fire_point(point)

            # выполняем полет к центру пожара
            self.do_waypoint_fly([PointAdapter.from_2d((x, y), self._height)])

            # и выполняем что говорят
            self._on_fire(point)

            # возвращаем дрон в зад, то есть в место где он себя нашел, когда летел и сканировал
            self.do_waypoint_fly([PointAdapter.from_2d(point, self._height)])

    def _fly_with_scan(self, target_point: tuple | list):
        # направляемся в новую точку
        point = PointAdapter.from_2d(target_point, self._height)
        self._log(f"go to {point} with scan")
        self._drone.go_to_local_point(x=point.x(), y=point.y(), z=point.z(), yaw=0)

        # измененный полет к точке
        start_time = time.time()
        self._log("wait for point reached event with temp scan...")

        have_piro_data = False
        while True:
            if not have_piro_data:
                tmp = self._drone.get_piro_sensor_data(blocking=False)
                if tmp is not None:
                    # self._log(f"piro data: temp={tmp}")
                    if tmp > self._min_fire_temp:
                        have_piro_data = True

            if have_piro_data:
                # точку надо получить блокирующим методом, чтоб уж наверняка
                tmp = self._drone.get_local_position_lps(blocking=True)
                if tmp is not None:
                    have_piro_data = False
                    point_2d = (tmp[0], tmp[1])
                    # self._log(f"point fire: piro={piro_data}, point={point_2d}")
                    self._fire_point_received(point_2d)

                    # и отправим задание лететь куда летели
                    self._drone.go_to_local_point(x=point.x(), y=point.y(), z=point.z(), yaw=0)
                    # self._log(f"current position {point_2d}")

            tmp = self._drone.point_reached(blocking=False)
            if tmp is not None:
                if tmp:
                    break  # прилетел

        end_time = time.time()
        self._log(f"point reached on {end_time - start_time} sec")

    def _on_fire(self, point: tuple | list):
        self._log(f"+++ Fire on {point} +++")
        self._drone.fire_detection(sim=USE_SIMULATOR)
        time.sleep(6)  # на всякий случай 6 секунд, мало ли что

    def _scan_area(self, area_id):
        area_start = CONFIG["areas"][area_id]["start"]
        area_size = CONFIG["areas"][area_id]["size"]
        area_overhead = CONFIG["area"]["max-overhead"]

        self._log("--- Start scan area ---")

        curr_line = 0
        offset_y = (curr_line * self._line_step) + area_start[1]
        while offset_y <= area_start[1] + area_size[1] + area_overhead:
            # считаем, что мы в точке старта
            self._log(f"*** Scan line {curr_line + 1} (offset)")
            if curr_line % 2 == 0:
                self._fly_with_scan((area_start[0] + area_size[0], offset_y))
            else:
                self._fly_with_scan((area_start[0], offset_y))

            curr_line += 1
            offset_y = (curr_line * self._line_step) + area_start[1]

            if offset_y <= area_start[1] + area_size[1] + area_overhead:
                if curr_line % 2 == 1:
                    self.do_waypoint_fly([PointAdapter(area_start[0] + area_size[0], 1, offset_y)])
                else:
                    self.do_waypoint_fly([PointAdapter(area_start[0], 1, offset_y)])

        self._log("=== Scan finished! ===")

    def run(self):
        self._log("Arm...")
        self._drone.arm()

        self._log("Takeoff...")
        self._drone.takeoff()

        # достаем параметры
        area = self._config["start"]
        area_start = CONFIG["areas"][self._config["start"]]["start"]

        # создаем план, согласно ему нужно сначала улететь по оси Y, потом по оси X
        self._log("=== Go to area start point ===")
        self.do_waypoint_fly([
            PointAdapter.from_2d(p, self._height)
            for p in [self._home_point, (self._home_point[0], area_start[1]), area_start]
        ])

        self._scan_area(area)
        if area <= 2:
            self._scan_area(area + 1)
        else:
            area_size = CONFIG["areas"][area]["size"]
            area0_start = CONFIG["areas"][0]["start"]
            self.do_waypoint_fly([
                PointAdapter.from_2d((area_start[0], area_start[1] + area_size[1]), 2),
                PointAdapter.from_2d((5.25, 5.25), 2),
                PointAdapter.from_2d(area0_start, 2),
                PointAdapter.from_2d(area0_start, self._height),
            ])
            self._scan_area(0)
        # self._log("Go to home"))

        self._drone.land()
        # self._log(f"INFO: fire points are: {self._fire_points}")

def main():
    # инициализация
    drones = []
    for drone in CONFIG["drones"]:
        drones.append(ScannerDrone(name=drone, config=CONFIG["drones"][drone]))

    # старт
    for drone in drones:
        drone.start()

    # ожидание завершения всех потоков
    for drone in drones:
        drone.join()

    print("Found points:", ScannerDrone.get_fire_points())
    # drone = LoaderSlave(route_factory, "unworker", CONFIG["drones"]["Worker-0"])
    # drone.run()


if __name__ == "__main__":
    main()