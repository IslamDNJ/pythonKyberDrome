import math
import traceback
import time
from datetime import datetime
from threading import Lock, Thread

from piosdk import Pioneer

USE_SIMULATOR = False

CONFIG = {
    "drones": {
        "Drone-0": {"ip": "127.0.0.1", "port": 8000, "start": 0},
        "Drone-1": {"ip": "127.0.0.1", "port": 8001, "start": 1}
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
        "fire-min-temp": 35,
        "fire-radius": 1.1,
        "scan-height": 1,
        "max-overhead": 0.2
    },
}

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

PIONEER_LOGGING = False
PIONEER_METHOD = 2

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

        return PointAdapter(point[0], point[2], point[1])

    def __str__(self):
        return f"Point (x={self._x}, y={self._y}, z={self._z})"

    def x(self):
        return self._x

    def y(self):
        return self._z

    def z(self):
        return self._y

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

        for point in points:

            self._log(f"go to {point}")
            self._drone.go_to_local_point(x=point.x(), y=point.y(), z=point.z(), yaw=0)

            start_time = time.time()
            self._log("wait for point reached event...")

            while True:
                res = self._drone.point_reached()
                if res is not None:
                    if res:
                        break

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

                    tmp = self._drone.get_local_position_lps(blocking=True)
                    if tmp is not None:
                        have_piro_data = False
                        point_2d = (tmp[0], tmp[1])
                        self._log(f"(sub area scan) point fire: piro={piro_data}, point={point_2d}")
                        fire_points.append(point_2d)

                tmp = self._drone.point_reached(blocking=False)
                if tmp is not None:
                    if tmp:
                        break

        return fire_points

    def _fire_point_received(self, point: list | tuple):

        if ScannerDrone._is_new_fire(point):

            points = self._fire_scan_area(point)

            x, y = 0, 0
            for p in points:
                x += p[0]
                y += p[1]
            x, y = x / len(points), y / len(points)

            ScannerDrone._add_fire_point(point)

            self.do_waypoint_fly([PointAdapter.from_2d((x, y), self._height)])

            self._on_fire(point)

            self.do_waypoint_fly([PointAdapter.from_2d(point, self._height)])

    def _fly_with_scan(self, target_point: tuple | list):

        point = PointAdapter.from_2d(target_point, self._height)
        self._log(f"go to {point} with scan")
        self._drone.go_to_local_point(x=point.x(), y=point.y(), z=point.z(), yaw=0)


        start_time = time.time()
        self._log("wait for point reached event with temp scan...")

        have_piro_data = False
        while True:
            if not have_piro_data:
                tmp = self._drone.get_piro_sensor_data(blocking=False)
                if tmp is not None:

                    if tmp > self._min_fire_temp:
                        have_piro_data = True

            if have_piro_data:

                tmp = self._drone.get_local_position_lps(blocking=True)
                if tmp is not None:
                    have_piro_data = False
                    point_2d = (tmp[0], tmp[1])

                    self._fire_point_received(point_2d)

                    self._drone.go_to_local_point(x=point.x(), y=point.y(), z=point.z(), yaw=0)

            tmp = self._drone.point_reached(blocking=False)
            if tmp is not None:
                if tmp:
                    break

        end_time = time.time()
        self._log(f"point reached on {end_time - start_time} sec")

    def _on_fire(self, point: tuple | list):
        self._log(f"+++ Fire on {point} +++")
        self._drone.fire_detection(sim=USE_SIMULATOR)
        time.sleep(6)

    def _scan_area(self, area_id):
        area_start = CONFIG["areas"][area_id]["start"]
        area_size = CONFIG["areas"][area_id]["size"]
        area_overhead = CONFIG["area"]["max-overhead"]

        self._log("--- Start scan area ---")

        curr_line = 0
        offset_y = (curr_line * self._line_step) + area_start[1]
        while offset_y <= area_start[1] + area_size[1] + area_overhead:

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

        area = self._config["start"]
        area_start = CONFIG["areas"][self._config["start"]]["start"]

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

        self._drone.land()

def main():

    drones = []
    for drone in CONFIG["drones"]:
        drones.append(ScannerDrone(name=drone, config=CONFIG["drones"][drone]))

    for drone in drones:
        drone.start()

    for drone in drones:
        drone.join()

    print("Found points:", ScannerDrone.get_fire_points())

if __name__ == "__main__":
    main()