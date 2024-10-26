import itertools
import math
import time
from datetime import datetime
from threading import Lock, Thread

from piosdk import Pioneer

CONFIG = {
    "drones": {
        "Worker-0": {"ip": "127.0.0.1", "port": 8000, "start": 0},
        "Worker-1": {"ip": "127.0.0.1", "port": 8001, "start": 1},
        "Worker-2": {"ip": "127.0.0.1", "port": 8002, "start": 2},
        "Worker-3": {"ip": "127.0.0.1", "port": 8003, "start": 3}
    },
    "polygon": {
        "size-x": 10.5,
        "size-y": 10.5,
        "start-points": [
            (-0.5, -4.5),
            (-1.5, -4.5),
            (-2.5, -4.5),
            (-3.5, -4.5)
        ],
        "load-points": [
            (4.2, 6.3),
            (6.0, 6.4),
            (6.0, 4.4),
            (4.2, 4.4)
        ],
        "unload-points": [
            (3.1, 7.4),
            (2.9, 2.9),
            (8.6, 3.0),
            (8.5, 7.7)
        ],
        "point-busy-radius": 0.6
    },
    "routes": {
        "echelons": {
            "count": 2,
            "start-height": 1.2,
            "layer-height": 0.8
        },
        "grid-step": 0.1
    }
}

def point_real_to_grid(src: tuple | list):
    """Преобразовать точку из реальных координат в координаты сетки"""
    assert len(src) == 2
    x = round(float(src[0]) / CONFIG["routes"]["grid-step"])
    y = round(float(src[1]) / CONFIG["routes"]["grid-step"])
    return x, y


def point_grid_to_real(src: tuple | list):
    """Преобразовать точку из координат сетки в реальные координаты"""
    assert len(src) == 2
    x = int(src[0]) * CONFIG["routes"]["grid-step"]
    y = int(src[1]) * CONFIG["routes"]["grid-step"]
    return round(x, 3), round(y, 3)


def points_grid_to_real(src: list):
    """Преобразовать массив точек из координат сетки в реальные координаты"""
    result = []
    for p in src:
        result.append(point_grid_to_real(p))
    return result

class Route:
    def __init__(self, factory, points: list, cost: float, real_points: list, height: float):
        self._points = points
        self._height = height
        self._cost = cost
        self._factory = factory
        self._real_points = real_points

    def get_grid_points(self):
        return self._points

    def get_cost(self):
        return self._cost

    def get_real_points(self):
        return PointAdapter.array_from_2d(self._real_points, self._height)

    def __str__(self):
        return f"cost: {self._cost}, points: {self._real_points}"

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._factory.remove_route(self)


def equals_points(p1: tuple | list, p2: tuple | list):
    return p1[0] == p2[0] and p1[1] == p2[1]

class RouteFactory:
    _POINT_DEFAULT = -1
    _POINT_TARGET = -2
    _POINT_BUSY = -3
    _POINT_BUSY_BY_ROUTE = -4
    SQRT_2 = 1.4

    def __init__(self):
        self._lock = Lock()
        self._size_x, self._size_y = point_real_to_grid((CONFIG["polygon"]["size-x"], CONFIG["polygon"]["size-y"]))
        self._grid_step = CONFIG["routes"]["grid-step"]
        self._point_busy_radius = round(CONFIG["polygon"]["point-busy-radius"] / self._grid_step)
        echelons = CONFIG["routes"]["echelons"]
        self._echelons_layer_height = echelons["layer-height"]
        self._echelons = [{"height": self._echelons_layer_height * e + self._echelons_layer_height, "routes": []}
                          for e in range(0, echelons["count"])]

    @staticmethod
    def _optimize_points(source: list):
        """Оптимизировать маршрут для дрона, то есть убрать лишние точки, через которые и так лежит маршрут"""
        assert len(source) > 0

        result = [source[0]]
        if len(source) >= 2:
            vec_x, vec_y = source[0][0] - source[1][0], source[0][1] - source[1][1]
            for index in range(1, len(source)):
                if index == len(source) - 1:

                    result.append(source[index])
                else:

                    new_vec_x = source[index][0] - source[index + 1][0]
                    new_vec_y = source[index][1] - source[index + 1][1]
                    if new_vec_y != vec_y or new_vec_x != vec_x:
                        vec_x, vec_y = new_vec_x, new_vec_y
                        result.append(source[index])

        return result

    def _map_lock_point(self, grid_map, point, point_type=None):
        if point_type is None:
            point_type = RouteFactory._POINT_BUSY
        start_x, start_y = point[0] - self._point_busy_radius, point[1] - self._point_busy_radius
        end_x, end_y = point[0] + self._point_busy_radius, point[1] + self._point_busy_radius
        for x in range(start_x, end_x + 1):
            for y in range(start_y, end_y + 1):
                if x < 0 or x >= self._size_x or y < 0 or y >= self._size_y:
                    continue
                grid_map[x][y] = point_type

    def _map_make_locked_points(self, grid_map, start_point, end_point, echelon):
        for p_src in CONFIG["polygon"]["start-points"]:
            p = point_real_to_grid(p_src)
            if not equals_points(p, start_point) and not equals_points(p, end_point):
                self._map_lock_point(grid_map, p)
        for p_src in CONFIG["polygon"]["load-points"]:
            p = point_real_to_grid(p_src)
            if not equals_points(p, start_point) and not equals_points(p, end_point):
                self._map_lock_point(grid_map, p)

        for route in self._echelons[echelon]["routes"]:
            for point in route.get_grid_points():
                self._map_lock_point(grid_map, point, point_type=RouteFactory._POINT_BUSY_BY_ROUTE)

    def _map_do_calc_costs(self, source_grid, cache: dict | None = None):
        """Одна итерация поиска стоимостей для каждой точки в сетке"""
        changes = False
        found_end_point = False
        dest_grid = source_grid.copy()
        points_with_cost = [

            (-1, 0, 1),
            (1, 0, 1),
            (0, -1, 1),
            (0, 1, 1),

            (-1, -1, RouteFactory.SQRT_2),
            (-1, 1, RouteFactory.SQRT_2),
            (1, -1, RouteFactory.SQRT_2),
            (1, 1, RouteFactory.SQRT_2),
        ]

        if "outer-zone" not in cache:
            cache["outer-zone"] = {
                "start": (cache["start-point"][0], cache["start-point"][1]),
                "end": (cache["start-point"][0], cache["start-point"][1]),
            }

        outer_x1, outer_y1 = cache["outer-zone"]["start"][0], cache["outer-zone"]["start"][1]
        outer_x2, outer_y2 = cache["outer-zone"]["end"][0], cache["outer-zone"]["end"][1]

        outer_x1, outer_y1 = max(outer_x1 - 1, 0), max(outer_y1 - 1, 0)
        outer_x2, outer_y2 = min(outer_x2 + 1, self._size_x - 1), min(outer_y2 + 1, self._size_y - 1)

        cache["outer-zone"] = {
            "start": (outer_x1, outer_y1),
            "end": (outer_x2, outer_y2),
        }

        for x in range(outer_x1, outer_x2 + 1):
            for y in range(outer_y1, outer_y2 + 1):
                src_v = source_grid[x][y]

                if src_v != RouteFactory._POINT_DEFAULT and src_v != RouteFactory._POINT_TARGET:
                    continue

                min_cost = math.inf
                for point in points_with_cost:
                    p_x, p_y, cost = x + point[0], y + point[1], point[2]
                    if p_x < 0 or p_x >= self._size_x or p_y < 0 or p_y >= self._size_y:
                        continue
                    v = source_grid[p_x][p_y]
                    if v >= 0:
                        min_cost = min(min_cost, v + cost)

                if min_cost != math.inf:
                    changes = True
                    dest_grid[x][y] = min_cost
                    if src_v == RouteFactory._POINT_TARGET:
                        found_end_point = True

        return dest_grid, changes, found_end_point, cache

    def _compute_route(self, start: tuple | list, end: tuple | list, echelon: int):

        grid_map = np.full((self._size_x, self._size_y), RouteFactory._POINT_DEFAULT, dtype=float)
        self._map_make_locked_points(grid_map, start, end, echelon)

        grid_map[start[0]][start[1]] = RouteFactory._POINT_TARGET
        grid_map[end[0]][end[1]] = 0

        _calc_costs_cache = {"start-point": end}
        while True:
            grid_map, changes, found_end_point, _calc_costs_cache = self._map_do_calc_costs(grid_map, _calc_costs_cache)
            if found_end_point:
                break
            if not changes:
                return None, math.inf

        circle_points = [
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, 1),
            (1, 1),
            (1, 0),
            (1, -1),
            (0, -1)
        ]

        curr_x, curr_y = start[0], start[1]
        result = [(curr_x, curr_y)]

        while True:

            if equals_points((curr_x, curr_y), end):
                break

            min_cost = math.inf
            next_x, next_y = None, None
            for point in circle_points:
                p_x, p_y = curr_x + point[0], curr_y + point[1]
                if p_x < 0 or p_x >= self._size_x or p_y < 0 or p_y >= self._size_y:
                    continue
                new_cost = grid_map[p_x][p_y]
                if new_cost < 0:
                    continue

                if new_cost < min_cost:
                    min_cost = new_cost
                    next_x, next_y = p_x, p_y

            assert next_x is not None and next_y is not None

            curr_x, curr_y = next_x, next_y

            result.append((curr_x, curr_y))

        return result, grid_map[start[0]][start[1]]

    def create_route(self, start: tuple | list, end: tuple | list):
        """Создать маршрут, точки задаются в реальных координатах"""
        start = point_real_to_grid(start)
        end = point_real_to_grid(end)

        with self._lock:
            routes = []

            for echelon in range(0, len(self._echelons)):
                points, cost = self._compute_route(start, end, echelon)
                if points is None:
                    return None
                routes.append({
                    "echelon": echelon,
                    "points": points,
                    "cost": (cost * self._grid_step) + (echelon * self._echelons_layer_height)
                })

            if len(routes) == 0:
                return None

            min_cost, route_src = math.inf, None
            for r in routes:
                if min_cost > r["cost"]:
                    min_cost = r["cost"]
                    route_src = r

            route = Route(self,
                          points=route_src["points"],
                          real_points=points_grid_to_real(RouteFactory._optimize_points(route_src["points"])),
                          cost=route_src["cost"] * CONFIG["routes"]["grid-step"],
                          height=self._echelons[route_src["echelon"]]["height"])
            self._echelons[route_src["echelon"]]["routes"].append(route)

            return route

    def remove_route(self, route: Route):
        with self._lock:
            for echelon in self._echelons:
                if route in echelon["routes"]:
                    echelon["routes"].remove(route)



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

class BasicSlave(Thread):
    def __init__(self, pioneer: Pioneer = None, ip: str = "127.0.0.1", port: int = 8000,
                 slave_name: str = "slave", enable_log: bool = True):
        Thread.__init__(self)
        self._slave_name = slave_name
        self._enable_log = enable_log
        if pioneer is not None:
            self._slave = pioneer
        else:
            if ip is None or port is None:
                raise Exception("ip or port not defined")
            self._slave = Pioneer(logger=PIONEER_LOGGING, method=PIONEER_METHOD, pioneer_ip=ip,
                                  pioneer_mavlink_port=port)

    def _log(self, message):
        if self._enable_log:
            print(f"{datetime.now().strftime('%H:%M:%S.%f')[:-3]} {self._slave_name}: {message}")

    def do_waypoint_fly(self, points: list):
        for point in points:

            self._log(f"go to {point}")
            self._slave.go_to_local_point(x=point.x(), y=point.y(), z=point.z(), yaw=0)

            start_time = time.time()
            self._log("wait for point reached event...")

            while True:
                res = self._slave.point_reached()
                if res is not None:
                    if res:
                        break

            end_time = time.time()
            self._log(f"point reached on {end_time - start_time} sec")

class WorkFactory:
    def __init__(self, load_points, unload_points):
        self._works = list(itertools.product(load_points, unload_points))
        self._lock = Lock()
        self._locked_points = []

    def _lock_point(self, point):
        if point not in self._locked_points:
            self._locked_points.append(point)

    def _try_get_work(self, curr_point):
        with self._lock:
            if len(self._works) > 0:
                works = []
                for w in self._works:
                    if equals_points(w[0], curr_point):
                        if w[1] in self._locked_points:
                            continue
                    elif equals_points(w[1], curr_point):
                        if w[0] in self._locked_points:
                            continue
                    else:
                        if w[0] in self._locked_points or w[1] in self._locked_points:
                            continue
                    works.append(w)

                if len(works) == 0:
                    return True, None

                min_len = math.inf
                min_index = 0
                for i in range(0, len(works)):
                    vec_len = math.sqrt((curr_point[0] - works[i][0][0]) ** 2 + (curr_point[1] - works[i][0][1]) ** 2)
                    if vec_len < min_len:
                        min_len = vec_len
                        min_index = i

                self._lock_point(works[min_index][0])
                self._lock_point(works[min_index][1])

                self._works.remove(works[min_index])

                return True, works[min_index]
            else:
                return False, None

    def get_work(self, curr_point: tuple | list):
        while True:
            result, work = self._try_get_work(curr_point)
            if work is not None:
                return work
            if not result:
                return None
            time.sleep(0.5)

    def remove_lock(self, point):
        with self._lock:
            if point in self._locked_points:
                self._locked_points.remove(point)
            else:
                print(f"WorkFactory: WARM: lock point {point} not found in locks!")


class LoaderSlave(BasicSlave):
    def __init__(self, route_factory: RouteFactory, work_factory: WorkFactory, name, config):
        super().__init__(slave_name=name, ip=config["ip"], port=config["port"])
        self._config = config
        self._target = None
        self._route_factory = route_factory
        self._work_factory = work_factory
        self._current_pos = None
        self._start = CONFIG["polygon"]["start-points"][config["start"]]
        self._current_pos = self._start

    def create_route(self, start, end):
        while True:
            route = self._route_factory.create_route(start, end)
            if route is not None:
                return route
            time.sleep(5)

    def do_move(self, target: tuple, unlock_after_takeoff=False):
        self._log("Arm...")
        self._slave.arm()

        self._log("Takeoff...")
        self._slave.takeoff()
        time.sleep(3)

        with self.create_route(self._current_pos, target) as route:
            if unlock_after_takeoff:
                self._work_factory.remove_lock(self._current_pos)
            self._log("Flying to target by waypoints")
            self.do_waypoint_fly(route.get_real_points())

        self._log("Land...")
        self._slave.land()
        self._current_pos = target

    def run(self):
        while True:
            work = self._work_factory.get_work(self._current_pos)
            if work is None:
                break
            self._log(f"Received work {work}")

            need_unlock_start = not equals_points(self._current_pos, work[1])

            self._log(f"Do move to load point {work[0]} (unlock: {need_unlock_start})")
            self.do_move(work[0], need_unlock_start)

            self._log("Do loading...")
            time.sleep(5)

            self._log(f"Do move to unload point {work[1]}")
            self.do_move(work[1], True)

        self.do_move(self._start, True)

def main():
    route_factory = RouteFactory()
    work_factory = WorkFactory(CONFIG["polygon"]["load-points"], CONFIG["polygon"]["unload-points"])
    drones = []
    for drone in CONFIG["drones"]:
        drones.append(LoaderSlave(name=drone,
                                  config=CONFIG["drones"][drone],
                                  route_factory=route_factory,
                                  work_factory=work_factory))

    for drone in drones:
        drone.start()

if __name__ == "__main__":
    main()