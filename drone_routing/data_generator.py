import random
from typing import List, Tuple
from .models import Drone, DeliveryPoint, NoFlyZone


def generate_drones(n: int,
                    area_size: Tuple[float, float] = (1000.0, 1000.0)) -> List[Drone]:
    """
    n adet drone oluşturur. start_pos, max_weight, battery, speed rastgeledir.
    area_size: (max_x, max_y)
    """
    drones: List[Drone] = []
    for i in range(1, n + 1):
        max_w = round(random.uniform(1.0, 5.0), 2)
        battery = random.randint(2000, 10000)
        speed = round(random.uniform(5.0, 15.0), 2)
        x = round(random.uniform(0, area_size[0]), 2)
        y = round(random.uniform(0, area_size[1]), 2)
        drones.append(Drone(id=i, max_weight=max_w, battery=battery, speed=speed, start_pos=(x, y)))
    return drones


def generate_deliveries(m: int,
                        area_size: Tuple[float, float] = (1000.0, 1000.0)) -> List[DeliveryPoint]:
    """
    m adet teslimat noktası oluşturur. pos, weight, priority ve time_window rastgeledir.
    """
    deliveries: List[DeliveryPoint] = []
    for i in range(1, m + 1):
        x = round(random.uniform(0, area_size[0]), 2)
        y = round(random.uniform(0, area_size[1]), 2)
        weight = round(random.uniform(0.1, 3.0), 2)
        priority = random.randint(1, 5)
        # time window baslangici 9-16 arasi, 1 saatlik pencere
        start_h = random.randint(9, 16)
        time_window = (f"{start_h:02d}:00", f"{start_h+1:02d}:00")
        deliveries.append(DeliveryPoint(id=i, pos=(x, y), weight=weight, priority=priority, time_window=time_window))
    return deliveries


def generate_no_fly_zones(k: int,
                          area_size: Tuple[float, float] = (1000.0, 1000.0)) -> List[NoFlyZone]:
    """
    k adet no-fly zone (dikdortgen) oluşturur. coordinates ve active_time rastgeledir.
    """
    zones: List[NoFlyZone] = []
    for i in range(1, k + 1):
        # dikdortgen merkez ve boyut
        cx = random.uniform(0, area_size[0])
        cy = random.uniform(0, area_size[1])
        w = random.uniform(area_size[0] * 0.05, area_size[0] * 0.2)
        h = random.uniform(area_size[1] * 0.05, area_size[1] * 0.2)
        x1, y1 = cx - w/2, cy - h/2
        x2, y2 = cx + w/2, cy - h/2
        x3, y3 = cx + w/2, cy + h/2
        x4, y4 = cx - w/2, cy + h/2
        coords = [(round(x1,2), round(y1,2)), (round(x2,2), round(y2,2)),
                  (round(x3,2), round(y3,2)), (round(x4,2), round(y4,2))]
        # aktif zaman araligi 9-17 arasi 1-2 saat arasinda
        start_h = random.randint(9, 16)
        length = random.randint(1, 2)
        active_time = (f"{start_h:02d}:00", f"{start_h+length:02d}:00")
        zones.append(NoFlyZone(id=i, coordinates=coords, active_time=active_time))
    return zones 