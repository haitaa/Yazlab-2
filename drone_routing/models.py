from dataclasses import dataclass
from typing import Tuple, List

@dataclass
class Drone:
    id: int  # Drone'un benzersiz kimlik numarası
    max_weight: float  # Taşıyabileceği maksimum ağırlık (kg)
    battery: int  # Batarya kapasitesi (mAh)
    speed: float  # Hızı (m/s)
    start_pos: Tuple[float, float]  # Başlangıç koordinatları (x, y)

@dataclass
class DeliveryPoint:
    id: int  # Teslimat noktasının benzersiz kimlik numarası
    pos: Tuple[float, float]  # Koordinatlar (x, y)
    weight: float  # Paketin ağırlığı (kg)
    priority: int  # Öncelik seviyesi (1-5)
    time_window: Tuple[str, str]  # Kabul edilebilir zaman aralığı, örn. ("09:00", "10:00")

@dataclass
class NoFlyZone:
    id: int  # Bölgenin benzersiz kimlik numarası
    coordinates: List[Tuple[float, float]]  # Köşe noktaları listesi
    active_time: Tuple[str, str]  # Aktif olduğu zaman aralığı, örn. ("09:30", "11:00") 