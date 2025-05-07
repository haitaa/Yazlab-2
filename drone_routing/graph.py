import math
from typing import Dict, List, Tuple
from .models import Drone, DeliveryPoint, NoFlyZone

class Graph:
    """
    Düğümler ve kenarlar üzerinden teslimat rotası planlaması için graf yapısı.
    """
    def __init__(self,
                 drones: List[Drone],
                 deliveries: List[DeliveryPoint],
                 no_fly_zones: List[NoFlyZone]):
        self.drones = drones
        self.deliveries = deliveries
        self.no_fly_zones = no_fly_zones
        # Düğümler: anahtar olarak 'drone_{id}' ve 'dp_{id}' kullanılır
        self.nodes: Dict[str, Drone or DeliveryPoint] = self._init_nodes()
        # Komşuluk listesi: node_key -> list of (neighbor_key, cost)
        self.adjacency: Dict[str, List[Tuple[str, float]]] = {}
        self.build_graph()
        # Ceza sabiti: no-fly zone ihlalinde ek maliyet
        self.NO_FLY_PENALTY = 10000

    def _init_nodes(self) -> Dict[str, Drone or DeliveryPoint]:
        nodes: Dict[str, Drone or DeliveryPoint] = {}
        for dr in self.drones:
            nodes[f"drone_{dr.id}"] = dr
        for dp in self.deliveries:
            nodes[f"dp_{dp.id}"] = dp
        return nodes

    def build_graph(self) -> None:
        """
        Her düğüm için geçerli komşuları ve maliyetleri hesaplayarak grafı oluşturur.
        Drone başlangıçlarından yalnızca teslimat noktasına, teslimat düğümlerinden diğer teslimat düğümlerine izin verir.
        Ağırlık kapasitesini aşan kenarlar elenir.
        """
        # Teslimat önceliklerinden maksimum değeri al
        max_priority = max((dp.priority for dp in self.deliveries), default=5)
        for src_key, src_node in self.nodes.items():
            self.adjacency[src_key] = []
            for dst_key, dst_node in self.nodes.items():
                if src_key == dst_key:
                    continue
                # Hedef mutlaka teslimat noktası olmalı
                if not isinstance(dst_node, DeliveryPoint):
                    continue
                # Drone'dan teslimata: ağırlık kapasitesi kontrolü
                if isinstance(src_node, Drone) and dst_node.weight > src_node.max_weight:
                    continue
                # Teslimat düğümünden teslimat düğümüne: önceki paket teslim edildiği için kapasite kontrolü yok
                cost = self._compute_cost(src_node, dst_node, max_priority)
                self.adjacency[src_key].append((dst_key, cost))

    def _compute_cost(self,
                      src_node: Drone or DeliveryPoint,
                      dst_node: DeliveryPoint,
                      max_priority: int) -> float:
        """
        Kenar maliyeti: iki düğüm arasındaki mesafe * paket ağırlığı + önceliğe göre ceza.
        Ceza = (max_priority - dst.priority) * 100
        """
        # Kaynak konumu
        src_pos = src_node.start_pos if isinstance(src_node, Drone) else src_node.pos  # type: ignore
        dst_pos = dst_node.pos
        dist = self._distance(src_pos, dst_pos)
        penalty = (max_priority - dst_node.priority) * 100
        return dist * dst_node.weight + penalty

    @staticmethod
    def _distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """Öklidyen mesafe hesaplama."""
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _time_to_min(t_str: str) -> float:
        """Zaman string'ini (HH:MM) dakika cinsinden dönüştürür."""
        h, m = map(int, t_str.split(':'))
        return h * 60 + m

    def _point_in_polygon(self, point: Tuple[float, float], polygon: List[Tuple[float, float]]) -> bool:
        """
        Noktanın çokgen içinde olup olmadığını kontrol eder.
        """
        x, y = point
        inside = False
        n = len(polygon)
        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[(i + 1) % n]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
        return inside

    def _segments_intersect(self, p1: Tuple[float, float], p2: Tuple[float, float],
                            q1: Tuple[float, float], q2: Tuple[float, float]) -> bool:
        """
        İki segmentin kesişip kesişmediğini kontrol eder.
        """
        def orientation(a, b, c):
            return (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
        def on_segment(a, b, c):
            return (min(a[0], c[0]) <= b[0] <= max(a[0], c[0]) and
                    min(a[1], c[1]) <= b[1] <= max(a[1], c[1]))
        o1 = orientation(p1, p2, q1)
        o2 = orientation(p1, p2, q2)
        o3 = orientation(q1, q2, p1)
        o4 = orientation(q1, q2, p2)
        if o1 * o2 < 0 and o3 * o4 < 0:
            return True
        # Özel durumlar
        if o1 == 0 and on_segment(p1, q1, p2): return True
        if o2 == 0 and on_segment(p1, q2, p2): return True
        if o3 == 0 and on_segment(q1, p1, q2): return True
        if o4 == 0 and on_segment(q1, p2, q2): return True
        return False

    def _segment_crosses_polygon(self, a: Tuple[float, float], b: Tuple[float, float],
                                  polygon: List[Tuple[float, float]]) -> bool:
        """
        Bir segmentin poligon içine girip girmediğini kontrol eder.
        """
        # Eğer uç noktalar polygon içinde ise
        if self._point_in_polygon(a, polygon) or self._point_in_polygon(b, polygon):
            return True
        # Segment kenarlarla kesişiyor mu?
        n = len(polygon)
        for i in range(n):
            if self._segments_intersect(a, b, polygon[i], polygon[(i + 1) % n]):
                return True
        return False

    def heuristic(self, node_key: str, goal_key: str) -> float:
        """
        A* tahmin fonksiyonu: mesafe + no-fly zone cezası.
        """
        node = self.nodes[node_key]
        goal = self.nodes[goal_key]
        src_pos = node.start_pos if isinstance(node, Drone) else node.pos  # type: ignore
        dst_pos = goal.pos
        h = self._distance(src_pos, dst_pos)
        # No-fly cezası
        for zone in self.no_fly_zones:
            if self._segment_crosses_polygon(src_pos, dst_pos, zone.coordinates):
                h += self.NO_FLY_PENALTY
        return h

    def find_path(self, start_key: str, goal_key: str) -> Tuple[List[str], float]:
        """
        A* ile en kısa maliyetli yolu bulur.
        """
        import heapq
        from collections import defaultdict

        # Zaman penceresi entegrasyonu için başlangıç zamanı
        earliest_start = min(self._time_to_min(dp.time_window[0]) for dp in self.deliveries)
        g_time = defaultdict(lambda: float('inf'))
        g_time[start_key] = earliest_start
        # Drone hızını al
        if start_key.startswith("drone_"):
            dr_id = int(start_key.split("_")[1])
            drone_speed = next(d.speed for d in self.drones if d.id == dr_id)
        else:
            raise ValueError("find_path başlangıcı bir drone düğümü olmalı")

        open_set = []
        g_score = defaultdict(lambda: float('inf'))
        came_from = {}
        g_score[start_key] = 0
        f_score = {start_key: self.heuristic(start_key, goal_key)}
        heapq.heappush(open_set, (f_score[start_key], start_key))

        closed_set = set()

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal_key:
                # Yolun yeniden oluşturulması
                path = []
                node = current
                while node in came_from:
                    path.append(node)
                    node = came_from[node]
                path.append(start_key)
                return path[::-1], g_score[current]
            if current in closed_set:
                continue
            closed_set.add(current)

            for neighbor, cost in self.adjacency[current]:
                # maliyet
                tentative_g = g_score[current] + cost
                # zaman ve konum hesaplama
                src_node = self.nodes[current]
                dst_node = self.nodes[neighbor]
                src_pos = src_node.start_pos if isinstance(src_node, Drone) else src_node.pos  # type: ignore
                dst_pos = dst_node.pos
                # seyahat süresi (dakika cinsinden)
                travel_time = (self._distance(src_pos, dst_pos) / drone_speed) / 60
                arrival_time = g_time[current] + travel_time
                # zaman penceresi kontrolü
                if isinstance(dst_node, DeliveryPoint):
                    ws = self._time_to_min(dst_node.time_window[0])
                    we = self._time_to_min(dst_node.time_window[1])
                    if arrival_time > we:
                        continue
                    if arrival_time < ws:
                        arrival_time = ws
                # güncelleme
                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    g_time[neighbor] = arrival_time
                    f = tentative_g + self.heuristic(neighbor, goal_key)
                    heapq.heappush(open_set, (f, neighbor))

        return [], float('inf')

    def solve_tsp_tw_for_drone(self, drone_id: int, dp_ids: List[int]) -> Tuple[List[int], float, float]:
        """
        Verilen drone ve teslimat ID'leri için TSPTW çözer.
        Geri döner: sıralı teslimat ID listesi, toplam seyahat süresi (dakika), toplam bekleme süresi (dakika).
        """
        n = len(dp_ids)
        if n == 0:
            return [], 0.0, 0.0
        # Başlangıç düğümü
        start_key = f"drone_{drone_id}"
        if start_key not in self.nodes:
            raise ValueError(f"Drone {drone_id} bulunamadı")
        start_node = self.nodes[start_key]  # type: ignore
        start_pos = start_node.start_pos
        # Erken başlangıç zamanı
        earliest_start = min(self._time_to_min(self.nodes[f"dp_{i}"].time_window[0]) for i in dp_ids)
        # Koordinatlar: 0=start, 1..n=dp
        coords = [start_pos] + [self.nodes[f"dp_{i}"].pos for i in dp_ids]
        # Drone hızı
        speed = next(d.speed for d in self.drones if d.id == drone_id)
        # Seyahat süresi matrisi (dakika)
        travel_time = [[0.0]*(n+1) for _ in range(n+1)]
        for u in range(n+1):
            for v in range(n+1):
                if u == v: continue
                dist = self._distance(coords[u], coords[v])
                travel_time[u][v] = dist / speed / 60
        # Zaman pencereleri
        ws = [self._time_to_min(self.nodes[f"dp_{i}"].time_window[0]) for i in dp_ids]
        we = [self._time_to_min(self.nodes[f"dp_{i}"].time_window[1]) for i in dp_ids]
        # DP tabloları
        dp_table = [dict() for _ in range(1<<n)]
        parent = [dict() for _ in range(1<<n)]
        # Başlangıç durumları
        for j in range(n):
            t0 = earliest_start + travel_time[0][j+1]
            if t0 > we[j]: continue
            arrival = max(t0, ws[j])
            mask = 1<<j
            dp_table[mask][j] = arrival
            parent[mask][j] = -1
        # DP iterasyonu
        for mask in range(1<<n):
            for j, arr_time in dp_table[mask].items():
                if not (mask & (1<<j)): continue
                for k in range(n):
                    if mask & (1<<k): continue
                    arr = arr_time + travel_time[j+1][k+1]
                    if arr > we[k]: continue
                    arr_w = max(arr, ws[k])
                    new_mask = mask | (1<<k)
                    if arr_w < dp_table[new_mask].get(k, float('inf')):
                        dp_table[new_mask][k] = arr_w
                        parent[new_mask][k] = j
        full_mask = (1<<n) - 1
        # En iyi son düğüm
        best_j, best_t = None, float('inf')
        for j, t in dp_table[full_mask].items():
            if t < best_t:
                best_t = t
                best_j = j
        if best_j is None:
            return [], float('inf'), float('inf')
        # Yolun yeniden oluşturulması
        seq = []
        mask, j = full_mask, best_j
        while j != -1:
            seq.append(dp_ids[j])
            prev_j = parent[mask][j]
            mask ^= (1<<j)
            j = prev_j
        seq.reverse()
        # Toplam seyahat ve bekleme süreleri
        current = earliest_start
        total_travel = 0.0
        total_wait = 0.0
        prev_pos = start_pos
        for di in seq:
            dst = self.nodes[f"dp_{di}"]
            d = self._distance(prev_pos, dst.pos)
            tt = d / speed / 60
            arr = current + tt
            w_start = self._time_to_min(dst.time_window[0])
            wait = max(0, w_start - arr)
            total_wait += wait
            if arr < w_start: arr = w_start
            total_travel += tt
            current = arr
            prev_pos = dst.pos
        return seq, total_travel, total_wait 