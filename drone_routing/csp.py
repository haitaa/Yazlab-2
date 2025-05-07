from typing import Dict, List
from .models import Drone, DeliveryPoint
from .graph import Graph

class CSP:
    """
    Dinamik kısıtlar için basit CSP: her drone en fazla bir teslimat yapar ve no-fly zone ihlali olmayan atamalar bulur.
    """
    def __init__(self, graph: Graph):
        self.graph = graph
        self.drones: List[Drone] = graph.drones
        self.deliveries: List[DeliveryPoint] = graph.deliveries

    def solve(self) -> Dict[int, int]:
        """
        Teslimatların drone atamalarını döner. {delivery_id: drone_id}
        """
        # Önceliğe göre sıralanmış teslimatlar (yüksek öncelik önce)
        sorted_deliveries = sorted(self.deliveries, key=lambda d: -d.priority)
        assignment: Dict[int, int] = {}
        used_drones = set()

        for dp in sorted_deliveries:
            for dr in self.drones:
                if dr.id in used_drones:
                    continue
                # Ağırlık kapasitesi kontrolü
                if dp.weight > dr.max_weight:
                    continue
                # Rota var mı ve no-fly ihlal yok mu?
                start_key = f"drone_{dr.id}"
                goal_key = f"dp_{dp.id}"
                path, cost = self.graph.find_path(start_key, goal_key)
                if path:
                    assignment[dp.id] = dr.id
                    used_drones.add(dr.id)
                    break
        return assignment 