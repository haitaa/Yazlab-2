import random
from typing import List, Dict, Tuple
from .graph import Graph
from .models import Drone, DeliveryPoint
from .energy_model import compute_energy

class GeneticAlgorithm:
    """
    Genetik Algoritma ile teslimat rotalarını optimize eden sınıf.
    Birey temsili: her drone için teslimat ID listesi {drone_id: [delivery_id, ...]}
    """
    def __init__(self,
                 graph: Graph,
                 population_size: int = 50,
                 generations: int = 100,
                 crossover_rate: float = 0.8,
                 mutation_rate: float = 0.2,
                 alpha: float = 10.0,  # teslimat sayısı ağırlığı
                 beta: float = 1.0,    # enerji tüketimi ağırlığı
                 gamma: float = 100.0, # kural ihlali ağırlığı
                 wind_speed: float = 0.0  # ortam rüzgâr hızı (m/s)
                 ):
        self.graph = graph
        self.drones: List[Drone] = graph.drones
        self.deliveries: List[DeliveryPoint] = graph.deliveries
        self.population_size = population_size
        self.generations = generations
        self.crossover_rate = crossover_rate
        self.mutation_rate = mutation_rate
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        self.wind_speed = wind_speed

    def _initialize_population(self) -> List[Dict[int, List[int]]]:
        population = []
        drone_ids = [dr.id for dr in self.drones]
        for _ in range(self.population_size):
            individual: Dict[int, List[int]] = {dr_id: [] for dr_id in drone_ids}
            for dp in self.deliveries:
                # atama ihtimali: belirli dronlar ya da atanmasın
                possible = [dr.id for dr in self.drones if dp.weight <= dr.max_weight] + [None]
                choice = random.choice(possible)
                if choice is not None:
                    individual[choice].append(dp.id)
            # rotaları karıştır
            for dr_id in drone_ids:
                random.shuffle(individual[dr_id])
            population.append(individual)
        return population

    def _evaluate(self, individual: Dict[int, List[int]]) -> float:
        """
        Fitness değerlendirmesi: alpha * teslimat_sayısı - beta * enerji - gamma * ihlal
        """
        # time window entegrasyonu için başlangıç zamanını al (dakika)
        earliest_start = min(self.graph._time_to_min(dp.time_window[0]) for dp in self.deliveries)
        delivered_count = 0
        energy = 0.0
        violations = 0
        # her drone için rota maliyetini hesapla
        for dr in self.drones:
            route = individual.get(dr.id, [])
            prev_pos = dr.start_pos
            current_time = earliest_start
            for dp_id in route:
                dp = next(d for d in self.deliveries if d.id == dp_id)
                # mesafe ve seyahat süresi (saat -> dakika)
                dist = self.graph._distance(prev_pos, dp.pos)
                travel_time = (dist / dr.speed) * (1/60)
                arrival_time = current_time + travel_time
                ws = self.graph._time_to_min(dp.time_window[0])
                we = self.graph._time_to_min(dp.time_window[1])
                # pencere kontrolü
                if arrival_time > we:
                    violations += 1
                    break
                if arrival_time < ws:
                    arrival_time = ws
                # gerçekçi enerji tüketimi
                # elevation_gain = 0 (varsayılan)
                energy += compute_energy(distance=dist,
                                         payload_weight=dp.weight,
                                         speed=dr.speed,
                                         wind_speed=self.wind_speed)
                delivered_count += 1
                current_time = arrival_time
                prev_pos = dp.pos
        fitness = self.alpha * delivered_count - self.beta * energy - self.gamma * violations
        return fitness

    def _tournament_selection(self, population: List[Dict[int, List[int]]],
                              fitnesses: List[float], k: int = 3) -> Dict[int, List[int]]:
        """K-tournament seçimi"""
        selected = random.sample(list(zip(population, fitnesses)), k)
        selected.sort(key=lambda x: x[1], reverse=True)
        return selected[0][0]

    def _crossover(self, parent1: Dict[int, List[int]],
                   parent2: Dict[int, List[int]]) -> Tuple[Dict[int, List[int]], Dict[int, List[int]]]:
        """Uniform crossover drone bazlı"""
        child1, child2 = {}, {}
        for dr_id in parent1.keys():
            if random.random() < 0.5:
                child1[dr_id] = parent1[dr_id][:]
                child2[dr_id] = parent2[dr_id][:]
            else:
                child1[dr_id] = parent2[dr_id][:]
                child2[dr_id] = parent1[dr_id][:]
        # teslimat kopyalarını temizle
        def fix(ind):
            all_dp = [dp.id for dp in self.deliveries]
            seen = set()
            for dr_id, route in ind.items():
                new_route = []
                for dp_id in route:
                    if dp_id not in seen:
                        seen.add(dp_id)
                        new_route.append(dp_id)
                ind[dr_id] = new_route
            # unassigned dp yol
        fix(child1)
        fix(child2)
        return child1, child2

    def _mutate(self, individual: Dict[int, List[int]]) -> None:
        """Bir teslimatı rasgele drona taşır ya da bırakır"""
        all_assigned = [(dr_id, dp_id) for dr_id, route in individual.items() for dp_id in route]
        if not all_assigned:
            return
        dr_id, dp_id = random.choice(all_assigned)
        # çıkar
        individual[dr_id].remove(dp_id)
        # yeniden ata
        possible = [dr.id for dr in self.drones if dp_id in [] or True] + [None]
        # basit: rastgele dron
        choice = random.choice([dr.id for dr in self.drones] + [None])
        if choice is not None:
            individual[choice].append(dp_id)

    def _route_distance(self, dr_id: int, route: List[int]) -> float:
        """
        Belirli bir dronun rotasındaki toplam mesafeyi hesaplar.
        """
        dr = next(d for d in self.drones if d.id == dr_id)
        prev_pos = dr.start_pos
        total = 0.0
        for dp_id in route:
            dp = next(d for d in self.deliveries if d.id == dp_id)
            total += self.graph._distance(prev_pos, dp.pos)
            prev_pos = dp.pos
        return total

    def _two_opt_route(self, dr_id: int, route: List[int]) -> List[int]:
        """
        2-opt algoritmasıyla tek bir rota üzerinde iyileştirme yapar.
        """
        best = route[:]
        improved = True
        while improved:
            improved = False
            for i in range(len(best) - 1):
                for j in range(i + 2, len(best)):
                    new_route = best[:i+1] + best[i+1:j+1][::-1] + best[j+1:]
                    if self._route_distance(dr_id, new_route) < self._route_distance(dr_id, best):
                        best = new_route
                        improved = True
                        break
                if improved:
                    break
        return best

    def _apply_local_search(self, individual: Dict[int, List[int]]) -> None:
        """
        Her dronun rotasına 2-opt local search uygular.
        """
        for dr_id, route in individual.items():
            if len(route) > 2:
                individual[dr_id] = self._two_opt_route(dr_id, route)

    def run(self) -> Tuple[Dict[int, List[int]], float]:
        """Genetik algoritmayı çalıştırır ve en iyi bireyi döner"""
        population = self._initialize_population()
        best_ind, best_fit = None, float('-inf')
        for _ in range(self.generations):
            fitnesses = [self._evaluate(ind) for ind in population]
            # elitizm
            for ind, fit in zip(population, fitnesses):
                if fit > best_fit:
                    best_fit = fit
                    best_ind = ind
            new_pop = []
            while len(new_pop) < self.population_size:
                p1 = self._tournament_selection(population, fitnesses)
                p2 = self._tournament_selection(population, fitnesses)
                if random.random() < self.crossover_rate:
                    c1, c2 = self._crossover(p1, p2)
                else:
                    c1, c2 = p1.copy(), p2.copy()
                if random.random() < self.mutation_rate:
                    self._mutate(c1)
                if random.random() < self.mutation_rate:
                    self._mutate(c2)
                # Local search uygulama
                self._apply_local_search(c1)
                self._apply_local_search(c2)
                new_pop.extend([c1, c2])
            population = new_pop[:self.population_size]
        return best_ind, best_fit 