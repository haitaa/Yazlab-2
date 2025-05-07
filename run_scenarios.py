import time
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from drone_routing.data_generator import generate_drones, generate_deliveries, generate_no_fly_zones
from drone_routing.models import Drone
from drone_routing.graph import Graph
from drone_routing.csp import CSP
from drone_routing.ga import GeneticAlgorithm


def evaluate_solution(individual, graph):
    delivered = 0
    energy = 0.0
    violations = 0
    total_wait = 0.0
    # başlangıç zamanı (dakika)
    earliest_start = min(graph._time_to_min(dp.time_window[0]) for dp in graph.deliveries)
    for dr_id, route in individual.items():
        drone = next(d for d in graph.drones if d.id == dr_id)
        prev_pos = drone.start_pos
        current_time = earliest_start
        for dp_id in route:
            dp = next(d for d in graph.deliveries if d.id == dp_id)
            # mesafe ve seyahat süresi (dakika)
            dist = graph._distance(prev_pos, dp.pos)
            travel_time = (dist / drone.speed) * (1/60)
            arrival_time = current_time + travel_time
            ws = graph._time_to_min(dp.time_window[0])
            we = graph._time_to_min(dp.time_window[1])
            # zaman penceresi kontrolü
            if arrival_time > we:
                violations += 1
                break
            wait_time = max(0, ws - arrival_time)
            total_wait += wait_time
            if arrival_time < ws:
                arrival_time = ws
            energy += dist * dp.weight
            delivered += 1
            prev_pos = dp.pos
            current_time = arrival_time
    avg_wait = total_wait / delivered if delivered > 0 else 0.0
    return delivered, energy, violations, avg_wait


def run_scenario(n_drones, m_deliveries, k_zones, title):
    print(f"--- {title} Başlıyor ---")
    drones = generate_drones(n_drones)
    deliveries = generate_deliveries(m_deliveries)
    zones = generate_no_fly_zones(k_zones)
    graph = Graph(drones, deliveries, zones)

    # CSP
    start = time.time()
    csp = CSP(graph)
    assignments = csp.solve()
    t_csp = time.time() - start
    # CSP sonuçlarını değerlendir
    # assignments: {delivery_id: drone_id} -> drone bazlı rota
    dr_routes = {dr.id: [] for dr in drones}
    for dp_id, dr_id in assignments.items():
        dr_routes[dr_id].append(dp_id)
    delivered_csp, energy_csp, violations_csp, avg_wait_csp = evaluate_solution(dr_routes, graph)
    perc_csp = delivered_csp / m_deliveries * 100
    print(f"CSP: teslimat %{perc_csp:.2f}, enerji {energy_csp:.2f}, süre {t_csp:.2f}s, "
          f"zaman pencere ihlali {violations_csp}, ort. bekleme {avg_wait_csp:.2f}dk")

    # GA
    start = time.time()
    ga = GeneticAlgorithm(graph)
    best_ind, best_fit = ga.run()
    t_ga = time.time() - start
    delivered_ga, energy_ga, violations_ga, avg_wait_ga = evaluate_solution(best_ind, graph)
    perc_ga = delivered_ga / m_deliveries * 100
    print(f"GA: teslimat %{perc_ga:.2f}, enerji {energy_ga:.2f}, süre {t_ga:.2f}s, "
          f"zaman pencere ihlali {violations_ga}, ort. bekleme {avg_wait_ga:.2f}dk")

    # Görselleştirme (CSP)
    fig, ax = plt.subplots()
    # Teslimat noktaları
    xs = [dp.pos[0] for dp in deliveries]
    ys = [dp.pos[1] for dp in deliveries]
    ax.scatter(xs, ys, c='blue', label='Teslimat Noktaları')
    # Drone başlangıçları
    xs = [dr.start_pos[0] for dr in drones]
    ys = [dr.start_pos[1] for dr in drones]
    ax.scatter(xs, ys, c='green', marker='^', label='Dronelar')
    # No-fly bölgeleri
    for zone in zones:
        poly = Polygon(zone.coordinates, closed=True, color='red', alpha=0.3)
        ax.add_patch(poly)
    # CSP rotaları
    for dp, dr in assignments.items():
        dr_pos = next(d.start_pos for d in drones if d.id == dr)
        dp_pos = next(d.pos for d in deliveries if d.id == dp)
        ax.plot([dr_pos[0], dp_pos[0]], [dr_pos[1], dp_pos[1]], c='green', linewidth=1)
    ax.set_title(f"{title} - CSP Rotası")
    ax.legend()
    plt.savefig(f"{title}_csp.png")
    plt.close(fig)
    print(f"{title} - Görselleştirme kaydedildi.")


if __name__ == "__main__":
    run_scenario(5, 20, 2, "Senaryo1")
    run_scenario(10, 50, 5, "Senaryo2") 