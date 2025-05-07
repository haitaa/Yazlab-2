import streamlit as st
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from drone_routing.data_generator import generate_drones, generate_deliveries, generate_no_fly_zones
from drone_routing.graph import Graph
from drone_routing.csp import CSP
from drone_routing.ga import GeneticAlgorithm
from typing import Dict, List
import os
from datetime import datetime

st.set_page_config(page_title="Drone Rota Planlayıcı", layout="wide")

st.title("Drone Rota Planlayıcı")

# Yan menü parametreleri
n_drones = st.sidebar.number_input("Drone sayısı", min_value=1, max_value=50, value=5)
m_deliveries = st.sidebar.number_input("Teslimat sayısı", min_value=1, max_value=200, value=20)
k_zones = st.sidebar.number_input("No-Fly Zone sayısı", min_value=0, max_value=20, value=2)
area_size_x = st.sidebar.number_input("Alan genişliği (m)", min_value=100.0, value=1000.0)
area_size_y = st.sidebar.number_input("Alan yüksekliği (m)", min_value=100.0, value=1000.0)
run_csp = st.sidebar.checkbox("CSP ile çöz", value=True)
run_ga = st.sidebar.checkbox("GA ile çöz", value=True)
br = st.sidebar.button("Çalıştır")

# Log alanı oluştur
log_container = st.empty()

# Zaman pencere kontrolü fonksiyonu

def evaluate_solution_ui(individual: Dict[int, List[int]], graph: Graph):
    delivered = 0
    energy = 0.0
    violations = 0
    total_wait = 0.0
    earliest_start = min(graph._time_to_min(dp.time_window[0]) for dp in graph.deliveries)
    for dr in graph.drones:
        route = individual.get(dr.id, [])
        prev_pos = dr.start_pos
        current_time = earliest_start
        for dp_id in route:
            dp = next(d for d in graph.deliveries if d.id == dp_id)
            dist = graph._distance(prev_pos, dp.pos)
            travel_time = (dist / dr.speed) * (1/60)
            arrival_time = current_time + travel_time
            ws = graph._time_to_min(dp.time_window[0])
            we = graph._time_to_min(dp.time_window[1])
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

if br:
    # Run ID ve log dizini yönetimi
    RUN_COUNTER_FILE = "run_counter.txt"
    RUN_LOG_DIR = "run_logs"
    os.makedirs(RUN_LOG_DIR, exist_ok=True)
    # Run numarası oku/güncelle
    if os.path.exists(RUN_COUNTER_FILE):
        with open(RUN_COUNTER_FILE) as f:
            count = int(f.read().strip())
    else:
        count = 0
    count += 1
    with open(RUN_COUNTER_FILE, "w") as f:
        f.write(str(count))
    run_id = count
    # Per-run log dosyası oluştur
    log_file_path = os.path.join(RUN_LOG_DIR, f"{run_id:03d}.log")
    file_handle = open(log_file_path, "w")

    logs: List[str] = []
    def log(msg: str):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        line = f"[Run {run_id}] [{timestamp}] {msg}"
        logs.append(line)
        log_container.text("\n".join(logs))
        file_handle.write(line + "\n")
        file_handle.flush()

    params = f"Senaryo: {n_drones} dron, {m_deliveries} teslimat, {k_zones} no-fly zone"
    log(params)
    # Veri üretimi
    log("Veri üretiliyor...")
    drones = generate_drones(n_drones, (area_size_x, area_size_y))
    deliveries = generate_deliveries(m_deliveries, (area_size_x, area_size_y))
    zones = generate_no_fly_zones(k_zones, (area_size_x, area_size_y))
    graph = Graph(drones, deliveries, zones)

    results = {}
    # CSP çözümü
    if run_csp:
        log("CSP çözümü başlıyor...")
        start = time.time()
        csp = CSP(graph)
        assignments = csp.solve()
        t_csp = time.time() - start
        # drone bazlı rotalara dönüştür
        dr_routes = {dr.id: [] for dr in drones}
        for dp_id, dr_id in assignments.items():
            dr_routes[dr_id].append(dp_id)
        res = evaluate_solution_ui(dr_routes, graph)
        log(f"CSP: teslimat %{res[0]/m_deliveries*100:.2f}, enerji {res[1]:.2f}, iklal {res[2]}, ort. bekleme {res[3]:.2f}dk, süre {t_csp:.2f}s")
        results['CSP'] = dr_routes
    # GA çözümü
    if run_ga:
        log("GA çözümü başlıyor...")
        start = time.time()
        ga = GeneticAlgorithm(graph)
        best_ind, best_fit = ga.run()
        t_ga = time.time() - start
        res = evaluate_solution_ui(best_ind, graph)
        log(f"GA: teslimat %{res[0]/m_deliveries*100:.2f}, enerji {res[1]:.2f}, iklal {res[2]}, ort. bekleme {res[3]:.2f}dk, süre {t_ga:.2f}s")
        results['GA'] = best_ind

    # Görselleştirme
    for key, routes in results.items():
        st.subheader(f"{key} Rotası Görselleştirme")
        fig, ax = plt.subplots()
        xs = [dp.pos[0] for dp in deliveries]
        ys = [dp.pos[1] for dp in deliveries]
        ax.scatter(xs, ys, c='blue', label='Teslimat Noktaları')
        xs = [dr.start_pos[0] for dr in drones]
        ys = [dr.start_pos[1] for dr in drones]
        ax.scatter(xs, ys, c='green', marker='^', label='Dronelar')
        for zone in zones:
            poly = Polygon(zone.coordinates, closed=True, color='red', alpha=0.3)
            ax.add_patch(poly)
        for dr_id, route in routes.items():
            dr_pos = next(d.start_pos for d in drones if d.id == dr_id)
            for dp_id in route:
                dp_pos = next(d.pos for d in deliveries if d.id == dp_id)
                ax.plot([dr_pos[0], dp_pos[0]], [dr_pos[1], dp_pos[1]], label=f"Drone {dr_id}" if dp_id==route[0] else "", linewidth=1)
                dr_pos = dp_pos
        ax.legend()
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        st.pyplot(fig)
    # Log dosyasını kapat
    file_handle.close() 