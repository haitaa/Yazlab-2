from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from typing import List, Tuple, Dict, Any
from drone_routing.models import Drone, DeliveryPoint, NoFlyZone
from drone_routing.graph import Graph
from drone_routing.csp import CSP
from drone_routing.ga import GeneticAlgorithm

app = FastAPI(
    title="Drone Rota Planlama Servisi",
    description="Dinamik ve gerçek-zamanlı rota planlama API'si",
    version="0.1"
)

# --- Şema tanımları ---
class DroneSchema(BaseModel):
    id: int
    max_weight: float
    battery: int
    speed: float
    start_pos: Tuple[float, float]

class DeliverySchema(BaseModel):
    id: int
    pos: Tuple[float, float]
    weight: float
    priority: int
    time_window: Tuple[str, str]

class NoFlyZoneSchema(BaseModel):
    id: int
    coordinates: List[Tuple[float, float]]
    active_time: Tuple[str, str]

class PlanRequest(BaseModel):
    drones: List[DroneSchema]
    deliveries: List[DeliverySchema]
    no_fly_zones: List[NoFlyZoneSchema]
    use_csp: bool = True
    use_ga: bool = False
    ga_params: Dict[str, Any] = {}

class PlanResponse(BaseModel):
    csp_assignment: Dict[int, int] = None
    ga_solution: Dict[int, List[int]] = None
    ga_fitness: float = None

# --- HTTP endpoint ---
@app.post("/plan", response_model=PlanResponse)
def plan_route(req: PlanRequest):
    # JSON -> dataclass dönüştürme
    drones = [Drone(**d.dict()) for d in req.drones]
    deliveries = [DeliveryPoint(**d.dict()) for d in req.deliveries]
    zones = [NoFlyZone(**z.dict()) for z in req.no_fly_zones]

    graph = Graph(drones, deliveries, zones)
    response = PlanResponse()
    if req.use_csp:
        csp = CSP(graph)
        response.csp_assignment = csp.solve()
    if req.use_ga:
        ga = GeneticAlgorithm(graph, **req.ga_params)
        sol, fit = ga.run()
        response.ga_solution = sol
        response.ga_fitness = fit
    return response

# --- WebSocket endpoint (dinamik güncellemeler) ---
@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    # Kısa ömürlü state
    drones: List[Drone] = []
    deliveries: List[DeliveryPoint] = []
    zones: List[NoFlyZone] = []
    graph: Graph = None
    try:
        while True:
            msg = await ws.receive_json()
            action = msg.get("action")
            payload = msg.get("payload", {})
            if action == "init":
                drones = [Drone(**d) for d in payload.get("drones", [])]
                deliveries = [DeliveryPoint(**d) for d in payload.get("deliveries", [])]
                zones = [NoFlyZone(**z) for z in payload.get("no_fly_zones", [])]
                graph = Graph(drones, deliveries, zones)
                await ws.send_json({"status":"initialized"})
            elif action == "update_no_fly":
                # yeni no-fly bölgeleri güncelle
                zones = [NoFlyZone(**z) for z in payload]
                graph.no_fly_zones = zones
                await ws.send_json({"status":"no_fly_zones_updated"})
            elif action == "new_delivery":
                dp = DeliveryPoint(**payload)
                deliveries.append(dp)
                graph.deliveries = deliveries
                graph.nodes = graph._init_nodes()
                graph.build_graph()
                await ws.send_json({"status":"delivery_added"})
            elif action == "replan":
                # yeniden planlama
                csp = CSP(graph)
                assign = csp.solve()
                ga = GeneticAlgorithm(graph, **payload.get("ga_params", {}))
                sol, fit = ga.run() if payload.get("use_ga") else ({}, None)
                await ws.send_json({
                    "csp_assignment": assign,
                    "ga_solution": sol,
                    "ga_fitness": fit
                })
            else:
                await ws.send_json({"error":"unknown_action"})
    except WebSocketDisconnect:
        return 