from flask import Flask, request, jsonify, render_template
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import requests
import os
from dotenv import load_dotenv

load_dotenv()

app = Flask(__name__)
customers = []

GOOGLE_API_KEY = os.environ.get("AIzaSyB0rX-Ev4J7r0pTiCmD-m5s5Cn8nZQZhZ4")

vehicles = [
    {"capacity": 1500},
    {"capacity": 1500},
    {"capacity": 1000},
    {"capacity": 750},
]

@app.route("/")
def home():
    return render_template("index.html")

@app.route("/add_customer", methods=["POST"])
def add_customer():
    data = request.json
    customers.append(data)
    return jsonify({"message": "Customer added"}), 200

@app.route("/optimize", methods=["POST"])
def optimize_routes():
    if len(customers) == 0:
        return jsonify({"error": "No customers provided"}), 400

    coords = [f"{c['lat']},{c['lng']}" for c in customers]
    depot = coords[0]
    locations = [depot] + coords
    dist_matrix = get_distance_matrix(locations)

    weights = [int(c["weight"]) for c in customers]
    deadlines = [convert_time(c["deadline"]) for c in customers]
    result = solve_vrp(dist_matrix, weights, deadlines)
    return jsonify(result)

def get_distance_matrix(locations):
    url = "https://maps.googleapis.com/maps/api/distancematrix/json"
    matrix = []
    for origin in locations:
        params = {
            "origins": origin,
            "destinations": "|".join(locations),
            "key": GOOGLE_API_KEY
        }
        res = requests.get(url, params=params).json()
        row = [el["duration"]["value"] for el in res["rows"][0]["elements"]]
        matrix.append(row)
    return matrix

def convert_time(deadline_str):
    from datetime import datetime
    deadline = datetime.strptime(deadline_str, "%H:%M")
    return deadline.hour * 3600 + deadline.minute * 60

def solve_vrp(distance_matrix, weights, deadlines):
    manager = pywrapcp.RoutingIndexManager(len(distance_matrix), len(vehicles), 0)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        return distance_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    def demand_callback(from_index):
        node = manager.IndexToNode(from_index)
        return weights[node - 1] if node != 0 else 0

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index, 0, [v["capacity"] for v in vehicles], True, "Capacity"
    )

    def time_callback(from_index, to_index):
        return distance_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

    time_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.AddDimension(
        time_callback_index, 30, 24 * 3600, False, "Time"
    )
    time_dimension = routing.GetDimensionOrDie("Time")

    for i, deadline in enumerate(deadlines, start=1):
        time_dimension.CumulVar(manager.NodeToIndex(i)).SetRange(0, deadline)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    solution = routing.SolveWithParameters(search_parameters)

    routes = []
    if solution:
        for vehicle_id in range(len(vehicles)):
            index = routing.Start(vehicle_id)
            route = []
            while not routing.IsEnd(index):
                node = manager.IndexToNode(index)
                if node != 0:
                    route.append(customers[node - 1]["name"])
                index = solution.Value(routing.NextVar(index))
            routes.append({"vehicle": vehicle_id + 1, "route": route})
    return {"routes": routes}
