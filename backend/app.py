from flask import Flask, request, jsonify, render_template
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import requests
import os
from dotenv import load_dotenv

load_dotenv()

app = Flask(__name__)
customers = []

GOOGLE_API_KEY = os.environ.get("GOOGLE_API_KEY")
print(GOOGLE_API_KEY)

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

@app.route("/get_customers", methods=["GET"])
def get_customers():
    return jsonify(customers)

@app.route("/clear_customers", methods=["POST"])
def clear_customers():
    customers.clear()
    return jsonify({"message": "All customers cleared."}), 200

@app.route("/remove_customers", methods=["POST"])
def remove_customers():
    data = request.get_json()
    to_remove = set((c['lat'], c['lng']) for c in data)
    global customers
    original_len = len(customers)
    customers = [c for c in customers if (c['lat'], c['lng']) not in to_remove]
    removed_count = original_len - len(customers)
    return jsonify({"message": f"{removed_count} customers removed."}), 200


@app.route("/optimize", methods=["POST"])
def optimize_routes():
    global customers  # we'll modify the list in place

    if len(customers) == 0:
        return jsonify({"error": "No customers provided"}), 400

    total_capacity = sum(v["capacity"] for v in vehicles)

    try:
        weights = [int(c["weight"]) if c["weight"] else 0 for c in customers]
    except ValueError as e:
        return jsonify({"error": f"Invalid weight value: {e}"}), 400

    total_weight = sum(weights)
    removed_customers = []

    if total_weight > total_capacity:
        sorted_customers = sorted(customers, key=lambda c: convert_time(c["deadline"]), reverse=True)
        allowed_customers = []
        current_weight = 0

        for c in sorted_customers[::-1]:
            w = int(c["weight"])
            if current_weight + w <= total_capacity:
                allowed_customers.append(c)
                current_weight += w

        removed_customers = [c for c in customers if c not in allowed_customers]
        customers = allowed_customers

    coords = [f"{c['lat']},{c['lng']}" for c in customers]
    depot = coords[0]
    locations = [depot] + coords
    dist_matrix = get_distance_matrix(locations)
    weights = [int(c["weight"]) for c in customers]
    deadlines = [convert_time(c["deadline"]) for c in customers]

    result = solve_vrp(dist_matrix, weights, deadlines)
    return jsonify({
        "routes": result["routes"],
        "removed_customers": removed_customers
    })


def get_distance_matrix(locations):
    import json
    url = "https://maps.googleapis.com/maps/api/distancematrix/json"
    matrix = []

    headers = {
        'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
    }

    for origin in locations:
        params = {
            "origins": origin,
            "destinations": "|".join(locations),
            "key": GOOGLE_API_KEY
        }

        res = requests.get(url, params=params, headers=headers).json()
        print(f"\nRequest from origin: {origin}")
        print("API response status:", res.get("status"))
        print("API full response:", json.dumps(res, indent=2))

        if res.get("status") != "OK":
            raise ValueError(f"Distance matrix API top-level error: {res.get('error_message', 'Unknown error')}")

        if "rows" not in res or not res["rows"]:
            raise ValueError(f"Distance matrix returned no rows for origin {origin}. Full response: {res}")

        elements = res["rows"][0].get("elements", [])
        if not elements:
            raise ValueError(f"No elements in response for origin {origin}. Full response: {res}")

        row = []
        for el in elements:
            if el.get("status") != "OK":
                print(f"Element error status: {el.get('status')}")
                row.append(9999999)
            else:
                row.append(el["duration"]["value"])
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

import requests

def test_connection():
    test_url = "https://maps.googleapis.com/maps/api/distancematrix/json?origins=21.1059361,79.0696657&destinations=21.1494952,79.066155&key=AIzaSyBQWLBpMBBwmGlDExp2Us-Eoc7rtIiKcAk"
    try:
        response = requests.get(test_url)
        print("Test connection response:", response.json())
    except Exception as e:
        print("Test connection failed:", e)

test_connection()
