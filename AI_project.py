import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import time

graph = None

# MAP & GRAPH FUNCTIONS
def get_map_data(city_name):
    place_name = city_name + ", France"
    global graph
    graph = ox.graph_from_place(place_name, network_type='drive')
    return graph

def get_node_names(graph):
    node_names = {}
    for node in graph.nodes():
        node_names[node] = graph.nodes[node].get('name', f"Unnamed Node {node}")
    return node_names

def update_nodes(event=None):
    selected_city = combobox_city.get()
    graph = get_map_data(selected_city)
    listbox_source.delete(0, tk.END)
    listbox_target.delete(0, tk.END)
    node_names = get_node_names(graph)
    for node in node_names:
        display = f"{node_names[node]} [{node}]"
        listbox_source.insert(tk.END, display)
        listbox_target.insert(tk.END, display)

def start_execution_timer():
    return time.perf_counter()


def end_execution_timer(start_timer):
    end_time = time.perf_counter()
    elapsed = end_time - start_timer
    return elapsed

# search algorithmes

# BFS
def bfs_search(graph, start, goal):
    t0 = start_execution_timer()
    from collections import deque
    visited = set()
    queue = deque([[start]])
    while queue:
        path = queue.popleft()
        node = path[-1]
        if node == goal:
            elapsed = end_execution_timer(t0)
            return path, elapsed
        if node not in visited:
            visited.add(node)
            for neighbor in graph.neighbors(node):
                new_path = list(path)
                new_path.append(neighbor)
                queue.append(new_path)
    elapsed = end_execution_timer(t0)
    return None, elapsed

# DFS
def dfs_search(graph, start, goal):
    t0 = start_execution_timer()
    stack = [[start]]
    visited = set()
    while stack:
        path = stack.pop()
        node = path[-1]
        if node == goal:
            elapsed = end_execution_timer(t0)
            return path, elapsed
        if node not in visited:
            visited.add(node)
            for neighbor in graph.neighbors(node):
                new_path = list(path)
                new_path.append(neighbor)
                stack.append(new_path)
    elapsed = end_execution_timer(t0)
    return None, elapsed

# IDS
def iterative_deepening(graph, start, goal, max_depth=1000):
    t0 = start_execution_timer()
    def dls(node, goal, depth, path, visited):
        if depth == 0 and node == goal:
            return path
        if depth > 0:
            for neighbor in graph.neighbors(node):
                if neighbor not in visited:
                    visited.add(neighbor)
                    result = dls(neighbor, goal, depth - 1, path + [neighbor], visited)
                    if result is not None:
                        return result
        return None

    for depth in range(max_depth):
        visited = {start}
        result = dls(start, goal, depth, [start], visited)
        if result is not None:
            path = result
            elapsed = end_execution_timer(t0)
            return path, elapsed
    elapsed = end_execution_timer(t0)
    return None, elapsed

# A* Search
def a_star_search(graph, source, target):
    t0 = start_execution_timer()
    return nx.astar_path(graph, source, target, weight='length'), end_execution_timer(t0)

# IDA*
def ida_star_search(graph, start, goal):
    import math
    t0 = start_execution_timer()
    
    def heuristic(u, v):
        x1, y1 = graph.nodes[u]['x'], graph.nodes[u]['y']
        x2, y2 = graph.nodes[v]['x'], graph.nodes[v]['y']
        return math.hypot(x2 - x1, y2 - y1)

    def search(path, g, bound):
        node = path[-1]
        f = g + heuristic(node, goal)
        if f > bound:
            return f
        if node == goal:
            return "FOUND"
        minimum = float('inf')
        for neighbor in graph.neighbors(node):
            if neighbor not in path:
                t = search(path + [neighbor], g + graph[node][neighbor][0].get('length', 1), bound)
                if t == "FOUND":
                    return "FOUND"
                if t < minimum:
                    minimum = t
        return minimum

    bound = heuristic(start, goal)
    path = [start]
    while True:
        t = search(path, 0, bound)
        if t == "FOUND":
            elapsed = end_execution_timer(t0)
            return path, elapsed
        if t == float('inf'):
            elapsed = end_execution_timer(t0)
            return None, elapsed
        bound = t

# VISUALIZATION
def plot_shortest_path(graph, shortest_path):
    fig, ax = ox.plot_graph_route(
        graph, shortest_path,
        route_color='g', route_linewidth=3, node_size=0,
        figsize=(15, 15), show=False, close=False
    )
    plt.tight_layout()
    plt.axis('off')
    plt.show(block=False)


def compute_path_metrics(G, path, default_speed_kph=30.0):
    if not path:
        return 0, 0.0, 0.0
    hop_count = len(path)
    total_distance = 0.0
    total_time_s = 0.0
    for u, v in zip(path[:-1], path[1:]):
        data = G.get_edge_data(u, v)
        if not data:
            continue
        best_edge = None
        best_len = float('inf')
        for key, ed in data.items():
            length = ed.get('length', 0.0)
            if length and length < best_len:
                best_len = length
                best_edge = ed
        if best_edge is None:
            best_edge = next(iter(data.values()))
            best_len = best_edge.get('length', 0.0)

        length = best_len or 0.0
        total_distance += length

        speed_kph = None
        if isinstance(best_edge.get('speed_kph'), (int, float)):
            speed_kph = best_edge.get('speed_kph')
        elif best_edge.get('maxspeed'):
            try:
                speed_kph = float(str(best_edge.get('maxspeed')).split()[0])
            except Exception:
                speed_kph = None

        if not speed_kph:
            speed_kph = default_speed_kph

        speed_ms = speed_kph * 1000.0 / 3600.0 if speed_kph > 0 else None
        if speed_ms:
            total_time_s += (length / speed_ms)

    return hop_count, total_distance, total_time_s

#Logic
def main():
    selected_source = listbox_source.get(tk.ACTIVE)
    selected_target = listbox_target.get(tk.ACTIVE)
    selected_algorithm = combobox_algorithm.get()

    if not (selected_source and selected_target):
        messagebox.showwarning("Warning", "Please select both source and target nodes.")
        return

    import re
    def parse_selected(val):
        if isinstance(val, int):
            return val
        s = str(val)
        m = re.search(r"\[(\d+)\]$", s)
        if m:
            return int(m.group(1))
        m = re.search(r"(\d+)$", s)
        if m:
            return int(m.group(1))
        try:
            return list(get_node_names(graph).keys())[0]
        except Exception:
            return None

    source = parse_selected(selected_source)
    target = parse_selected(selected_target)

    messagebox.showinfo("Info", f"Running {selected_algorithm}...")

    try:
   
        if selected_algorithm == "BFS":
            path, elapsed = bfs_search(graph, source, target)
        elif selected_algorithm == "DFS":
            path, elapsed = dfs_search(graph, source, target)
        elif selected_algorithm == "Iterative Deepening":
            path, elapsed = iterative_deepening(graph, source, target)
        elif selected_algorithm == "A*":
            path, elapsed = a_star_search(graph, source, target)
        elif selected_algorithm == "IDA*":
            path, elapsed = ida_star_search(graph, source, target)
        else:
            messagebox.showwarning("Attention!", "Selectionnez un algorithme valide.")
            return

        if path:
            plot_shortest_path(graph, path)
            messagebox.showinfo("Success",
                                f"Temps d'exécution: {elapsed:.6f} s\nLongueur du chemin: {len(path)} nœuds")
        else:
            messagebox.showerror("Erreur", f"Pas de chemin trouvé! (Elapsed: {elapsed:.6f} s)")
    except Exception as e:
        messagebox.showerror("Erreur", str(e))

# INTERFACE
root = tk.Tk()
root.title("Shortest Path Finder - France")
root.geometry("480x500")

frame_input = ttk.Frame(root, padding="10")
frame_input.pack(fill='both', expand=True)
frame_buttons = ttk.Frame(root, padding="10")
frame_buttons.pack(fill='x')

label_city = ttk.Label(frame_input, text="Select City:")
label_city.pack(pady=5)
cities = [
    "Paris", "Marseille", "Lyon", "Toulouse", "Nice", "Nantes", "Strasbourg",
    "Montpellier", "Bordeaux", "Lille", "Rennes", "Reims", "Le Havre", "Saint-Étienne",
    "Toulon", "Grenoble", "Dijon", "Angers", "Nîmes", "Clermont-Ferrand",
    "Le Mans", "Aix-en-Provence", "Brest", "Tours", "Amiens", "Limoges",
    "Perpignan", "Metz", "Besançon", "Orléans", "Mulhouse", "Rouen",
    "Caen", "Nancy", "Avignon"
]
combobox_city = ttk.Combobox(frame_input, values=cities)
combobox_city.pack(pady=5)
combobox_city.bind("<<ComboboxSelected>>", update_nodes)

label_algo = ttk.Label(frame_input, text="Select Algorithm:")
label_algo.pack(pady=5)
algorithms = ["BFS", "DFS", "Iterative Deepening", "A*", "IDA*"]
combobox_algorithm = ttk.Combobox(frame_input, values=algorithms)
combobox_algorithm.pack(pady=5)

label_source = ttk.Label(frame_input, text="Source Node:")
label_source.pack(pady=5)
listbox_source = tk.Listbox(frame_input, height=5)
listbox_source.pack(fill='x', padx=5, pady=5)

label_target = ttk.Label(frame_input, text="Destination Node:")
label_target.pack(pady=5)
listbox_target = tk.Listbox(frame_input, height=5)
listbox_target.pack(fill='x', padx=5, pady=5)

btn_find_path = ttk.Button(frame_buttons, text="Find Shortest Path", command=main)
btn_find_path.pack(fill='x', padx=5, pady=5)

root.mainloop()

