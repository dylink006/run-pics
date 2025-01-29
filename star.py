import osmnx as ox
import networkx as nx
import folium
from math import sin, cos, radians, sqrt
from global_land_mask import globe
from scipy.spatial.distance import directed_hausdorff

# Generate star points based on the size and center coordinates
def generate_star_points(center_lat, center_lon, size=0.02):
    points = []
    for i in range(10):
        angle = radians(i * 36)  # 36° increments for 10 points
        radius = size if i % 2 == 0 else size / 2
        lat = center_lat + radius * cos(angle)
        lon = center_lon + radius * sin(angle)
        points.append((lat, lon))
    return points

# Snap to the nearest node in the graph
def snap_to_nearest_node(G, lat, lon):
    try:
        return ox.nearest_nodes(G, lon, lat)
    except Exception:
        return None

# Improved water detection using global_land_mask
def is_point_over_water(lat, lon):
    return not globe.is_land(lat, lon)

# Distance calculation for a point to a line segment
def calculate_distance_to_line(point, line):
    x0, y0 = point
    (x1, y1), (x2, y2) = line
    num = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    denom = sqrt((y2 - y1)**2 + (x2 - x1)**2)
    return num / denom if denom != 0 else float("inf")

# Improved route generation to prioritize adherence to the star shape
def generate_star_route(G, snapped_star_points):
    final_route = []
    visited_nodes = set()

    for i in range(len(snapped_star_points)):
        p1 = snapped_star_points[i]
        p2 = snapped_star_points[(i + 1) % len(snapped_star_points)]  # Wrap around

        start_node = snap_to_nearest_node(G, p1[0], p1[1])
        end_node = snap_to_nearest_node(G, p2[0], p2[1])

        if start_node is None or end_node is None:
            continue

        if not nx.has_path(G, start_node, end_node):
            print(f"No path found between {start_node} and {end_node}. Skipping segment.")
            continue

        def custom_cost(u, v, d):
            road_length = d.get("length", 1)
            point = (G.nodes[v]["y"], G.nodes[v]["x"])
            deviation_penalty = calculate_distance_to_line(point, [p1, p2]) ** 2 * 50000
            loop_penalty = 70000 if v in visited_nodes else 0
            jaggedness_penalty = 50000 if len(final_route) > 2 and final_route[-2] == v else 0
            return road_length + deviation_penalty + loop_penalty + jaggedness_penalty

        try:
            path = nx.shortest_path(G, start_node, end_node, weight=custom_cost)
            for node in path:
                if node not in visited_nodes:
                    visited_nodes.add(node)
                    final_route.append(node)

        except nx.NetworkXNoPath:
            print(f"Failed to find a path from {start_node} to {end_node}.")
            continue

    return final_route

# Plot route on a map
def plot_route(G, route_nodes, snapped_star_points, out_file="star_route_map.html"):
    if not route_nodes or len(route_nodes) < 2:
        print("No route to plot.")
        return
    lat0 = G.nodes[route_nodes[0]]['y']
    lon0 = G.nodes[route_nodes[0]]['x']
    m = folium.Map(location=(lat0, lon0), zoom_start=13)

    for lat, lon in snapped_star_points:
        folium.CircleMarker(location=(lat, lon), radius=5, color="blue", fill=True).add_to(m)

    coords = [(G.nodes[n]['y'], G.nodes[n]['x']) for n in route_nodes]
    folium.PolyLine(coords, color="red", weight=3).add_to(m)
    m.save(out_file)
    print(f"Map saved to {out_file}")

# Improved scoring system with normalization
def calculate_star_score(G, route_nodes, ideal_star_segments, star_points, snapped_star_points, size):
    total_deviation = 0
    route_distance = 0
    symmetry_penalty = 0

    ideal_distance = sum(
        sqrt((seg[1][0] - seg[0][0]) ** 2 + (seg[1][1] - seg[0][1]) ** 2) for seg in ideal_star_segments
    ) / size  # Normalize by size

    for i in range(len(route_nodes) - 1):
        start_node = route_nodes[i]
        end_node = route_nodes[i + 1]
        segment_start = (G.nodes[start_node]["y"], G.nodes[start_node]["x"])
        segment_end = (G.nodes[end_node]["y"], G.nodes[end_node]["x"])

        route_distance += sqrt((segment_end[0] - segment_start[0]) ** 2 + (segment_end[1] - segment_start[1]) ** 2)

        for ideal_segment in ideal_star_segments:
            deviation_start = calculate_distance_to_line(segment_start, ideal_segment)
            deviation_end = calculate_distance_to_line(segment_end, ideal_segment)
            total_deviation += (deviation_start + deviation_end) / 2 / size  # Normalize by size

    avg_ideal_length = ideal_distance / len(ideal_star_segments)
    segment_lengths = [
        sqrt((seg[1][0] - seg[0][0]) ** 2 + (seg[1][1] - seg[0][1]) ** 2) / size for seg in ideal_star_segments
    ]
    symmetry_penalty = sum(abs(length - avg_ideal_length) for length in segment_lengths) * 10000

    # Shape similarity using normalized Hausdorff distance
    route_coords = [(G.nodes[node]["y"], G.nodes[node]["x"]) for node in route_nodes]
    ideal_coords = [(lat, lon) for lat, lon in star_points]
    hausdorff_distance = max(
        directed_hausdorff(route_coords, ideal_coords)[0],
        directed_hausdorff(ideal_coords, route_coords)[0]
    ) / size * 50000  # Normalize by size

    jaggedness_penalty = sum(
        50000 for i in range(2, len(route_nodes)) if route_nodes[i] == route_nodes[i - 2]
    )

    connectivity_penalty = 100000 if len(route_nodes) < len(snapped_star_points) else 0

    score = total_deviation * 2 + symmetry_penalty + abs(route_distance - ideal_distance) * 100 + hausdorff_distance + jaggedness_penalty + connectivity_penalty
    return score

# Main routine to evaluate configurations
def build_and_run_star_matrix(city="San Francisco, California"):
    latitude_offsets = [0]
    longitude_offsets = [0]
    sizes = [0.02]
    results = []

    for north_offset in latitude_offsets:
        for east_offset in longitude_offsets:
            for size in sizes:
                center_lat, center_lon = ox.geocode(city)
                center_lat += north_offset
                center_lon += east_offset
                G = ox.graph_from_point((center_lat, center_lon), dist=3000, network_type="drive")

                star_points = generate_star_points(center_lat, center_lon, size=size)
                snapped_star_points = []
                for lat, lon in star_points:
                    if is_point_over_water(lat, lon):
                        print(f"Skipping point ({lat}, {lon}) over water.")
                        continue
                    snapped_node = snap_to_nearest_node(G, lat, lon)
                    if snapped_node is not None:
                        snapped_star_points.append((G.nodes[snapped_node]["y"], G.nodes[snapped_node]["x"]))

                route_nodes = generate_star_route(G, snapped_star_points)
                if not route_nodes:
                    continue

                ideal_star_segments = [
                    (star_points[i], star_points[(i + 1) % len(star_points)])
                    for i in range(len(star_points))
                ]

                score = calculate_star_score(G, route_nodes, ideal_star_segments, star_points, snapped_star_points, size)
                results.append({
                    "north_offset": north_offset,
                    "east_offset": east_offset,
                    "size": size,
                    "score": score
                })

                # Plot the route if only one configuration is provided
                if len(latitude_offsets) == 1 and len(longitude_offsets) == 1 and len(sizes) == 1:
                    out_file = f"star_route_{north_offset}_{east_offset}_size_{size}.html"
                    plot_route(G, route_nodes, snapped_star_points, out_file=out_file)

    results = sorted(results, key=lambda x: x["score"])
    print("Ranking of star routes by adherence to the ideal star shape:")
    for rank, result in enumerate(results, 1):
        print(f"Rank {rank}: north_offset={result['north_offset']}, "
              f"east_offset={result['east_offset']}, size={result['size']}, score={result['score']}")
    return results

if __name__ == "__main__":
    build_and_run_star_matrix()
