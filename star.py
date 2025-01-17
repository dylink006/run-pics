import osmnx as ox
import networkx as nx
import folium
from math import sin, cos, radians, sqrt
from global_land_mask import globe

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
    return ox.nearest_nodes(G, lon, lat)

# Improved water detection using global_land_mask
def is_point_over_water(lat, lon):
    return not globe.is_land(lat, lon)

# Distance calculation for a point to a line segment
def calculate_distance_to_line(point, line):
    x0, y0 = point
    (x1, y1), (x2, y2) = line
    num = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2*y1 - y2*x1)
    denom = sqrt((y2 - y1)**2 + (x2 - x1)**2)
    return num / denom if denom != 0 else float("inf")

# Helper to detect unnecessary excursions ('hairs')
def detect_hairs(route):
    hairs = 0
    for i in range(len(route) - 2):
        if route[i] == route[i + 2]:
            hairs += 1
    return hairs

# Determine if a point is in the interior of the star
def point_in_star_interior(point, star_points):
    center_lat = sum(lat for lat, lon in star_points) / len(star_points)
    center_lon = sum(lon for lat, lon in star_points) / len(star_points)
    dist_to_center = sqrt((point[0] - center_lat) ** 2 + (point[1] - center_lon) ** 2)
    avg_dist_to_star_points = sum(
        sqrt((center_lat - lat) ** 2 + (center_lon - lon) ** 2) for lat, lon in star_points
    ) / len(star_points)
    return dist_to_center < avg_dist_to_star_points * 0.5  # Adjust threshold as needed

# Improved route generation to prioritize adherence to the star shape
def generate_star_route(G, snapped_star_points):
    final_route = []

    for i in range(len(snapped_star_points)):
        p1 = snapped_star_points[i]
        p2 = snapped_star_points[(i + 1) % len(snapped_star_points)]  # Wrap around

        start_node = snap_to_nearest_node(G, p1[0], p1[1])
        end_node = snap_to_nearest_node(G, p2[0], p2[1])

        def custom_cost(u, v, d):
            road_length = d.get("length", 1)
            point = (G.nodes[v]["y"], G.nodes[v]["x"])
            previous_point = (G.nodes[u]["y"], G.nodes[u]["x"])

            # Calculate the deviation from the ideal line segment
            deviation_penalty = calculate_distance_to_line(point, [p1, p2]) ** 2 * 10000

            # Add strong penalty for deviations that lead to the star's interior
            interior_penalty = 50000 if point_in_star_interior(point, snapped_star_points) else 0

            # Encourage direct movement toward the next star point
            direction_penalty = sqrt((p2[0] - point[0]) ** 2 + (p2[1] - point[1]) ** 2) * 2000

            # Penalize redundant movements and zig-zags
            zig_zag_penalty = calculate_distance_to_line(previous_point, [p1, p2]) * 3000

            # Add penalty for returning to the same node or close loops
            loop_penalty = 50000 if u == v else 0

            # Add a penalty for excursions that deviate and return to the same segment
            excursion_penalty = 25000 if calculate_distance_to_line(point, [p1, p2]) > 0.002 else 0

            # Overall cost combining length and penalties
            cost = road_length + deviation_penalty + interior_penalty + direction_penalty + zig_zag_penalty + loop_penalty + excursion_penalty

            return cost

        try:
            path = nx.shortest_path(G, start_node, end_node, weight=custom_cost)
            final_route.extend(path)
        except nx.NetworkXNoPath:
            print(f"No path between nodes {start_node} and {end_node}")
            return []  # Abort if any segment fails

    return final_route



# Calculate total deviation adjusted for star size and interior penalties
def calculate_total_deviation_adjusted(G, route_nodes, ideal_star_segments, size, star_points):
    total_deviation = 0
    interior_penalty = 0
    continuity_penalty = 0
    route_distance = 0
    ideal_distance = sum(
        sqrt((seg[1][0] - seg[0][0]) ** 2 + (seg[1][1] - seg[0][1]) ** 2) for seg in ideal_star_segments
    )

    for i in range(len(route_nodes) - 1):
        start_node = route_nodes[i]
        end_node = route_nodes[i + 1]
        segment_start = (G.nodes[start_node]["y"], G.nodes[start_node]["x"])
        segment_end = (G.nodes[end_node]["y"], G.nodes[end_node]["x"])
        route_distance += sqrt((segment_end[0] - segment_start[0]) ** 2 + (segment_end[1] - segment_start[1]) ** 2)

        for ideal_segment in ideal_star_segments:
            deviation_start = calculate_distance_to_line(segment_start, ideal_segment)
            deviation_end = calculate_distance_to_line(segment_end, ideal_segment)
            total_deviation += (deviation_start + deviation_end) / 2

        if point_in_star_interior(segment_start, star_points) or point_in_star_interior(segment_end, star_points):
            interior_penalty += 1

    continuity_penalty = len(route_nodes) / len(ideal_star_segments) * 500
    size_factor = 1 / size
    distance_penalty = abs(route_distance - ideal_distance) * 500
    hairs_penalty = detect_hairs(route_nodes) * 1000

    return (total_deviation * size_factor +
            interior_penalty * 2000 +
            distance_penalty +
            continuity_penalty +
            hairs_penalty)

# Plot route on a map
def plot_route(G, route_nodes, snapped_star_points, out_file="star_route_map.html"):
    if not route_nodes or len(route_nodes) < 2:
        print("No route to plot.")
        return
    lat0 = G.nodes[route_nodes[0]]["y"]
    lon0 = G.nodes[route_nodes[0]]["x"]
    m = folium.Map(location=(lat0, lon0), zoom_start=13)

    for lat, lon in snapped_star_points:
        folium.CircleMarker(location=(lat, lon), radius=5, color="blue", fill=True).add_to(m)

    coords = [(G.nodes[n]["y"], G.nodes[n]["x"]) for n in route_nodes]
    folium.PolyLine(coords, color="red", weight=3).add_to(m)
    m.save(out_file)
    print(f"Map saved to {out_file}")

# Main routine to evaluate configurations
def build_and_run_star_matrix(city="San Francisco, California"):
    latitude_offsets = [0.01]
    longitude_offsets = [-0.01]
    sizes = [0.01]  # Different star sizes to test
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
                skip_route = False
                for lat, lon in star_points:
                    if is_point_over_water(lat, lon):
                        print(f"Skipping configuration due to star point over water at ({lat}, {lon}).")
                        skip_route = True
                        break

                    snapped_node = snap_to_nearest_node(G, lat, lon)
                    snapped_lat = G.nodes[snapped_node]["y"]
                    snapped_lon = G.nodes[snapped_node]["x"]
                    snapped_star_points.append((snapped_lat, snapped_lon))

                if skip_route:
                    continue

                route_nodes = generate_star_route(G, snapped_star_points)
                if not route_nodes:
                    continue

                ideal_star_segments = [
                    (star_points[i], star_points[(i + 1) % len(star_points)])
                    for i in range(len(star_points))
                ]
                total_deviation_adjusted = calculate_total_deviation_adjusted(
                    G, route_nodes, ideal_star_segments, size, star_points
                )

                results.append({
                    "north_offset": north_offset,
                    "east_offset": east_offset,
                    "size": size,
                    "adjusted_total_deviation": total_deviation_adjusted
                })

                # Optionally plot single configuration maps
                if len(latitude_offsets) == 1 and len(longitude_offsets) == 1 and len(sizes) == 1:
                    out_file = f"star_route_{north_offset}_{east_offset}_size_{size}.html"
                    plot_route(G, route_nodes, snapped_star_points, out_file=out_file)

    results = sorted(results, key=lambda x: x["adjusted_total_deviation"])
    print("Ranking of star routes by adherence to the ideal star shape:")
    for rank, result in enumerate(results, 1):
        print(f"Rank {rank}: north_offset={result['north_offset']}, "
              f"east_offset={result['east_offset']}, size={result['size']}, "
              f"adjusted_total_deviation={result['adjusted_total_deviation']}")
    return results

if __name__ == "__main__":
    results = build_and_run_star_matrix("San Francisco, California")