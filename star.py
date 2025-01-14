import osmnx as ox
import networkx as nx
import folium
from math import sin, cos, radians, sqrt


def generate_star_points(center_lat, center_lon, size=0.02):
    """
    Create 10 points for a classic 5-point star:
      - 5 outer tips
      - 5 inner tips
    """
    points = []
    for i in range(10):
        angle = radians(i * 36)  # Alternate between outer and inner points
        radius = size if i % 2 == 0 else size / 2
        lat = center_lat + radius * cos(angle)
        lon = center_lon + radius * sin(angle)
        points.append((lat, lon))
    return points


def snap_to_nearest_node(G, lat, lon):
    """
    Snap a coordinate to the nearest node in the graph G.
    """
    return ox.nearest_nodes(G, lon, lat)


def calculate_distance_to_point(point, target):
    """
    Calculate the Euclidean distance from a point to a target.
    """
    x1, y1 = point
    x2, y2 = target
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def calculate_distance_to_line(point, line):
    """
    Calculate the perpendicular distance from a point to a line segment.
    """
    x0, y0 = point
    x1, y1 = line[0]
    x2, y2 = line[1]
    num = abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1)
    denom = sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
    return num / denom if denom != 0 else float("inf")


def generate_star_route(G, snapped_star_points):
    """
    Generate the star route by connecting snapped star points using the road network.
    """
    final_route = []

    for i in range(len(snapped_star_points)):
        p1 = snapped_star_points[i]
        p2 = snapped_star_points[(i + 1) % len(snapped_star_points)]  # Wrap around to form a closed loop

        # Snap start and end points to the road network
        start_node = snap_to_nearest_node(G, p1[0], p1[1])
        end_node = snap_to_nearest_node(G, p2[0], p2[1])

        # Custom cost function: Consider both the current ideal line and the next star point
        def custom_cost(u, v, d):
            road_length = d.get("length", 1)
            point = (G.nodes[v]["y"], G.nodes[v]["x"])
            # Distance to the ideal line for the current segment
            deviation_penalty = calculate_distance_to_line(point, [p1, p2])
            # Distance to the next star point
            target_penalty = calculate_distance_to_point(point, p2)
            # Avoid loops by penalizing already visited nodes
            loop_penalty = 0 if v not in final_route else 1000
            return road_length + deviation_penalty * 300 + target_penalty * 50 + loop_penalty

        # Compute shortest path using custom cost
        try:
            path = nx.shortest_path(G, start_node, end_node, weight=custom_cost)
            final_route.extend(path)
        except nx.NetworkXNoPath:
            print(f"No path between nodes {start_node} and {end_node}")
            final_route.append(start_node)
            final_route.append(end_node)

    return final_route


def plot_route(G, route_nodes, snapped_star_points, out_file="star_route_map.html"):
    """
    Plot the route on a Folium map.
    """
    if not route_nodes or len(route_nodes) < 2:
        print("No route to plot.")
        return

    # Initialize map at the first route node
    lat0 = G.nodes[route_nodes[0]]["y"]
    lon0 = G.nodes[route_nodes[0]]["x"]
    m = folium.Map(location=(lat0, lon0), zoom_start=13)

    # Plot snapped star points
    for lat, lon in snapped_star_points:
        folium.CircleMarker(location=(lat, lon), radius=5, color="blue", fill=True).add_to(m)

    # Draw route polyline
    coords = [(G.nodes[n]["y"], G.nodes[n]["x"]) for n in route_nodes]
    folium.PolyLine(coords, color="red", weight=3).add_to(m)

    m.save(out_file)
    print(f"Map saved to {out_file}")


def build_and_run_star(city="San Francisco, California"):
    """
    Build and execute the star route visualization.
    """
    # Get city center
    center_lat, center_lon = ox.geocode(city)

    # Build road graph
    G = ox.graph_from_point((center_lat, center_lon), dist=3000, network_type="drive")

    # Generate star points
    star_points = generate_star_points(center_lat, center_lon, size=0.02)

    # Snap star points to road network
    snapped_star_points = []
    for lat, lon in star_points:
        snapped_node = snap_to_nearest_node(G, lat, lon)
        snapped_lat, snapped_lon = G.nodes[snapped_node]["y"], G.nodes[snapped_node]["x"]
        snapped_star_points.append((snapped_lat, snapped_lon))

    # Generate star route
    route_nodes = generate_star_route(G, snapped_star_points)

    # Plot and save the map
    plot_route(G, route_nodes, snapped_star_points, "star_route_map.html")


# Run the script
if __name__ == "__main__":
    build_and_run_star("San Francisco, California")
