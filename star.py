import osmnx as ox
import networkx as nx
import folium
from math import sin, cos, radians, sqrt
from shapely.geometry import LineString, Point


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


def calculate_closest_point_on_line(point, line):
    """
    Calculate the closest point on a line to a given point.
    """
    point = Point(point[1], point[0])  # Convert to (lon, lat) for Shapely
    line = LineString([(lon, lat) for lat, lon in line])  # Convert line to Shapely
    closest_point = line.interpolate(line.project(point))
    return closest_point.y, closest_point.x  # Convert back to (lat, lon)


def enforce_alignment(G, route, ideal_line, max_deviation=0.002):
    """
    Enforce alignment with the ideal line by correcting deviations beyond a threshold.
    """
    corrected_route = []
    for node in route:
        node_lat, node_lon = G.nodes[node]["y"], G.nodes[node]["x"]
        distance_to_line = calculate_distance_to_line((node_lat, node_lon), ideal_line)
        if distance_to_line > max_deviation:
            # Correct by snapping back to the closest point on the line
            closest_lat, closest_lon = calculate_closest_point_on_line((node_lat, node_lon), ideal_line)
            corrected_node = snap_to_nearest_node(G, closest_lat, closest_lon)
            corrected_route.append(corrected_node)
        else:
            corrected_route.append(node)
    return corrected_route


def generate_star_route(G, snapped_star_points, max_deviation=0.002):
    """
    Generate the star route by connecting snapped star points and enforcing alignment with the ideal star geometry.
    """
    final_route = []

    for i in range(len(snapped_star_points)):
        p1 = snapped_star_points[i]
        p2 = snapped_star_points[(i + 1) % len(snapped_star_points)]  # Wrap around

        # Snap start and end points to the road network
        start_node = snap_to_nearest_node(G, p1[0], p1[1])
        end_node = snap_to_nearest_node(G, p2[0], p2[1])

        # Define the ideal line segment
        ideal_line = [p1, p2]

        # Custom cost function: Penalize deviations from the ideal line
        def custom_cost(u, v, d):
            road_length = d.get("length", 1)
            point = (G.nodes[v]["y"], G.nodes[v]["x"])
            deviation_penalty = calculate_distance_to_line(point, ideal_line)
            return road_length + deviation_penalty * 100

        # Compute the shortest path
        try:
            path = nx.shortest_path(G, start_node, end_node, weight=custom_cost)
            # Enforce alignment with the ideal line
            corrected_path = enforce_alignment(G, path, ideal_line, max_deviation)
            final_route.extend(corrected_path)
        except nx.NetworkXNoPath:
            print(f"No path between nodes {start_node} and {end_node}")
            final_route.append(start_node)
            final_route.append(end_node)

    return final_route


def calculate_distance_to_line(point, line):
    """
    Calculate the perpendicular distance from a point to a line.
    """
    point = Point(point[1], point[0])  # Convert to (lon, lat) for Shapely
    line = LineString([(lon, lat) for lat, lon in line])  # Convert line to Shapely
    return point.distance(line)


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

    # Snap star points to road network (retrieve lat/lon of snapped nodes)
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
