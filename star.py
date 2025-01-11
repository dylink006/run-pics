import folium
import osmnx as ox
import networkx as nx
from math import sin, cos, radians

# Define the coordinates for Gainesville, Florida
latitude, longitude = 29.6516, -82.3248

# Function to calculate star points
def generate_star_coordinates(lat, lon, size, points=5):
    coords = []
    for i in range(2 * points):
        angle = radians(360 * i / (2 * points))
        r = size if i % 2 == 0 else size / 2  # Alternate between long and short points
        x = lat + r * cos(angle) * 0.1  # Adjust scale for latitude
        y = lon + r * sin(angle) * 0.1  # Adjust scale for longitude
        coords.append((x, y))
    return coords

# Generate star coordinates
star_coords = generate_star_coordinates(latitude, longitude, size=0.2)

# Download road network for Gainesville
G = ox.graph_from_point((latitude, longitude), dist=5000, network_type='drive')

# Snap star points to nearest road nodes
snapped_points = [ox.nearest_nodes(G, coord[1], coord[0]) for coord in star_coords]

# Generate routes between snapped points
routes = []
for i in range(len(snapped_points) - 1):
    route = nx.shortest_path(G, snapped_points[i], snapped_points[i + 1], weight='length')
    routes.append(route)

# Close the loop by adding the route from the last point to the first point
closing_route = nx.shortest_path(G, snapped_points[-1], snapped_points[0], weight='length')
routes.append(closing_route)

# Combine all routes
all_routes = []
for route in routes:
    all_routes.extend(route)

# Create a map centered on Gainesville
m = folium.Map(location=[latitude, longitude], zoom_start=13)

# Plot each route on the map
for route in routes:
    route_coords = [(G.nodes[node]['y'], G.nodes[node]['x']) for node in route]
    folium.PolyLine(route_coords, color="red", weight=2.5).add_to(m)

# Save the map
m.save("gainesville_star_route.html")
print("Map saved as 'gainesville_star_route.html'. Open this file to view the map.")
