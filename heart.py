import osmnx as ox
import networkx as nx
import folium
from math import sin, cos, radians, sqrt, atan2, pi, exp
from global_land_mask import globe
from scipy.spatial.distance import directed_hausdorff
import time
import math
from shapely.geometry import Polygon, LineString
from shapely.errors import TopologicalError
from shapely.affinity import scale
import gpxpy
import gpxpy.gpx
import numpy as np

# Generate heart points based on the size and center coordinates
def generate_heart_points(center_lat, center_lon, size=0.02):
    points = []
    # we only want a single cusp point at the top (i=0) and bottom (i=10),
    # so skip the two adjacent points on either side of each
    skip = {1, 19, 9, 11}
    for i in range(20):
        if i in skip:
            continue

        t = radians(i * 18)  # 20 pts around the heart

        # standard parametric heart
        x = 16 * sin(t)**3
        y = 13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t)

        # normalize & scale
        x_norm = x / 18 * size
        y_norm = y / 18 * size

        # latitude ← y, longitude ← x
        lat = center_lat + y_norm
        lon = center_lon + x_norm
        points.append((lat, lon))

    return points

# Improved water detection using global_land_mask
def is_point_over_water(lat, lon):
    return not globe.is_land(lat, lon)

# Find the nearest node in the graph
def snap_to_nearest_valid_node(G, lat, lon, max_attempts=50):
    """Find the nearest node that's on land and part of the road network."""
    try:
        # Use OSMnx's built-in function for efficiency
        nearest_nodes = ox.distance.nearest_nodes(G, 
                                              X=[lon], 
                                              Y=[lat],
                                              return_dist=True)
        
        # nearest_nodes returns (node_ids, distances)
        node_ids = nearest_nodes[0]
        distances = nearest_nodes[1]
        
        # If only one node is returned, wrap it in a list
        if not isinstance(node_ids, list):
            node_ids = [node_ids]
            distances = [distances]
        
        # Sort nodes by distance
        sorted_nodes = sorted(zip(node_ids, distances), key=lambda x: x[1])
        
        # Try the closest nodes first, but check if they're on land
        for node_id, distance in sorted_nodes[:max_attempts]:
            node_lat = G.nodes[node_id]['y']
            node_lon = G.nodes[node_id]['x']
            
            if not is_point_over_water(node_lat, node_lon):
                return node_id
    except Exception as e:
        print(f"Error finding nearest node: {str(e)}")
        
        # Fallback: manually find closest node
        nearest_node = None
        min_dist = float('inf')
        
        for node, data in G.nodes(data=True):
            node_point = (data['y'], data['x'])  # lat, lon
            dist = ((lat - node_point[0])**2 + (lon - node_point[1])**2)**0.5
            
            if dist < min_dist and not is_point_over_water(node_point[0], node_point[1]):
                min_dist = dist
                nearest_node = node
        
        return nearest_node
    
    # If we can't find a valid node, return None
    return None

# Distance calculation for a point to a line segment
def calculate_distance_to_line(point, line):
    x0, y0 = point
    (x1, y1), (x2, y2) = line
    
    # Line vector
    dx = x2 - x1
    dy = y2 - y1
    
    # Parametric value of closest point
    if dx == 0 and dy == 0:  # Line is a point
        return ((x0 - x1)**2 + (y0 - y1)**2)**0.5
    
    t = ((x0 - x1) * dx + (y0 - y1) * dy) / (dx**2 + dy**2)
    
    if t < 0:  # Closest to start point
        return ((x0 - x1)**2 + (y0 - y1)**2)**0.5
    elif t > 1:  # Closest to end point
        return ((x0 - x2)**2 + (y0 - y2)**2)**0.5
    else:  # Closest to middle of line
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        return ((x0 - closest_x)**2 + (y0 - closest_y)**2)**0.5

# Improved route generation to prioritize adherence to the road network
def generate_heart_route(G, snapped_heart_points, timeout=60):
    import time
    start_time = time.time()
    
    final_route = []
    visited_nodes = set()
    
    for i in range(len(snapped_heart_points)):
        if time.time() - start_time > timeout:
            return final_route
            
        p1 = snapped_heart_points[i]
        p2 = snapped_heart_points[(i + 1) % len(snapped_heart_points)]
        
        start_node = snap_to_nearest_valid_node(G, p1[0], p1[1])
        end_node = snap_to_nearest_valid_node(G, p2[0], p2[1])
        
        if start_node is None or end_node is None:
            continue
            
        if not nx.has_path(G, start_node, end_node):
            continue
        
        # First find a path that strictly follows the road network
        try:
            road_path = nx.shortest_path(G, start_node, end_node, weight="length")
            
            # Then try to find a path that balances road adherence with heart shape
            def custom_cost(u, v, d):
                # Base cost is the road length
                road_length = d.get("length", 1)
                
                # Calculate how far this point deviates from the ideal line
                point = (G.nodes[v]["y"], G.nodes[v]["x"])
                # Use a more modest penalty than the original
                deviation_penalty = calculate_distance_to_line(point, [p1, p2]) * 1000
                
                # Smaller penalties for loops and jaggedness
                loop_penalty = 1000 if v in visited_nodes else 0
                jaggedness_penalty = 500 if len(final_route) > 2 and final_route[-2] == v else 0
                
                # Ensure we heavily penalize highways or non-walkable roads
                highway_type = d.get("highway", "")
                road_penalty = 100000 if highway_type in ["motorway", "motorway_link", "trunk", "trunk_link"] else 0
                
                return road_length + deviation_penalty + loop_penalty + jaggedness_penalty + road_penalty
            
            # Try to find a balanced path with timeout protection
            balanced_path = None
            path_start_time = time.time()
            max_path_time = 10  # seconds
            
            try:
                if time.time() - path_start_time < max_path_time:
                    balanced_path = nx.shortest_path(G, start_node, end_node, weight=custom_cost)
                path = balanced_path if balanced_path else road_path
            except:
                path = road_path
            
            # Add the path to our route
            if i == 0:
                # For the first segment, add all nodes
                for node in path:
                    visited_nodes.add(node)
                    final_route.append(node)
            else:
                # For subsequent segments, skip the first node to avoid duplicates
                for node in path[1:]:
                    if node not in visited_nodes:
                        visited_nodes.add(node)
                        final_route.append(node)
                    
        except nx.NetworkXNoPath:
            continue
        except Exception as e:
            continue
    
    return final_route

# Plot route on a map to ensure it follows roads
def plot_route(G, route_nodes, heart_points, out_file="heart_route_map.html"):
    if not route_nodes or len(route_nodes) < 2:
        print("No route to plot.")
        return
    
    # Create a map centered on the route
    lat0 = G.nodes[route_nodes[0]]['y']
    lon0 = G.nodes[route_nodes[0]]['x']
    m = folium.Map(location=(lat0, lon0), zoom_start=14)
    
    # Plot the ideal heart points
    for lat, lon in heart_points:
        folium.CircleMarker(location=(lat, lon), radius=5, color="blue", fill=True, 
                          popup="Heart Point").add_to(m)
    
    # Plot the route ensuring it follows roads
    for i in range(len(route_nodes) - 1):
        u = route_nodes[i]
        v = route_nodes[i + 1]
        
        # Check if there's a direct edge between these nodes
        if G.has_edge(u, v):
            # Get the edge data
            edge_data = G.get_edge_data(u, v)
            
            # If the edge has geometry, use it to show the actual road shape
            geometry_exists = False
            for key, data in edge_data.items():
                if 'geometry' in data and data['geometry'] is not None:
                    coords = [(point[1], point[0]) for point in data['geometry'].coords]
                    folium.PolyLine(coords, color="red", weight=3).add_to(m)
                    geometry_exists = True
                    break
            
            # If no geometry, use a straight line between nodes
            if not geometry_exists:
                u_coords = (G.nodes[u]['y'], G.nodes[u]['x'])
                v_coords = (G.nodes[v]['y'], G.nodes[v]['x'])
                folium.PolyLine([u_coords, v_coords], color="red", weight=3).add_to(m)
        else:
            # If no direct edge, find and plot the shortest path
            try:
                path = nx.shortest_path(G, u, v, weight="length")
                # Plot each segment of the path
                for j in range(len(path) - 1):
                    n1, n2 = path[j], path[j+1]
                    n1_coords = (G.nodes[n1]['y'], G.nodes[n1]['x'])
                    n2_coords = (G.nodes[n2]['y'], G.nodes[n2]['x'])
                    folium.PolyLine([n1_coords, n2_coords], color="red", weight=3).add_to(m)
            except:
                # If path finding fails, use straight line with dashed style
                u_coords = (G.nodes[u]['y'], G.nodes[u]['x'])
                v_coords = (G.nodes[v]['y'], G.nodes[v]['x'])
                folium.PolyLine([u_coords, v_coords], color="red", weight=3, dash_array="5").add_to(m)
    
    # Add the road network as a background layer for context
    edges = ox.graph_to_gdfs(G, nodes=False, edges=True)
    folium.GeoJson(
        edges['geometry'],
        style_function=lambda x: {
            'color': 'gray',
            'weight': 1,
            'opacity': 0.5
        }
    ).add_to(m)
    
    m.save(out_file)
    print(f"Map saved to {out_file}")

def save_route_gpx(G, route_nodes, out_file):
    gpx = gpxpy.gpx.GPX()
    track = gpxpy.gpx.GPXTrack()
    gpx.tracks.append(track)
    segment = gpxpy.gpx.GPXTrackSegment()
    track.segments.append(segment)
    for node in route_nodes:
        lat = G.nodes[node]['y']
        lon = G.nodes[node]['x']
        segment.points.append(gpxpy.gpx.GPXTrackPoint(lat, lon))
    with open(out_file, 'w') as f:
        f.write(gpx.to_xml())
    print(f"GPX saved to {out_file}")

# Improved scoring system with stricter criteria for heart shape
def calculate_heart_score(G, route_nodes, heart_template_coords, size):
    if not route_nodes or len(route_nodes) < 3:
        return 0  # Invalid route gets zero score

    # Extract route coordinates
    route_coords = []
    for node in route_nodes:
        try:
            route_coords.append((G.nodes[node]["x"], G.nodes[node]["y"]))
        except KeyError:
            print(f"Missing coordinate for node {node}")
            return 0

    # Ensure route is closed
    if route_coords[0] != route_coords[-1]:
        route_coords.append(route_coords[0])

    try:
        # Convert to Shapely geometries
        route_poly = Polygon(route_coords).buffer(0)
        heart_poly = Polygon(heart_template_coords).buffer(0)
        route_line = LineString(route_coords)
        heart_line = LineString(heart_template_coords)

        # IoU
        if route_poly.is_valid and heart_poly.is_valid:
            inter_area = route_poly.intersection(heart_poly).area
            union_area = route_poly.union(heart_poly).area
            iou = inter_area / union_area if union_area > 0 else 0
        else:
            iou = 0

        # Hausdorff distance
        h_dist = max(
            directed_hausdorff(np.array(route_coords), np.array(heart_template_coords))[0],
            directed_hausdorff(np.array(heart_template_coords), np.array(route_coords))[0]
        )
        norm_hausdorff = h_dist / (size * 111000)  # scale to degrees -> km

        # Symmetry (overlap with mirror image)
        minx, _, maxx, _ = route_poly.bounds
        center_x = 0.5 * (minx + maxx)
        mirrored = scale(route_poly, xfact=-1, yfact=1, origin=(center_x, 0))
        if route_poly.area > 0:
            symmetry = mirrored.intersection(route_poly).area / route_poly.area
        else:
            symmetry = 0

        # Normalize metrics to [0, 1]
        S_iou = iou ** 2
        S_haus = math.exp(-6 * norm_hausdorff)
        S_sym = max(0, min(1, symmetry))

        # Composite score
        raw_score = S_iou * 0.5 + S_haus * 0.3 + S_sym * 0.2

        # Hard penalty for critical failure
        if S_iou < 0.3 or S_haus < 0.3 or S_sym < 0.5:
            raw_score *= 0.1

        return round(max(0, min(100, 100 * raw_score)), 1)

    except TopologicalError as e:
        print(f"Geometry error: {e}")
        return 0
    except Exception as e:
        print(f"Scoring error: {e}")
        return 0

# Visualize road snapping for debugging
def visualize_road_snapping(G, ideal_points, snapped_nodes, output_file="road_snapping.html"):
    """Visualize how heart points are snapped to the road network."""
    if not ideal_points:
        print("No ideal points to visualize")
        return
        
    m = folium.Map(location=[ideal_points[0][0], ideal_points[0][1]], zoom_start=14)
    
    # Add ideal heart points in blue
    for i, (lat, lon) in enumerate(ideal_points):
        folium.CircleMarker(
            location=(lat, lon),
            radius=5,
            color='blue',
            fill=True,
            popup=f"Ideal Heart Point {i+1}"
        ).add_to(m)
    
    # Add snapped road nodes in red
    for i, node in enumerate(snapped_nodes):
        if node is not None and i < len(ideal_points):
            node_lat = G.nodes[node]['y']
            node_lon = G.nodes[node]['x']
            folium.CircleMarker(
                location=(node_lat, node_lon),
                radius=5,
                color='red',
                fill=True,
                popup=f"Snapped Road Point {i+1}"
            ).add_to(m)
            
            # Draw line from ideal to snapped
            folium.PolyLine(
                locations=[(ideal_points[i][0], ideal_points[i][1]), (node_lat, node_lon)],
                color='orange',
                weight=2,
                dash_array='5'
            ).add_to(m)
    
    # Show the road network
    for u, v, data in G.edges(data=True):
        coords = [(G.nodes[u]['y'], G.nodes[u]['x']), (G.nodes[v]['y'], G.nodes[v]['x'])]
        folium.PolyLine(coords, color='gray', weight=1).add_to(m)
    
    m.save(output_file)
    print(f"Snapping visualization saved to {output_file}")

# Main routine to evaluate configurations
def build_and_run_heart_matrix(city="San Francisco, California"):
    """
    Generate heart-shaped routes for a given city with different configurations
    and rank them based on how well they follow the road network while maintaining
    a heart shape.
    """
    print(f"Generating heart routes for {city}...")
    import time
    
    # Configuration parameters to try
    latitude_offsets = [-0.02, -0.01, 0, 0.01, 0.02]
    longitude_offsets = [-0.02, -0.01, 0, 0.01, 0.02]
    sizes = [0.005, 0.01, 0.015, 0.02]  # Different heart sizes to try
    results = []

    for north_offset in latitude_offsets:
        for east_offset in longitude_offsets:
            for size in sizes:
                start_time = time.time()
                print(f"\nTrying configuration: north_offset={north_offset}, east_offset={east_offset}, size={size}")
                
                # Get the city center coordinates
                center_lat, center_lon = ox.geocode(city)
                center_lat += north_offset
                center_lon += east_offset
                
                # Calculate the maximum distance needed from center to any heart point (with a small buffer)
                max_heart_distance = size * 111000 * 1.2  # Convert to meters with 20% buffer
                
                # Use a walking/pedestrian network instead of driving
                print(f"Creating graph centered at ({center_lat}, {center_lon}) with radius {max_heart_distance}m...")
                G = ox.graph_from_point((center_lat, center_lon), dist=max_heart_distance, network_type="walk")
                
                print(f"Graph created with {len(G.nodes)} nodes and {len(G.edges)} edges in {time.time() - start_time:.2f} seconds")
                
                # Explicitly filter out highways and unwalkable roads
                edges_to_remove = []
                for u, v, key, data in G.edges(keys=True, data=True):
                    if "highway" in data and data["highway"] in ["motorway", "motorway_link", "trunk", "trunk_link"]:
                        edges_to_remove.append((u, v, key))
                
                for edge in edges_to_remove:
                    G.remove_edge(*edge)
                
                print(f"Removed {len(edges_to_remove)} highway edges")
                
                # Remove isolated nodes
                G = ox.utils_graph.remove_isolated_nodes(G)
                
                # Generate the ideal heart points
                heart_points = generate_heart_points(center_lat, center_lon, size=size)
                print(f"Generated {len(heart_points)} heart points")
                
                # Snap heart points to the road network
                snapped_heart_points = []
                snapped_nodes = []
                
                for lat, lon in heart_points:
                    if is_point_over_water(lat, lon):
                        print(f"Point ({lat}, {lon}) is over water, finding nearest land...")
                        # Move point toward center until it's on land (up to 20 attempts)
                        found_land = False
                        for j in range(1, 21):
                            # Move j/20 closer to center each time
                            adjusted_lat = lat - (lat - center_lat) * (j/20)
                            adjusted_lon = lon - (lon - center_lon) * (j/20)
                            if not is_point_over_water(adjusted_lat, adjusted_lon):
                                lat, lon = adjusted_lat, adjusted_lon
                                print(f"  Found land at ({lat}, {lon})")
                                found_land = True
                                break
                        
                        if not found_land:
                            print(f"  Could not find land, skipping point")
                            continue
                    
                    node = snap_to_nearest_valid_node(G, lat, lon)
                    if node is not None:
                        snapped_nodes.append(node)
                        snapped_heart_points.append((G.nodes[node]["y"], G.nodes[node]["x"]))
                
                print(f"Successfully snapped {len(snapped_heart_points)} points to road network")
                
                if len(snapped_heart_points) < 3:
                    print(f"Not enough valid points for size {size}. Skipping.")
                    continue
                
                # Create ideal heart segments for scoring
                ideal_heart_segments = [
                    (heart_points[i], heart_points[(i + 1) % len(heart_points)])
                    for i in range(len(heart_points))
                ]
                
                # Visualize the road snapping (for debugging)
                #visualize_road_snapping(G, heart_points, snapped_nodes, 
                                        #f"road_snapping_{north_offset}_{east_offset}_{size}.html")
                
                # Generate a route that follows the heart shape
                route_start_time = time.time()
                print("Starting route generation...")
                
                # Try to generate route with timeout protection
                try:
                    route_nodes = generate_heart_route(G, snapped_heart_points, timeout=120)
                    print(f"Route generation completed in {time.time() - route_start_time:.2f} seconds")
                except Exception as e:
                    print(f"Route generation failed: {str(e)}")
                    continue
                
                if not route_nodes or len(route_nodes) < 3:
                    print(f"Failed to generate a valid route for size {size}.")
                    continue
                
                # Score the route
                ideal_heart_coords = [(lon, lat) for lat, lon in heart_points]

                score = calculate_heart_score(G, route_nodes, ideal_heart_coords, size)
                print(f"Route scored {score} points")
                
                results.append({
                    "north_offset": north_offset,
                    "east_offset": east_offset,
                    "size": size,
                    "score": score,
                    "route_nodes": route_nodes,
                    "G": G,
                    "heart_points": heart_points
                })
                
                print(f"Total time for this configuration: {time.time() - start_time:.2f} seconds")
    
    # Sort results by score (higher is better)
    results = sorted(results, key=lambda x: x["score"], reverse=True)
    
    print("\nRanking of heart routes by adherence to the ideal heart shape:")
    for rank, result in enumerate(results[:], 1):
        if result['score'] > 82.0:
        
            print(f"Rank {rank}: north_offset={result['north_offset']}, "
              f"east_offset={result['east_offset']}, size={result['size']}, score={result['score']:.1f}")

            gpx_file = f"heart_route_rank{rank}_{result['north_offset']}_{result['east_offset']}_{result['size']}.gpx"
            save_route_gpx(result["G"], result["route_nodes"], out_file=gpx_file)
    
    return results

if __name__ == "__main__":
    build_and_run_heart_matrix("Hollywood, Florida")