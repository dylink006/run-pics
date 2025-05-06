import osmnx as ox
import networkx as nx
import folium
from math import sin, cos, radians, sqrt, atan2, pi, exp
from global_land_mask import globe
from scipy.spatial.distance import directed_hausdorff
import time
import math
from shapely.geometry import Polygon
from shapely.affinity import scale
import gpxpy
import gpxpy.gpx

# Generate star points based on the size and center coordinates
def generate_star_points(center_lat, center_lon, size=0.02):
    points = []
    
    # Generate initial star points
    for i in range(10):
        angle = radians(i * 36)  # 36° increments for 10 points
        radius = size if i % 2 == 0 else size / 2
        lat = center_lat + radius * cos(angle)
        lon = center_lon + radius * sin(angle)
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
def generate_star_route(G, snapped_star_points, timeout=60):
    import time
    start_time = time.time()
    
    final_route = []
    visited_nodes = set()
    
    for i in range(len(snapped_star_points)):
        if time.time() - start_time > timeout:
            return final_route
            
        p1 = snapped_star_points[i]
        p2 = snapped_star_points[(i + 1) % len(snapped_star_points)]
        
        start_node = snap_to_nearest_valid_node(G, p1[0], p1[1])
        end_node = snap_to_nearest_valid_node(G, p2[0], p2[1])
        
        if start_node is None or end_node is None:
            continue
            
        if not nx.has_path(G, start_node, end_node):
            continue
        
        # First find a path that strictly follows the road network
        try:
            road_path = nx.shortest_path(G, start_node, end_node, weight="length")
            
            # Then try to find a path that balances road adherence with star shape
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
def plot_route(G, route_nodes, star_points, out_file="star_route_map.html"):
    if not route_nodes or len(route_nodes) < 2:
        print("No route to plot.")
        return
    
    # Create a map centered on the route
    lat0 = G.nodes[route_nodes[0]]['y']
    lon0 = G.nodes[route_nodes[0]]['x']
    m = folium.Map(location=(lat0, lon0), zoom_start=14)
    
    # Plot the ideal star points
    for lat, lon in star_points:
        folium.CircleMarker(location=(lat, lon), radius=5, color="blue", fill=True, 
                          popup="Star Point").add_to(m)
    
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

# Improved scoring system with stricter criteria for star shape
def calculate_star_score(G, route_nodes, ideal_star_segments, star_points, snapped_star_points, size):
    if not route_nodes or len(route_nodes) < 3:
        return 0  # Invalid route gets zero score
    
    # Extract coordinates of the route
    route_coords_lon_lat = []
    for node in route_nodes:
        try:
            # OSMnx usually stores lon as 'x', lat as 'y'
            route_coords_lon_lat.append((G.nodes[node]["x"], G.nodes[node]["y"]))
        except KeyError:
            print(f"Warning: Node {node} missing coordinate data. Skipping.")
            return 0 # Cannot score if coordinate is missing
    
    # 1. Shape similarity (0-30 points) - Based on distance to ideal segments
    shape_deviation = 0
    for lon, lat in route_coords_lon_lat:
        point = (lon, lat) # Use (lon, lat) for distance calc if needed
        min_deviation = float('inf')
        for seg in ideal_star_segments:
            # Assuming calculate_distance_to_line expects ((lat1, lon1), (lat2, lon2))
            # Need to ensure segment format matches function expectation.
            # If calculate_distance_to_line expects (x,y), adapt point and seg accordingly.
            # Let's assume calculate_distance_to_line is robust or uses lat, lon:
            point_lat_lon = (lat, lon)
            seg_lat_lon = [(p[0], p[1]) for p in seg] # Ensure seg points are (lat, lon)
            try:
                deviation = calculate_distance_to_line(point_lat_lon, seg_lat_lon)
                min_deviation = min(min_deviation, deviation)
            except Exception as e:
                 print(f"Warning: Error in calculate_distance_to_line: {e}. Using large deviation.")
                 min_deviation = max(min_deviation, size * 5) # Penalize if calc fails


        shape_deviation += min_deviation

    avg_deviation = shape_deviation / len(route_coords_lon_lat) / size if len(route_coords_lon_lat) > 0 else 1
    # Less harsh exponential decay, adjusted weight
    shape_score = 30 * math.exp(-2 * avg_deviation) # Weight: 30 points

    # 2. Point coverage (0-15 points)
    points_reached = 0
    point_coverage_flags = []
    for i, ideal_point_lat_lon in enumerate(star_points): # star_points are (lat, lon)
        min_distance = float('inf')
        for route_lon, route_lat in route_coords_lon_lat:
            # Simple Euclidean distance (approximation, better to use haversine if needed)
            dist = math.sqrt((ideal_point_lat_lon[0] - route_lat)**2 + (ideal_point_lat_lon[1] - route_lon)**2)
            min_distance = min(min_distance, dist)

        if min_distance < size * 0.10: # Slightly wider threshold (10% of star size)
            points_reached += 1
            point_coverage_flags.append(1)
        else:
            point_coverage_flags.append(0)
    coverage_score = 15 * (points_reached / len(star_points)) # Weight: 15 points

    # 3. Star symmetry (0-15 points) - Using original logic, adjusted weight
    symmetry_score = 0
    # (Keep the original symmetry calculation logic here, just adjust the final scaling)
    point_angles = []
    center_lat = sum(p[0] for p in star_points) / len(star_points)
    center_lon = sum(p[1] for p in star_points) / len(star_points)

    for i, point_lat_lon in enumerate(star_points):
        if i < len(point_coverage_flags) and point_coverage_flags[i]:
             # Use atan2(y, x) which is atan2(lon - center_lon, lat - center_lat)
            angle = math.atan2(point_lat_lon[1] - center_lon, point_lat_lon[0] - center_lat)
            point_angles.append(angle)

    if len(point_angles) >= 5:
        point_angles.sort()
        angle_diffs = []
        for i in range(len(point_angles)):
            next_i = (i + 1) % len(point_angles)
            diff = point_angles[next_i] - point_angles[i]
            # Handle wrap-around using modulo arithmetic on angles
            diff = (diff + math.pi) % (2 * math.pi) - math.pi # Result in [-pi, pi]
            angle_diffs.append(abs(diff))

        if angle_diffs:
             mean_diff = sum(angle_diffs) / len(angle_diffs)
             std_dev = math.sqrt(sum((d - mean_diff)**2 for d in angle_diffs) / len(angle_diffs))
             max_allowed_std = math.pi / 5 # Allow ~36 degrees avg variance
             symmetry_score = 15 * max(0, (1 - (std_dev / max_allowed_std))) # Weight: 15 points

    # --- New Area and Shape (IoU) Score Components ---

    route_polygon = None
    ideal_star_polygon = None
    area_similarity = 0
    iou_score = 0

    try:
        # Ensure route is closed for polygon (add start point to end if not already)
        if route_coords_lon_lat[0] != route_coords_lon_lat[-1]:
            route_coords_lon_lat.append(route_coords_lon_lat[0])

        if len(route_coords_lon_lat) >= 3:
             # Attempt to create polygon, buffer(0) can fix some invalid geometries
            route_polygon = Polygon(route_coords_lon_lat).buffer(0)


        # Ideal star points need to be in (lon, lat) order for Shapely
        ideal_star_coords_lon_lat = [(p[1], p[0]) for p in star_points]
        if len(ideal_star_coords_lon_lat) >= 3:
            ideal_star_polygon = Polygon(ideal_star_coords_lon_lat).buffer(0)

        # 4. Area Similarity (0-20 points)
        if route_polygon and ideal_star_polygon and route_polygon.is_valid and ideal_star_polygon.is_valid and ideal_star_polygon.area > 0:
            route_area = route_polygon.area
            ideal_area = ideal_star_polygon.area
            # Score based on ratio difference, penalizing large deviations
            area_diff_ratio = abs(route_area - ideal_area) / ideal_area
            area_similarity = max(0, 1 - area_diff_ratio) # Linear decay
            area_score = 20 * area_similarity # Weight: 20 points
        else:
             area_score = 0

        # 5. Shape Similarity (IoU) (0-20 points)
        if route_polygon and ideal_star_polygon and route_polygon.is_valid and ideal_star_polygon.is_valid:
            intersection_area = route_polygon.intersection(ideal_star_polygon).area
            union_area = route_polygon.union(ideal_star_polygon).area
            if union_area > 0:
                iou = intersection_area / union_area
                iou_score = 20 * iou # Weight: 20 points
            else:
                iou_score = 0
        else:
             iou_score = 0

    except TopologicalError as e:
        print(f"Shapely TopologicalError during scoring: {e}. Assigning 0 for area/IoU.")
        area_score = 0
        iou_score = 0
    except Exception as e:
        print(f"Error during Shapely polygon processing: {e}. Assigning 0 for area/IoU.")
        area_score = 0
        iou_score = 0


    # --- Combine Scores (Total 100 points) ---
    final_score = shape_score + coverage_score + symmetry_score + area_score + iou_score

    # Optional: Apply penalty/cap for very poor IoU or low point coverage
    if iou_score < 5 or points_reached < 4: # If IoU < 0.25 (scaled) or < 4 points reached
        final_score = min(final_score, 50) # Cap score for fundamentally non-star shapes

    return round(final_score, 1)

# Visualize road snapping for debugging
def visualize_road_snapping(G, ideal_points, snapped_nodes, output_file="road_snapping.html"):
    """Visualize how star points are snapped to the road network."""
    if not ideal_points:
        print("No ideal points to visualize")
        return
        
    m = folium.Map(location=[ideal_points[0][0], ideal_points[0][1]], zoom_start=14)
    
    # Add ideal star points in blue
    for i, (lat, lon) in enumerate(ideal_points):
        folium.CircleMarker(
            location=(lat, lon),
            radius=5,
            color='blue',
            fill=True,
            popup=f"Ideal Star Point {i+1}"
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
def build_and_run_star_matrix(city="San Francisco, California"):
    """
    Generate star-shaped routes for a given city with different configurations
    and rank them based on how well they follow the road network while maintaining
    a star shape.
    """
    print(f"Generating star routes for {city}...")
    import time
    
    # Configuration parameters to try
    latitude_offsets = [-0.01, 0, 0.01]
    longitude_offsets = [-0.01, 0, 0.01]
    sizes = [0.005, 0.01, 0.015]  # Different star sizes to try
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
                
                # Calculate the maximum distance needed from center to any star point (with a small buffer)
                max_star_distance = size * 111000 * 1.2  # Convert to meters with 20% buffer
                
                # Use a walking/pedestrian network instead of driving
                print(f"Creating graph centered at ({center_lat}, {center_lon}) with radius {max_star_distance}m...")
                G = ox.graph_from_point((center_lat, center_lon), dist=max_star_distance, network_type="walk")
                
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
                
                # Generate the ideal star points
                star_points = generate_star_points(center_lat, center_lon, size=size)
                print(f"Generated {len(star_points)} star points")
                
                # Snap star points to the road network
                snapped_star_points = []
                snapped_nodes = []
                
                for lat, lon in star_points:
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
                        snapped_star_points.append((G.nodes[node]["y"], G.nodes[node]["x"]))
                
                print(f"Successfully snapped {len(snapped_star_points)} points to road network")
                
                if len(snapped_star_points) < 3:
                    print(f"Not enough valid points for size {size}. Skipping.")
                    continue
                
                # Create ideal star segments for scoring
                ideal_star_segments = [
                    (star_points[i], star_points[(i + 1) % len(star_points)])
                    for i in range(len(star_points))
                ]
                
                # Visualize the road snapping (for debugging)
                #visualize_road_snapping(G, star_points, snapped_nodes, 
                                        #f"road_snapping_{north_offset}_{east_offset}_{size}.html")
                
                # Generate a route that follows the star shape
                route_start_time = time.time()
                print("Starting route generation...")
                
                # Try to generate route with timeout protection
                try:
                    route_nodes = generate_star_route(G, snapped_star_points, timeout=120)
                    print(f"Route generation completed in {time.time() - route_start_time:.2f} seconds")
                except Exception as e:
                    print(f"Route generation failed: {str(e)}")
                    continue
                
                if not route_nodes or len(route_nodes) < 3:
                    print(f"Failed to generate a valid route for size {size}.")
                    continue
                
                # Score the route
                score = calculate_star_score(G, route_nodes, ideal_star_segments, star_points, snapped_star_points, size)
                print(f"Route scored {score} points")
                
                results.append({
                    "north_offset": north_offset,
                    "east_offset": east_offset,
                    "size": size,
                    "score": score,
                    "route_nodes": route_nodes,
                    "G": G,
                    "star_points": star_points
                })
                
                print(f"Total time for this configuration: {time.time() - start_time:.2f} seconds")
    
    # Sort results by score (higher is better)
    results = sorted(results, key=lambda x: x["score"], reverse=True)
    
    print("\nRanking of star routes by adherence to the ideal star shape:")
    for rank, result in enumerate(results[:], 1):
        print(f"Rank {rank}: north_offset={result['north_offset']}, "
              f"east_offset={result['east_offset']}, size={result['size']}, score={result['score']:.1f}")
        
        out_file = f"star_route_rank{rank}_{result['north_offset']}_{result['east_offset']}_{result['size']}.html"
        plot_route(result["G"], result["route_nodes"], result["star_points"], out_file=out_file)
        print(f"Route map saved to {out_file}")

        gpx_file = f"star_route_rank{rank}_{result['north_offset']}_{result['east_offset']}_{result['size']}.gpx"
        save_route_gpx(result["G"], result["route_nodes"], out_file=gpx_file)
    
    return results

if __name__ == "__main__":
    build_and_run_star_matrix()
