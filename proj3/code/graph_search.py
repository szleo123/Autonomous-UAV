from heapq import heappush, heappop  # Recommended.
import heapq
import numpy as np

from flightsim.world import World

from .occupancy_map import OccupancyMap # Recommended.

def graph_search(world, resolution, margin, start, goal, astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        return a tuple (path, nodes_expanded)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        nodes_expanded, the number of nodes that have been expanded
    """

    # While not required, we have provided an occupancy map you may use or modify.
    occ_map = OccupancyMap(world, resolution, margin)
    # Retrieve the index in the occupancy grid matrix corresponding to a position in space.
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))
    dist = np.ones_like(occ_map.map) * np.inf
    parents = np.ones_like(occ_map.map) * (-1)
    dist[start_index] = 0
    if astar:
        dist[start_index] += 1.5*np.linalg.norm((goal - start))
    visited = set()
    # directions = [(1, 0, 0), (-1, 0, 0),
    #              (0, 1, 0), (0, -1, 0),
    #              (0, 0, 1), (0, 0, -1)]
    directions = []
    pose = [0, 1, -1]
    for i in pose:
        for j in pose:
            for k in pose:
                directions.append((i, j, k))
    if not "blocks" in world.world.keys():
        return np.array([start, goal]), 1

    path = None
    while True:
        num = np.argmin(dist, axis=None)
        cur = np.unravel_index(np.argmin(dist, axis=None), dist.shape)
        cost = dist[cur]
        if cost == np.inf:
            break
        dist[cur] = np.inf
        if cur == goal_index:
            path = []
            break
        visited.add(cur)
        for dir in directions:
            next_index = tuple(np.array(cur) + np.array(dir))
            if next_index in visited:
                continue
            if not occ_map.is_occupied_index(next_index):
                new_cost = cost + np.linalg.norm((np.array(dir) * resolution))
                if astar:
                    new_cost -= 1.5*np.linalg.norm(goal - occ_map.index_to_metric_center(cur))
                    new_cost += 1.5*np.linalg.norm(goal - occ_map.index_to_metric_center(next_index))
                if new_cost < dist[next_index]:
                    dist[next_index] = new_cost
                    parents[next_index] = num


    if path is not None:
        cur = goal_index
        while True:
            if cur == (0,0,1):
                break
            path.append(occ_map.index_to_metric_center(cur))
            temp = parents[cur]
            if temp == -1:
                break
            cur = np.unravel_index(temp, dist.shape)
        path.reverse()
        path[0] = start
        path[-1] = goal

        path = np.array(path)
    # Return a tuple (path, nodes_expanded)
    return path, len(visited)

