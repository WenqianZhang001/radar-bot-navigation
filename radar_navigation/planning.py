"""Offline four-connected A* with explicit footprint inflation and unknown policy."""
import heapq
import math


def astar(grid, start, goal, radius=.22, allow_unknown=False):
    values = grid.values()
    blocked = set()
    cells = math.ceil(radius/grid.resolution)
    for idx, value in enumerate(values):
        if value >= 65:
            x, y = idx % grid.width, idx // grid.width
            for dx in range(-cells, cells+1):
                for dy in range(-cells, cells+1):
                    if math.hypot(dx, dy)*grid.resolution <= radius:
                        blocked.add((x+dx, y+dy))
    def valid(p):
        idx = grid.index(p)
        return idx is not None and p not in blocked and (allow_unknown or values[idx] >= 0)
    if not valid(start) or not valid(goal):
        return []
    costs, parents, queue = {start: 0}, {}, [(0, start)]
    while queue:
        _, p = heapq.heappop(queue)
        if p == goal:
            path = [p]
            while p in parents:
                p = parents[p]
                path.append(p)
            return path[::-1]
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
            q = p[0]+dx, p[1]+dy
            cost = costs[p]+1
            if valid(q) and cost < costs.get(q, float('inf')):
                costs[q], parents[q] = cost, p
                heapq.heappush(queue, (cost+abs(q[0]-goal[0])+abs(q[1]-goal[1]), q))
    return []
