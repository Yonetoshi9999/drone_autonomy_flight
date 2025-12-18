"""
Path Planning Algorithms for Autonomous Drone Navigation

Implements A*, RRT*, and APF algorithms for 3D path planning with
obstacle avoidance. All algorithms target <100ms computation time.
"""

import time
import logging
from typing import List, Tuple, Optional
from dataclasses import dataclass
from abc import ABC, abstractmethod

import numpy as np
from scipy.spatial import KDTree
import heapq


logger = logging.getLogger(__name__)


@dataclass
class PathNode:
    """Node in a path."""
    position: np.ndarray  # (3,) array [x, y, z]
    yaw: float = 0.0  # Heading angle in radians


class PathPlanner(ABC):
    """Abstract base class for path planners."""

    @abstractmethod
    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        obstacles: List[Tuple[np.ndarray, float]],
    ) -> Optional[List[PathNode]]:
        """
        Plan a path from start to goal avoiding obstacles.

        Args:
            start: Start position (3,) array [x, y, z]
            goal: Goal position (3,) array [x, y, z]
            obstacles: List of (center, radius) tuples

        Returns:
            List of PathNode objects or None if no path found
        """
        pass


class AStarPlanner(PathPlanner):
    """
    A* path planning algorithm for 3D grid-based navigation.

    Fast and optimal for grid-based environments.
    Target computation time: <100ms
    """

    def __init__(
        self,
        grid_resolution: float = 0.5,
        safety_distance: float = 3.0,
        bounds: Tuple[np.ndarray, np.ndarray] = (
            np.array([-100, -100, 0]),
            np.array([100, 100, 50]),
        ),
    ):
        """
        Initialize A* planner.

        Args:
            grid_resolution: Grid cell size in meters
            safety_distance: Minimum distance from obstacles in meters
            bounds: (min_bounds, max_bounds) tuple defining search space
        """
        self.grid_resolution = grid_resolution
        self.safety_distance = safety_distance
        self.min_bounds, self.max_bounds = bounds

    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        obstacles: List[Tuple[np.ndarray, float]],
    ) -> Optional[List[PathNode]]:
        """Plan path using A* algorithm."""
        start_time = time.time()

        # Discretize start and goal to grid
        start_grid = self._to_grid(start)
        goal_grid = self._to_grid(goal)

        # Initialize data structures
        open_set = []
        heapq.heappush(open_set, (0.0, start_grid))
        came_from = {}
        g_score = {start_grid: 0.0}
        f_score = {start_grid: self._heuristic(start, goal)}

        # A* search
        while open_set:
            _, current = heapq.heappop(open_set)

            # Check if goal reached
            if self._distance(current, goal_grid) < 1.0:
                path = self._reconstruct_path(came_from, current)
                computation_time = (time.time() - start_time) * 1000
                logger.info(f"A* path found in {computation_time:.2f}ms")
                return path

            # Explore neighbors
            for neighbor in self._get_neighbors(current):
                # Check if neighbor is valid
                neighbor_pos = self._to_position(neighbor)

                if not self._is_valid(neighbor_pos, obstacles):
                    continue

                # Calculate scores
                tentative_g = g_score[current] + self._distance(current, neighbor)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(
                        neighbor_pos, goal
                    )
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

            # Timeout check (100ms)
            if (time.time() - start_time) * 1000 > 100:
                logger.warning("A* timeout after 100ms")
                return None

        logger.warning("A* failed to find path")
        return None

    def _to_grid(self, position: np.ndarray) -> Tuple[int, int, int]:
        """Convert position to grid coordinates."""
        grid = ((position - self.min_bounds) / self.grid_resolution).astype(int)
        return tuple(grid)

    def _to_position(self, grid: Tuple[int, int, int]) -> np.ndarray:
        """Convert grid coordinates to position."""
        return (
            np.array(grid) * self.grid_resolution + self.min_bounds
        )

    def _heuristic(self, pos1: np.ndarray, pos2: np.ndarray) -> float:
        """Euclidean distance heuristic."""
        return np.linalg.norm(np.array(pos1) - np.array(pos2))

    def _distance(
        self,
        grid1: Tuple[int, int, int],
        grid2: Tuple[int, int, int],
    ) -> float:
        """Distance between grid cells."""
        return np.linalg.norm(np.array(grid1) - np.array(grid2)) * self.grid_resolution

    def _get_neighbors(
        self,
        grid: Tuple[int, int, int],
    ) -> List[Tuple[int, int, int]]:
        """Get valid neighboring grid cells (26-connected)."""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue

                    neighbor = (grid[0] + dx, grid[1] + dy, grid[2] + dz)

                    # Check bounds
                    pos = self._to_position(neighbor)
                    if np.all(pos >= self.min_bounds) and np.all(pos <= self.max_bounds):
                        neighbors.append(neighbor)

        return neighbors

    def _is_valid(
        self,
        position: np.ndarray,
        obstacles: List[Tuple[np.ndarray, float]],
    ) -> bool:
        """Check if position is collision-free."""
        for obs_center, obs_radius in obstacles:
            dist = np.linalg.norm(position - obs_center)
            if dist < obs_radius + self.safety_distance:
                return False
        return True

    def _reconstruct_path(
        self,
        came_from: dict,
        current: Tuple[int, int, int],
    ) -> List[PathNode]:
        """Reconstruct path from came_from map."""
        path = [self._to_position(current)]

        while current in came_from:
            current = came_from[current]
            path.append(self._to_position(current))

        path.reverse()

        # Convert to PathNode objects with yaw
        path_nodes = []
        for i, pos in enumerate(path):
            if i < len(path) - 1:
                direction = path[i + 1] - pos
                yaw = np.arctan2(direction[1], direction[0])
            else:
                yaw = path_nodes[-1].yaw if path_nodes else 0.0

            path_nodes.append(PathNode(position=pos, yaw=yaw))

        return path_nodes


class RRTStarPlanner(PathPlanner):
    """
    RRT* (Rapidly-exploring Random Tree) path planning algorithm.

    Probabilistically complete and asymptotically optimal.
    Good for complex environments with many obstacles.
    """

    def __init__(
        self,
        max_iterations: int = 1000,
        step_size: float = 2.0,
        goal_sample_rate: float = 0.1,
        search_radius: float = 5.0,
        safety_distance: float = 3.0,
    ):
        """
        Initialize RRT* planner.

        Args:
            max_iterations: Maximum number of iterations
            step_size: Maximum step size in meters
            goal_sample_rate: Probability of sampling goal
            search_radius: Radius for finding nearby nodes
            safety_distance: Minimum distance from obstacles
        """
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.safety_distance = safety_distance

    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        obstacles: List[Tuple[np.ndarray, float]],
    ) -> Optional[List[PathNode]]:
        """Plan path using RRT* algorithm."""
        start_time = time.time()

        # Initialize tree
        tree = {tuple(start): {'parent': None, 'cost': 0.0}}
        vertices = [start]

        for i in range(self.max_iterations):
            # Sample random point (bias towards goal)
            if np.random.random() < self.goal_sample_rate:
                sample = goal
            else:
                sample = self._sample_free_space(obstacles)

            # Find nearest vertex
            nearest_idx = self._nearest(vertices, sample)
            nearest = vertices[nearest_idx]

            # Steer towards sample
            new_point = self._steer(nearest, sample)

            # Check if path is collision-free
            if not self._collision_free(nearest, new_point, obstacles):
                continue

            # Find nearby nodes for rewiring
            near_indices = self._near(vertices, new_point)

            # Choose best parent
            min_cost = tree[tuple(nearest)]['cost'] + np.linalg.norm(new_point - nearest)
            best_parent = nearest

            for near_idx in near_indices:
                near_vertex = vertices[near_idx]
                cost = tree[tuple(near_vertex)]['cost'] + np.linalg.norm(
                    new_point - near_vertex
                )

                if cost < min_cost and self._collision_free(
                    near_vertex, new_point, obstacles
                ):
                    min_cost = cost
                    best_parent = near_vertex

            # Add new vertex
            vertices.append(new_point)
            tree[tuple(new_point)] = {'parent': tuple(best_parent), 'cost': min_cost}

            # Rewire tree
            for near_idx in near_indices:
                near_vertex = vertices[near_idx]
                cost = min_cost + np.linalg.norm(new_point - near_vertex)

                if cost < tree[tuple(near_vertex)]['cost'] and self._collision_free(
                    new_point, near_vertex, obstacles
                ):
                    tree[tuple(near_vertex)]['parent'] = tuple(new_point)
                    tree[tuple(near_vertex)]['cost'] = cost

            # Check if goal reached
            if np.linalg.norm(new_point - goal) < self.step_size:
                if self._collision_free(new_point, goal, obstacles):
                    tree[tuple(goal)] = {
                        'parent': tuple(new_point),
                        'cost': min_cost + np.linalg.norm(goal - new_point),
                    }
                    path = self._extract_path(tree, goal)
                    computation_time = (time.time() - start_time) * 1000
                    logger.info(f"RRT* path found in {computation_time:.2f}ms")
                    return path

            # Timeout check (100ms)
            if (time.time() - start_time) * 1000 > 100:
                logger.warning("RRT* timeout after 100ms")
                # Return best path found so far if any
                closest_to_goal = min(
                    vertices, key=lambda v: np.linalg.norm(v - goal)
                )
                if np.linalg.norm(closest_to_goal - goal) < 10.0:
                    return self._extract_path(tree, closest_to_goal)
                return None

        logger.warning("RRT* failed to find path")
        return None

    def _sample_free_space(
        self,
        obstacles: List[Tuple[np.ndarray, float]],
    ) -> np.ndarray:
        """Sample a random point in free space."""
        max_attempts = 100
        for _ in range(max_attempts):
            sample = np.random.uniform([-100, -100, 0], [100, 100, 50])

            # Check if collision-free
            collision = False
            for obs_center, obs_radius in obstacles:
                if np.linalg.norm(sample - obs_center) < obs_radius + self.safety_distance:
                    collision = True
                    break

            if not collision:
                return sample

        # If all attempts fail, return random point anyway
        return np.random.uniform([-100, -100, 0], [100, 100, 50])

    def _nearest(self, vertices: List[np.ndarray], point: np.ndarray) -> int:
        """Find index of nearest vertex to point."""
        distances = [np.linalg.norm(v - point) for v in vertices]
        return int(np.argmin(distances))

    def _near(self, vertices: List[np.ndarray], point: np.ndarray) -> List[int]:
        """Find indices of vertices within search radius."""
        distances = [np.linalg.norm(v - point) for v in vertices]
        return [i for i, d in enumerate(distances) if d < self.search_radius]

    def _steer(self, from_point: np.ndarray, to_point: np.ndarray) -> np.ndarray:
        """Steer from from_point towards to_point with step_size limit."""
        direction = to_point - from_point
        distance = np.linalg.norm(direction)

        if distance < self.step_size:
            return to_point

        return from_point + direction / distance * self.step_size

    def _collision_free(
        self,
        from_point: np.ndarray,
        to_point: np.ndarray,
        obstacles: List[Tuple[np.ndarray, float]],
    ) -> bool:
        """Check if path between two points is collision-free."""
        steps = int(np.linalg.norm(to_point - from_point) / 0.5) + 1
        for i in range(steps + 1):
            t = i / steps
            point = from_point + t * (to_point - from_point)

            for obs_center, obs_radius in obstacles:
                if np.linalg.norm(point - obs_center) < obs_radius + self.safety_distance:
                    return False

        return True

    def _extract_path(self, tree: dict, goal: np.ndarray) -> List[PathNode]:
        """Extract path from tree."""
        path = [goal]
        current = tuple(goal)

        while tree[current]['parent'] is not None:
            current = tree[current]['parent']
            path.append(np.array(current))

        path.reverse()

        # Convert to PathNode objects
        path_nodes = []
        for i, pos in enumerate(path):
            if i < len(path) - 1:
                direction = path[i + 1] - pos
                yaw = np.arctan2(direction[1], direction[0])
            else:
                yaw = path_nodes[-1].yaw if path_nodes else 0.0

            path_nodes.append(PathNode(position=pos, yaw=yaw))

        return path_nodes


class APFPlanner(PathPlanner):
    """
    Artificial Potential Field (APF) path planning.

    Fast reactive planner using attractive and repulsive forces.
    Good for dynamic environments but can get stuck in local minima.
    """

    def __init__(
        self,
        k_att: float = 1.0,
        k_rep: float = 50.0,
        d0: float = 5.0,
        max_iterations: int = 500,
        step_size: float = 0.5,
        goal_threshold: float = 1.0,
        safety_distance: float = 3.0,
    ):
        """
        Initialize APF planner.

        Args:
            k_att: Attractive potential gain
            k_rep: Repulsive potential gain
            d0: Influence distance of obstacles
            max_iterations: Maximum iterations
            step_size: Step size for gradient descent
            goal_threshold: Distance to goal for termination
            safety_distance: Minimum distance from obstacles
        """
        self.k_att = k_att
        self.k_rep = k_rep
        self.d0 = d0
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_threshold = goal_threshold
        self.safety_distance = safety_distance

    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        obstacles: List[Tuple[np.ndarray, float]],
    ) -> Optional[List[PathNode]]:
        """Plan path using APF method."""
        start_time = time.time()

        path = [start.copy()]
        current = start.copy()

        for i in range(self.max_iterations):
            # Calculate total force
            f_att = self._attractive_force(current, goal)
            f_rep = self._repulsive_force(current, obstacles)
            f_total = f_att + f_rep

            # Move in direction of force
            force_magnitude = np.linalg.norm(f_total)
            if force_magnitude > 0:
                step = (f_total / force_magnitude) * self.step_size
                current = current + step
                path.append(current.copy())

            # Check if goal reached
            if np.linalg.norm(current - goal) < self.goal_threshold:
                path.append(goal)
                computation_time = (time.time() - start_time) * 1000
                logger.info(f"APF path found in {computation_time:.2f}ms")

                # Smooth and subsample path
                path_nodes = self._smooth_path(path)
                return path_nodes

            # Check for local minimum (stuck)
            if i > 10 and np.linalg.norm(path[-1] - path[-10]) < 0.1:
                logger.warning("APF stuck in local minimum")
                return None

            # Timeout check (100ms)
            if (time.time() - start_time) * 1000 > 100:
                logger.warning("APF timeout after 100ms")
                return None

        logger.warning("APF max iterations reached")
        return None

    def _attractive_force(self, current: np.ndarray, goal: np.ndarray) -> np.ndarray:
        """Calculate attractive force towards goal."""
        return self.k_att * (goal - current)

    def _repulsive_force(
        self,
        current: np.ndarray,
        obstacles: List[Tuple[np.ndarray, float]],
    ) -> np.ndarray:
        """Calculate repulsive force from obstacles."""
        f_rep = np.zeros(3)

        for obs_center, obs_radius in obstacles:
            diff = current - obs_center
            distance = np.linalg.norm(diff)
            effective_distance = distance - obs_radius

            if effective_distance < self.d0:
                if effective_distance < 0.1:
                    effective_distance = 0.1  # Avoid division by zero

                # Repulsive force magnitude
                magnitude = self.k_rep * (1.0 / effective_distance - 1.0 / self.d0) * (
                    1.0 / (effective_distance ** 2)
                )

                # Force direction
                f_rep += magnitude * (diff / distance)

        return f_rep

    def _smooth_path(self, path: List[np.ndarray]) -> List[PathNode]:
        """Smooth and subsample path."""
        if len(path) < 2:
            return [PathNode(position=path[0], yaw=0.0)]

        # Subsample path (every 5th point)
        subsampled = path[::5]
        if not np.array_equal(subsampled[-1], path[-1]):
            subsampled.append(path[-1])

        # Convert to PathNode objects
        path_nodes = []
        for i, pos in enumerate(subsampled):
            if i < len(subsampled) - 1:
                direction = subsampled[i + 1] - pos
                yaw = np.arctan2(direction[1], direction[0])
            else:
                yaw = path_nodes[-1].yaw if path_nodes else 0.0

            path_nodes.append(PathNode(position=pos, yaw=yaw))

        return path_nodes
