import heapq
import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt
import math

class AStarPathfinder:
    def __init__(self, map_array: np.array, start: tuple, goal: tuple, wall_influence=5.0, buffer_factor=2.0):
        """
        Initialize A* with a map, start cell, goal cell, and influence parameters.

        Args:
            map_array (np.array): Binary map with obstacles and free space.
            start (tuple): Start cell as (row, column).
            goal (tuple): Goal cell as (row, column).
            wall_influence (float): Weight applied to cells near walls.
            buffer_factor (float): Scale factor for the wall influence.
        """
        self.start = start
        self.goal = goal
        self.wall_influence = wall_influence
        self.buffer_factor = buffer_factor
        self.GOAL_REACHEABLE = False  
        
        # Prepare the map by expanding its borders and adjusting the array.
        self.map = map_array.copy()
        self.map_array = self.preprocess_map(map_array)

        # Create a map-based potential field to influence the path.
        self.potential_field = self.create_potential_field()

    def preprocess_map(self, map_array: np.array) -> np.array:
        """
        Adjust the map by converting intermediate values into obstacles.

        Args:
            map_array (np.array): Original map.

        Returns:
            np.array: Processed map.
        """
        map_array[map_array == 128] = 255
        return map_array

    def create_potential_field(self) -> np.array:
        """
        Generate a potential field based on the distance from obstacles.

        Returns:
            np.array: Potential field.
        """
        inverted_map = np.where(self.map_array == 255, 1, 0)
        distance_map = distance_transform_edt(inverted_map)
        potential_field = self.wall_influence * np.exp(-distance_map / self.buffer_factor)
        return potential_field

    def heuristic(self, a: tuple, b: tuple) -> float:
        """
        Compute the heuristic between two points.

        Args:
            a (tuple): Point A.
            b (tuple): Point B.

        Returns:
            float: Heuristic value.
        """

        return np.linalg.norm(np.array(a) - np.array(b))

    def find_path(self):
        """
        Run A* to find a path to the goal.

        Returns:
            dict: Node predecessors in the path. Returns None if no path is found.
            tuple: The final node, usually the goal, or None if no path is found.
        """

        rows, cols = self.map_array.shape
        open_set = []
        heapq.heappush(open_set, (0, self.start))
        came_from = {}
        g_score = {self.start: 0}
        f_score = {self.start: self.heuristic(self.start, self.goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == self.goal:
                return came_from, current

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                    if self.map_array[neighbor] > 120:
                        tentative_g_score = g_score[current] + 1 + self.potential_field[neighbor]
                        if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                            came_from[neighbor] = current
                            g_score[neighbor] = tentative_g_score
                            f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, self.goal)
                            heapq.heappush(open_set, (f_score[neighbor], neighbor))

        print("Path not found")
        return None, None

    def reconstruct_path(self, came_from: dict, current: tuple) -> list:
        """
        Reconstruct the path from the final node back to the start.
        
        Args:
            came_from (dict): Dictionary of path predecessors.
            current (tuple): Final node, usually the goal.
        
        Returns:
            list: List of cells in the reconstructed path.
        """
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(self.start)
        path.reverse()
        return path

    def know_path(self, path: list) -> list:
        """
        Remove unknown segments and adjust the path when needed.

        Args:
            path (list): Full path.

        Returns:
            list: Adjusted path.
        """
        # Filter the path by removing unknown cells.
        path_ = [p for p in path if self.map[p] != 128]

        # Define the maximum gap allowed between consecutive path cells.
        max_distance = 2

        # Check the distance between consecutive cells in the adjusted path.
        for i in range(1, len(path_)):
            # Compute the Euclidean distance between consecutive cells.
            distance = math.sqrt((path_[i][0] - path_[i-1][0]) ** 2 + (path_[i][1] - path_[i-1][1]) ** 2)
            print(distance)
            if distance > max_distance:
                # If the gap is too large, keep the path only up to the previous cell.
                path = path_[:i]
                break
        else:
            # If there are no distant cells, apply the normal adjustment.
            if len(path) != len(path_):
                path = path_[:-2]
            else:
                self.GOAL_REACHEABLE = True

        return path

    def simplify_path(self, path: list) -> list:
        """
        Simplify the path by removing repeated directions.
        
        Args:
            path (list): Full path.

        Returns:
            list: Simplified path.
        """
        if not path:
            return []

        simplified_path = [path[0]]
        direction = (0, 0)

        for i in range(1, len(path)):
            prev = path[i - 1]
            curr = path[i]
            new_direction = (curr[0] - prev[0], curr[1] - prev[1])

            if new_direction != direction:
                simplified_path.append(prev)
                direction = new_direction

        simplified_path.append(path[-1])
        return simplified_path

    def plot_path(self, path: list, simplified_path: list):
        """
        Display the map with the full and simplified paths.

        Args:
            path (list): Full path found by the planner.
            simplified_path (list): Simplified path found by the planner.
        """
        simplified_path = self.simplify_path(path)

        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='gray')
        plt.scatter(self.start[1], self.start[0], color='green', s=100, label='Start')
        plt.scatter(self.goal[1], self.goal[0], color='blue', s=100, label='Goal')

        if path:
            path_x, path_y = zip(*path)
            plt.plot(path_y, path_x, color='magenta', linewidth=1, label='Full Path')
            simp_x, simp_y = zip(*simplified_path)
            plt.plot(simp_y, simp_x, color='red', linewidth=2, linestyle='--', label='Simplified Path')
        else:
            plt.title("Path not found")

        plt.legend()
        plt.axis('equal')
        plt.show()

    def run(self, show_path=True):
        """
        Run the A* pipeline used by the navigator to generate a path.
        This executes search, reconstruction, simplification, and optional visualization.
        
        Args:
            show_path (bool): If True, display the path graphically.

        Returns:
            list or None: Simplified path, or None if no path is found.
        """
        print("Starting path search...")
        came_from, final_node = self.find_path()

        if final_node:
            print("Reconstructing path...")
            path = self.reconstruct_path(came_from, final_node)
            
            print("Filtering unknown regions from the path...")
            path = self.know_path(path)

            print("Path found, simplifying...")
            simplified_path = self.simplify_path(path)

            print("Plotting path...")
            if show_path:
                self.plot_path(path, simplified_path)
            
            return simplified_path
        else:
            print("No path could be found.")
            return None


def prep_map(map_path: str) -> np.array:
    """
    Prepare the map by loading and processing the input image.

    Args:
        map_path (str): Path to the map file.

    Returns:
        np.array: Processed map as a NumPy array.
    """
    map_array = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)
    map_array[map_array == 0] = 0
    map_array[map_array == 205] = 128
    map_array[map_array == 254] = 255
    map_array[(map_array >= 60) & (map_array != 128) & (map_array != 255)] = 0
    map_array = map_array.astype(np.uint8)
    kernel = np.ones((3, 3), np.uint8)
    map_array = cv2.morphologyEx(map_array, cv2.MORPH_OPEN, kernel)
    map_array = np.flipud(map_array)
    map_array = np.pad(map_array, ((0, 200), (0, 200)), 'constant', constant_values=128)
    return map_array


def main():
    map_array = prep_map('map5.pgm')
    astar = AStarPathfinder(map_array, (60, 20), (60, 120), wall_influence=10.0, buffer_factor=3.0)
    astar.run()


if __name__ == '__main__':
    main()
