import heapq
import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt
import math

class AStarPathfinder:
    def __init__(self, map_array: np.array, start: tuple, goal: tuple,
                 wall_influence=5.0, buffer_factor=2.0, center_bias=1.0, min_step=5):
        """
        Args:
            map_array: mapa 0 (obst), 128 (desconhecido), 255 (livre)
            start, goal: (linha, coluna)
            wall_influence/buffer_factor: penaliza proximidade de parede
            center_bias: (>=0) puxa o caminho para o centro do corredor
            min_step: distância mínima entre pontos quando simplificar
        """
        self.start = start
        self.goal = goal
        self.wall_influence = float(wall_influence)
        self.buffer_factor = float(buffer_factor)
        self.center_bias = float(center_bias)
        self.min_step = int(min_step)

        self.GOAL_REACHEABLE = False

        # mapa original (para filtrar desconhecido depois)
        self.map = map_array.copy()
        # mapa de planejamento (desconhecido tratado como livre)
        self.map_array = self.preprocess_map(map_array)

        # campo potencial e EDT
        self.potential_field, self.edt_map, self.edt_max = self.create_potential_field()

    def preprocess_map(self, map_array: np.array) -> np.array:
        # trata desconhecido como livre só para planejar
        map_array = map_array.copy()
        map_array[map_array == 128] = 255
        return map_array

    def create_potential_field(self):
        """
        - edt_map: distância até obstáculo (alto = centro do corredor)
        - potential_field: penalidade perto de parede
        """
        free_mask = (self.map_array == 255).astype(np.uint8)
        edt_map = distance_transform_edt(free_mask).astype(np.float32)
        edt_max = float(edt_map.max() + 1e-6)

        potential_field = self.wall_influence * np.exp(-edt_map / self.buffer_factor)
        return potential_field.astype(np.float32), edt_map, edt_max

    def heuristic(self, a: tuple, b: tuple) -> float:
        return np.linalg.norm(np.array(a) - np.array(b))

    def find_path(self):
        rows, cols = self.map_array.shape
        open_set = []
        heapq.heappush(open_set, (0.0, self.start))
        came_from = {}
        g_score = {self.start: 0.0}
        f_score = {self.start: self.heuristic(self.start, self.goal)}

        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1),
                     (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == self.goal:
                return came_from, current

            for dx, dy in neighbors:
                ni, nj = current[0] + dx, current[1] + dy
                if 0 <= ni < rows and 0 <= nj < cols:
                    if self.map_array[ni, nj] > 120:  # livre
                        base = 1.0 + float(self.potential_field[ni, nj])
                        # --- viés pro centro (EDT alto => custo menor) ---
                        # termo ∈ [0, center_bias], maior perto de parede, menor no centro:
                        center_term = self.center_bias * (1.0 - float(self.edt_map[ni, nj]) / self.edt_max)
                        tentative_g = g_score[current] + base + center_term

                        nb = (ni, nj)
                        if nb not in g_score or tentative_g < g_score[nb]:
                            came_from[nb] = current
                            g_score[nb] = tentative_g
                            f_score[nb] = tentative_g + self.heuristic(nb, self.goal)
                            heapq.heappush(open_set, (f_score[nb], nb))

        print("Caminho não encontrado")
        return None, None

    def reconstruct_path(self, came_from: dict, current: tuple) -> list:
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(self.start)
        path.reverse()
        return path

    def know_path(self, path: list) -> list:
        """
        Corta trechos que caem em área desconhecida no mapa original (valor 128)
        e valida saltos muito grandes.
        """
        path_ = [p for p in path if self.map[p] != 128]

        max_distance = 2
        for i in range(1, len(path_)):
            if math.dist(path_[i], path_[i-1]) > max_distance:
                return path_[:i]

        if len(path) != len(path_):
            return path_[:-2] if len(path_) >= 2 else path_

        self.GOAL_REACHEABLE = True
        return path

    def simplify_path(self, path: list, min_step: int = None) -> list:
        """
        Mantém quinas e descarta pontos muito próximos.
        """
        if not path:
            return []

        if min_step is None:
            min_step = self.min_step

        out = [path[0]]
        last = np.array(path[0], dtype=float)
        prev_dir = None

        for i in range(1, len(path)):
            prev = np.array(path[i-1], dtype=float)
            curr = np.array(path[i], dtype=float)
            step_dir = (int(curr[0] - prev[0]), int(curr[1] - prev[1]))
            turn = (prev_dir is not None and step_dir != prev_dir)
            far = np.linalg.norm(curr - last) >= float(min_step)
            is_last = (i == len(path) - 1)

            if turn or far or is_last:
                out.append(tuple(curr.astype(int)))
                last = curr
                prev_dir = step_dir
            else:
                prev_dir = step_dir

        return out

    def plot_path(self, path: list, simplified_path: list):
        plt.figure(figsize=(10, 10))
        plt.imshow(self.map, cmap='gray')
        plt.scatter(self.start[1], self.start[0], color='green', s=100, label='Início')
        plt.scatter(self.goal[1], self.goal[0], color='blue', s=100, label='Objetivo')

        if path:
            px, py = zip(*path)
            plt.plot(py, px, color='magenta', linewidth=1, label='Caminho A* (com viés de centro)')
            sx, sy = zip(*simplified_path)
            plt.plot(sy, sx, color='red', linewidth=2, linestyle='--', label='Caminho Simplificado')
        else:
            plt.title("Caminho não encontrado")

        plt.legend()
        plt.axis('equal')
        plt.show()

    def run(self, show_path=True):
        print("Iniciando busca pelo caminho...")
        came_from, final_node = self.find_path()

        if not final_node:
            print("Nenhum caminho pôde ser encontrado.")
            return None

        print("Reconstruindo caminho...")
        path = self.reconstruct_path(came_from, final_node)

        print("Robo não anda no desconhecido")
        path = self.know_path(path)

        print("Caminho encontrado, simplificando...")
        simplified_path = self.simplify_path(path, min_step=self.min_step)

        print("Plotando o caminho...")
        if show_path:
            self.plot_path(path, simplified_path)

        return simplified_path


# ---------------- utilitário de preparo de mapa ----------------
def prep_map(map_path: str) -> np.array:
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
    astar = AStarPathfinder(
        map_array,
        start=(60, 20),
        goal=(60, 120),
        wall_influence=10.0,
        buffer_factor=3.0,
        center_bias=1.0,   # 0.5–2.0 puxa mais pro centro
        min_step=6,        # reduz waypoints muito próximos
    )
    astar.run()


if __name__ == '__main__':
    main()
