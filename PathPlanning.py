import heapq
import time

class Node():
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0  # cost from start to this node
        self.h = 0  # heuristic (estimated cost to goal)
        self.f = 0  # total cost (g + h)

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f


def astar_path(maze, start, end):
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    open_list = []
    closed_list = set()
    path = []

    heapq.heappush(open_list, start_node)

    while len(open_list) > 0:
        current_node = heapq.heappop(open_list)

        # Move current node from open to closed list
        closed_list.add(current_node.position)

        if current_node == end_node:
            while current_node is not None:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]  # Return path from start to goal (reversed)

        # Generate children
        children = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if (dx == 0) and (dy == 0):
                    continue
                node_position = (current_node.position[0] + dx, current_node.position[1] + dy)

                # Check if the node is within bounds and not blocked
                if node_position[0] >= len(maze) or node_position[0] < 0 or node_position[1] >= len(maze[0]) or node_position[1] < 0:
                    continue
                if maze[node_position[0]][node_position[1]] != 0:
                    continue

                new_node = Node(current_node, node_position)
                children.append(new_node)

        # Process each child
        for child in children:
            # Skip child if it's already in the closed list
            if child.position in closed_list:
                continue

            child.g = current_node.g + 1
            child.h = (child.position[0] - end_node.position[0]) ** 2 + (child.position[1] - end_node.position[1]) ** 2
            child.f = child.g + child.h

            # If child is not in open list, add it
            found_in_open = False
            for open_node in open_list:
                if child == open_node:  # Found the child in open list
                    found_in_open = True
                    if child.g < open_node.g:  # If a cheaper g value is found
                        open_node.g = child.g
                        open_node.f = child.f
                        open_node.parent = current_node
                    break  # Once we update, stop checking the open list for this child

            # If the child is not found in the open list, add it
            if not found_in_open:
                heapq.heappush(open_list, child)

    print("No path found!")
    return []  # In case no path is found


def print_maze_with_path(maze, path):
    """
    Function to print the maze with the path marked
    '*' for path cells
    """
    #visualization
    maze_copy = [row[:] for row in maze]  # Create a deep copy to avoid modifying the original maze

    for position in path:
        x, y = position
        maze_copy[x][y] = '*'  # Mark the path with '*'

    # Print the maze with path visualization
    for row in maze_copy:
        print(' '.join(str(cell) for cell in row))


def main():
    # Example maze where 0 is open, other integer is blocked
    maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 5],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    start = (0, 0)
    end = (7, 6)
    start_time = time.time()
    path = astar_path(maze, start, end)
    end_time = time.time()
    if path:
        print("Path found! Visualizing the maze with the path:")
        print_maze_with_path(maze, path)
    else:
        print("No path found!")

    print(f'Time taken: {end_time - start_time:.5f} seconds')


if __name__ == '__main__':
    main()