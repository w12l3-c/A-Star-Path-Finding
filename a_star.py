# A Star Pathfinding Algorithm
# There are no weighted links (supposed)
import sys

# F(n) = G(n) + H(n)
# F(n) = Total cost of the path, lower is better
# G(n) = Exact cost of the path from the start node to the current node
# H(n) = Estimated cost of the path from the current node to the goal node (using L2 distance)

# Start node FGH are F = H to have the first estimate
# All other nodes initialize with inf

# Open list is a list of nodes to be evaluated
# Closed list is a list of nodes already evaluated

# Basic algorithm
# 1st look at node A
# 2nd look at edges of node A
# 3rd calculate and update the FGH of the edges, if the FGH is lower than the previous FGH, update it
# G is the shortest distance from A to B while H is the estimate from B to end
# 4th record the previous node A when advance to B
# 5th put node B in open set (F, B) and remove node A from open set
# 6th look at neighbours of B, C and D
# 7th calculate and update the FGH of the edges (B to C and B to D)
# 8th add C and D to open set (F, C) and (F, D) and remove B from open set
# 9th D is the end, so we are popping the lowest F score from open list and finish the algorithm
# Then we can trace back from D last node is B and last node is A

import pygame, math, random, time
from queue import PriorityQueue

pygame.init()

WIDTH = 800
WIN = pygame.display.set_mode((WIDTH,WIDTH))
FPS = 60
clock = pygame.time.Clock()
pygame.display.set_caption("A Star Path Finding Algorithm")

# Colours
RED = (219, 15, 15)
GREEN = (94, 227, 5)
BLUE = (66, 120, 245)
YELLOW = (237, 216, 24)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (76, 34, 181)
ORANGE = (209, 131, 21)
GREY = (182, 176, 191)
TURQUOISE = (81, 237, 221)


class Node:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        # x and y are the coordinates of the node because it is a grid
        self.x = row * width
        self.y = col * width
        # White is the unchecked neighbour nodes
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows

    def get_pos(self):
        return self.row, self.col

    def get_colour(self):
        return self.color

    def is_closed(self):
        # closed list, red squares
        return self.color == RED

    def is_open(self):
        # open list, green squares
        return self.color == GREEN

    def is_barrier(self):
        # barrier, black squares
        return self.color == BLACK

    def is_start(self):
        # start, orange squares
        return self.color == ORANGE

    def is_end(self):
        # end, purple squares
        return self.color == TURQUOISE

    def reset(self):
        # reset every node to white
        self.color = WHITE

    def make_closed(self):
        # create red squares
        self.color = RED

    def make_open(self):
        # create green squares
        self.color = GREEN

    def make_barrier(self):
        # create black squares
        self.color = BLACK

    def make_start(self):
        # create orange squares
        self.color = ORANGE

    def make_end(self):
        # create purple squares
        self.color = TURQUOISE

    def make_path(self):
        # create turquoise squares as path
        self.color = PURPLE

    def draw(self, win):
        # draw squares
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbours(self, grid):
        # update the neighbours of the current node
        self.neighbors = []

        # check if the node doesn't exceed the window or the row below is barrier
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier():
            # append the node below
            self.neighbors.append(grid[self.row + 1][self.col])

        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():
            # append the node above
            self.neighbors.append(grid[self.row - 1][self.col])

        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier():
            # append the node right
            self.neighbors.append(grid[self.row][self.col + 1])

        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():
            # append the node left
            self.neighbors.append(grid[self.row][self.col - 1])

    def __lt__(self, other):
        # less than operator
        # handle the function to compare two nodes' F score
        return False


def heuristic(p1, p2):
    # L2 Euclidean Distance
    x1, y1, = p1    # unpack the tuple point 1
    x2, y2, = p2    # unpack the tuple point end
    return abs(x1 - x2) + abs(y1 - y2)
    # return math.sqrt(pow(x1-x2, 2) + pow(y1-y2, 2))


def reconstruct_path(last_node, current, drawing):
    while current in last_node:
        current = last_node[current]
        current.make_path()
        drawing()


def algorithm(drawing, grid, start, end):
    count = 0
    open_set = PriorityQueue()  # keep sort items by F score
    open_set.put((0, count, start)) # Start node
    last_node = {}
    g_score = {node:float('inf') for row in grid for node in row}
    g_score[start] = 0
    f_score = {node:float('inf') for row in grid for node in row}
    f_score[start] = heuristic(start.get_pos(), end.get_pos())

    # To check whether things are in the PriorityQueue
    open_set_hash = {start}

    # run until there is no mode nodes in the open set
    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        # open_set stores (F, count, node) and only node is needed
        current = open_set.get()[2]
        # remove the current node from the open set
        open_set_hash.remove(current)

        # if the current node is the end node, we are done
        if current == end:
            reconstruct_path(last_node, end, drawing)
            end.make_end()  # make the end node turquoise not part of the path
            return True

        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1

            # found a better path to the neighbor
            if temp_g_score < g_score[neighbor]:
                # as the g score is smaller, the prev node is updated
                last_node[neighbor] = current
                # update the g score of the node
                g_score[neighbor] = temp_g_score
                # equation of F score
                f_score[neighbor] = temp_g_score + heuristic(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

        drawing()

        if current != start:
            current.make_closed()

    return False


def make_grid(rows, width):
    grid = []
    gap = width // rows # width of each node squares
    for i in range(rows):
        grid.append([]) # 2D array
        for j in range(rows):
            # create a node object
            node = Node(i, j, gap, rows)
            # add the node to the grid across the row
            grid[i].append(node)
    return grid


def draw_gridline(win, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i*gap), (width, i*gap))
    for j in range(rows):
        pygame.draw.line(win, GREY, (j*gap, 0), (j*gap, width))


def draw(win, grid, rows, width):
    win.fill(WHITE)
    for row in grid:
        for node in row:
            node.draw(win)

    draw_gridline(win, rows, width)
    pygame.display.update()


def get_clicked_pos(pos, rows, width):
    gap = width // rows
    y, x = pos

    # get the row and col of the clicked node
    row = y // gap
    col = x // gap

    return row, col


def main(win, width):
    ROWS = 50
    grid = make_grid(ROWS, width)

    start = None
    end = None

    run = True

    while run:
        draw(win, grid, ROWS, width)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            # left click
            if pygame.mouse.get_pressed()[0]:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                node = grid[row][col]
                # cannot overwrite end with start
                if not start and node != end:
                    start = node
                    start.make_start()
                # cannot overwrite start with end
                elif not end and node != start:
                    end = node
                    end.make_end()

                elif node != start and node != end:
                    node.make_barrier()

            # right click
            elif pygame.mouse.get_pressed()[2]:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                node = grid[row][col]
                node.reset()
                if node == start:
                    start = None
                elif node == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                # we need a start and end before running the algorithm
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for node in row:
                            node.update_neighbours(grid)
                    algorithm(lambda: draw(win, grid, ROWS, width), grid, start, end)

                # clear the board
                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)

        clock.tick(60)
    pygame.quit()

main(WIN, WIDTH)