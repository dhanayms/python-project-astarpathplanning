import numpy as np
import math
import heapq
import path_planning as pp
import matplotlib.pyplot as plt

# Priority Queue based on heapq
class PriorityQueue:
    def __init__(self):
        self.elements = []
    def isEmpty(self):
        return len(self.elements) == 0
    def add(self, item, priority):
        heapq.heappush(self.elements,(priority,item))
    def remove(self):
        return heapq.heappop(self.elements)[1]

def manhattan(a, b):
    d = 0
    for i, j in zip(a, b):
        d += abs(i - j)
    return d
def astar_m(map,size):

    total = 0
    cost=0
    came_from = {}
    m = map
    x, y = np.where(m == -2)
    start=(x[0],y[0])
    xg, yg = np.where(m == -3)
    goal = (xg[0], yg[0])
    f = PriorityQueue()
    hs = manhattan(start, goal)
    gs = manhattan(start, start)
    fs=hs+gs

    f.add((x[0], y[0], fs), fs)
    came_from[(x[0], y[0])] = 0



    while not f.isEmpty():
        cost+=1
        (x, y, cost1) = f.remove()

        lx = x - 1
        ly = y
        rx = x + 1
        ry = y
        ux = x
        uy = y + 1
        dx = x
        dy = y - 1
        left=(lx,ly)
        right=(rx,ry)
        down=(dx,dy)
        up=(ux,uy)

        # if left[0]>0 and up[1]>0 andright[0]<size and down[1]<size:

        if (lx > -1 and ly > -1) and (lx < size and ly < size):

            if m[lx, ly] == -3:
                print('goal is reached')
                came_from[(lx, ly)] = (x, y)

                # found=True
                break

            if m[lx, ly] == 0:
                hl = manhattan(left, goal)
                gl = manhattan(left, start)
                fl=hl+gl
                m[lx, ly] = fl
                f.add((lx, ly, fl), fl)
                came_from[(lx, ly)] = (x, y)
                total += 1
        if (rx > -1 and ry > -1) and (rx < size and ry < size):

            if m[rx, ry] == -3:
                print('goal is reached')
                came_from[(rx, ry)] = (x, y)

                #                 found=True
                break

            if m[rx, ry] == 0:
                hr = manhattan(right, goal)
                gr = manhattan(right, start)
                fr=hr+gr
                m[rx, ry] = fr
                f.add((rx, ry, fr), fr)
                came_from[(rx, ry)] = (x, y)
                total += 1

        if (ux > -1 and uy > -1) and (ux < size and uy < size):

            if m[ux, uy] == -3:
                print('goal is reached')
                came_from[(ux, uy)] = (x, y)

                # found=True
                break

            if m[ux, uy] == 0:
                hu = manhattan(up, goal)
                gu = manhattan(up, start)
                fu=hu+gu
                m[ux, uy] = fu
                f.add((ux, uy, fu), fu)
                came_from[(ux, uy)] = (x, y)
                total += 1
        if (dx > -1 and dy > -1) and (dx < size and dy < size):

            if m[dx, dy] == -3:
                print('goal is reached')
                came_from[(dx, dy)] = (x, y)

                # found=True
                break

            if m[dx, dy] == 0:
                hd = manhattan(down, goal)
                gd = manhattan(down, start)
                fd=hd+gd
                m[dx, dy] = fd
                f.add((dx, dy, fd), fd)
                came_from[(dx, dy)] = (x, y)
                total += 1
    a1 = came_from
    path1 = []
    full = list(a1.keys())
    last_cell = full[-1]
    f_cell = full[0]

    print(last_cell, f_cell)
    optimal_count = 0

    while a1[last_cell] != 0:
        if last_cell in a1:
            optimal_count += 1

            path1.append(last_cell)
            last_cell = a1[last_cell]
    # path = path.reverse()
    # print('The final path is*****',path)
    path1.append(f_cell)
    path1.reverse()
    l = len(path1)

    path_final = np.array(path1)
    pathnew = np.flip(path_final)
    # pp.plotMap(aa, pathnew, 'BFS')

    return total, l, pathnew


#aa = pp.generateMap2d([40, 40])
aa,inf= pp.generateMap2d_obstacle([60, 60])
print(aa)
plt.imshow(aa)
plt.show()

a1, a2, a3 = astar_m(aa, 60)
pp.plotMap(aa, a3, 'Astar manhattan')

