from projekt.Vertex_class import *
from projekt.Point_class import *
from projekt.Triangle_class import *
from projekt.main import *
from projekt.helper_functions import *
from projekt.Broom_class import *
from projekt.RBTree_class import *


class Polygon:
    def __init__(self, vertices):  # wierzchołki są zadawane w lewa strone
        self.vertices = self.__sorted_vertices(vertices)
        self.bottom_point_index = 0  # index
        self.top_point_index = self.__top_point()  # index
        self.triangles = set()
        self.chain = self.__get_chain()
        self.scenes = []  # puste
        self.sides = [] #jako listy
        self.__is_triangulated = False
        self.parent = None #dzielimy niemonotoniczny na monotoniczne i one mają ten bazowy jako parent
        self.sub_polygons = []
        self.additional_diagonals = []
        for i in range(len(self.vertices)):
            self.sides.append([(self.vertices[i].point.x, self.vertices[i].point.y),
                               (self.vertices[(i + 1) % len(self.vertices)].point.x,
                                self.vertices[(i + 1) % len(self.vertices)].point.y)])

    def __sorted_vertices(self, vertices):
        index = 0
        for i in range(len(vertices)):
            current = vertices[index].point
            candidate = vertices[i].point
            if candidate.y < current.y:
                index = i
            elif candidate.y == current.y and candidate.x < current.x:
                index = i
        return vertices[index:] + vertices[:index]

    def __top_point(self):
        index = 0
        for i in range(len(self.vertices)):
            current = self.vertices[index].point
            candidate = self.vertices[i].point
            if candidate.y > current.y:
                index = i
            elif candidate.y == current.y and candidate.x < current.x:
                index = i
        return index

    def __get_chain(self):
        C = [1 for _ in range(len(self.vertices))]
        i = 0
        while self.vertices[i] != self.vertices[self.top_point_index]:
            C[i] = -1
            i += 1
        C[0] = -1
        C[self.top_point_index] = 1
        return C

    def add_triangle(self, triangle, change = True):
        self.triangles.add(triangle)
        if change:
            triangle.polygon = self

    def __is_y_monotone(self):
        V = self.vertices
        for i in range(self.top_point_index, len(V)):
            if V[(i + 1) % len(V)].point.y > V[i % len(V)].point.y:
                return False
        for i in range(self.top_point_index):
            if V[(i + 1) % len(V)].point.y < V[i % len(V)].point.y:
                return False
        return True

    def __classify_vertices(self, epsilon=0):
        # kolory opisane ponizej do pozniejszej wizualizacji
        classification = {
            # 'limegreen'
            'poczatkowe': [],
            # 'red'
            'koncowe': [],
            # 'mediumblue'
            'laczace': [],
            # 'lightsteelblue'
            'dzielace': [],
            # 'sienna'
            'prawidlowe': []
        }

        classified_vertices = {}

        V = self.vertices
        n = len(V)

        for i in range(n):
            p, q, r = V[(i - 1) % n].point, V[i % n].point, V[(i + 1) % n].point
            diff = (p.y - q.y, r.y - q.y)
            d = Point.det(p, q, r)
            if diff[0] > 0 and diff[1] > 0:  # oba punkty są powyzej
                if d > epsilon:  # phi > pi / clockwise
                    classification['laczace'].append(q)
                    classified_vertices[q] = 'laczace'
                    continue
                elif d < epsilon:  # phi < pi / counterclockwise
                    classification['koncowe'].append(q)
                    classified_vertices[q] = 'koncowe'
                    continue
            elif diff[0] < 0 and diff[1] < 0:  # oba punkty są ponizej
                if d > epsilon:  # phi > pi
                    classification['dzielace'].append(q)
                    classified_vertices[q] = 'dzielace'
                    continue
                elif d < epsilon:  # phi < pi
                    classification['poczatkowe'].append(q)
                    classified_vertices[q] = 'poczatkowe'
                    continue
            else:  # prawidlowy
                classification['prawidlowe'].append(q)
                classified_vertices[q] = 'prawidlowe'

        self.scenes.append(
            Scene(points=[PointsCollection([(p.x, p.y) for p in classification['poczatkowe']], color='limegreen'),
                          PointsCollection([(p.x, p.y) for p in classification['koncowe']], color='red'),
                          PointsCollection([(p.x, p.y) for p in classification['laczace']], color='mediumblue'),
                          PointsCollection([(p.x, p.y) for p in classification['dzielace']], color='lightsteelblue'),
                          PointsCollection([(p.x, p.y) for p in classification['prawidlowe']], color='sienna')],
                  lines=[LinesCollection(self.sides)]))
        return classified_vertices

    #rozwiazanie brutalne, w przypadku rozwiazania prepare_for_triangulation() wkradały
    #się błędy
    def n2_prepare_for_triangulation(self):
        classification = self.__classify_vertices()
        points = self.vertices.copy()
        edges = []
        newEdges = []
        for i in range(len(points)):
            edges.append([points[i - 1], points[i]])
        points.sort(key=(compareKey1), reverse=True)  
        edges.sort(key=(compareKey2))  
        sweepline=RedBlackTree()
        for vertex in points:
            if classification[vertex.point]=='prawidlowe':
                start=binary_searchleftmost(edges,vertex.point.y,0,len(edges)-1)
                for i in range(start,len(edges)):
                    if edges[i][0].point.x==vertex.point.x:
                        start=i
                        break
                if(edges[start][0].point.y>edges[start][1].point.y):
                    edge=sweepline.searchVertex(Sweepline(vertex,vertex,vertex))
                    if classification[edge.label.helper.point]=='laczace':
                        newEdges.append([edge.label.helper,edge.label.vertices[1]])
                    sweepline=sweepline.remove(edge.label)
                    sweepline=sweepline.insert(Sweepline(vertex,edges[start][0],edges[start][1]))
                else:
                    edge=sweepline.searchSweepline(Sweepline(vertex,vertex,vertex))
                    if classification[edge.label.helper.point]=='laczace':
                        newEdges.append([edge.label.helper,vertex])
                    edge.label.helper=vertex
            elif classification[vertex.point]=='poczatkowe':
                start=binary_searchleftmost(edges,vertex.point.y,0,len(edges)-1)
                for i in range(start,len(edges)):
                    if edges[i][0].point.x==vertex.point.x:
                        sweepline=sweepline.insert(Sweepline(vertex,edges[i][0],edges[i][1]))
                        break
            elif classification[vertex.point]=='koncowe':
                edge=sweepline.searchVertex(Sweepline(vertex,vertex,vertex))
                if classification[edge.label.helper.point]=='laczace':
                    newEdges.append([edge.label.helper,edge.label.vertices[1]])
                sweepline=sweepline.remove(edge.label)
            elif classification[vertex.point]=='dzielace':
                edge=sweepline.searchSweepline(Sweepline(vertex,vertex,vertex))
                newEdges.append([edge.label.helper,vertex])
                edge.helper=vertex
                start=binary_searchleftmost(edges,vertex.point.y,0,len(edges)-1)
                for i in range(start,len(edges)):
                    if edges[i][0].point.x==vertex.point.x:
                        sweepline=sweepline.insert(Sweepline(vertex,edges[i][0],edges[i][1]))
                        break
            elif classification[vertex.point]=='laczace':
                edge=sweepline.searchVertex(Sweepline(vertex,vertex,vertex))
                if classification[edge.label.helper.point]=='laczace':
                    newEdges.append([edge.label.helper,edge.label.vertices[1]])
                sweepline=sweepline.remove(edge.label)
                edge=sweepline.searchSweepline(Sweepline(vertex,vertex,vertex))
                if classification[edge.label.helper.point]=='laczace':
                    newEdges.append([edge.label.helper,edge.label.vertices[1]])
                edge.label.helper=vertex
        return newEdges

    def prepare_for_triangulation(self):
        classification = self.__classify_vertices()
        points = self.vertices.copy()
        edges = []
        newEdges = []
        for i in range(len(points)):
            edges.append([points[i - 1], points[i]])
        points.sort(key=(compareKey1), reverse=True)  
        edges.sort(key=(compareKey2))  
        sweepline = []  # miotla, na pozycji 0 ma pomocnika krawedzi, a na pozycji 1 krawedz
        for vertex in points:
            if classification[vertex.point]=='prawidlowe':
                start=binary_searchleftmost(edges,vertex.point.y,0,len(edges)-1)
                for i in range(start,len(edges)):
                    if edges[i][0].point.x==vertex.point.x:
                        start=i
                        break
                if(edges[start][0].point.y>edges[start][1].point.y):
                    for edge in sweepline:
                        if edge[1][1].point==vertex.point:
                            if classification[edge[0].point]=='laczace':
                                newEdges.append([edge[0],edge[1][1]])
                            sweepline.remove(edge)
                            break
                    sweepline.append([vertex,edges[start]])
                else:
                    if len(sweepline)==1:
                        if classification[sweepline[0][0].point]=='laczace':
                            newEdges.append([sweepline[0][0],vertex])
                        sweepline[0][0]=vertex
                    else:
                        indeks=0
                        distance=float('inf')
                        for i in range(len(sweepline)):
                            curr_dis=((sweepline[i][1][0].point.x-sweepline[i][1][1].point.x)/(sweepline[i][1][0].point.y-sweepline[i][1][1].point.y)*
                               (vertex.point.y-sweepline[i][1][1].point.y)+sweepline[i][1][1].point.x)
                            curr_dis-=vertex.point.x
                            curr_dis*=-1
                            if curr_dis>0 and curr_dis<distance:
                                distance=curr_dis
                                indeks=i
                        if classification[sweepline[indeks][0].point]=='laczace':
                            newEdges.append([sweepline[indeks][0],vertex])
                        sweepline[indeks][0]=vertex
            elif classification[vertex.point]=='poczatkowe':
                start=binary_searchleftmost(edges,vertex.point.y,0,len(edges)-1)
                for i in range(start,len(edges)):
                    if edges[i][0].point.x==vertex.point.x:
                        sweepline.append([vertex,edges[i]])
                        break
            elif classification[vertex.point]=='koncowe':
                for edge in sweepline:
                    if edge[1][1].point==vertex.point:
                        if classification[edge[0].point]=='laczace':
                            newEdges.append([edge[0],edge[1][1]])
                        sweepline.remove(edge)
                        break
            elif classification[vertex.point]=='dzielace':
                indeks=0
                distance=float('inf')
                for i in range(len(sweepline)):
                    curr_dis=((sweepline[i][1][0].point.x-sweepline[i][1][1].point.x)/(sweepline[i][1][0].point.y-sweepline[i][1][1].point.y)*
                       (vertex.point.y-sweepline[i][1][1].point.y)+sweepline[i][1][1].point.x)
                    curr_dis-=vertex.point.x
                    curr_dis*=-1
                    if curr_dis>0 and curr_dis<distance:
                        distance=curr_dis
                        indeks=i
                newEdges.append([vertex,sweepline[indeks][0]])
                sweepline[indeks][0]=vertex
                start=binary_searchleftmost(edges,vertex.point.y,0,len(edges)-1)
                for i in range(start,len(edges)):
                    if edges[i][0].point.x==vertex.point.x:
                        sweepline.append([vertex,edges[i]])
                        break
            elif classification[vertex.point]=='laczace':
                for edge in sweepline:
                    if edge[1][1].point==vertex.point:
                        if classification[edge[0].point]=='laczace':
                            newEdges.append([edge[0],edge[1][1]])
                        sweepline.remove(edge)
                if len(sweepline)==1:
                    if classification[sweepline[0][0].point]=='laczace':
                        newEdges.append([sweepline[0][0],sweepline[0][1][1]])
                    sweepline[0][0]=vertex
                else:
                    indeks=0
                    distance=float('inf')
                    for i in range(len(sweepline)):
                        curr_dis=((sweepline[i][1][0].point.x-sweepline[i][1][1].point.x)/(sweepline[i][1][0].point.y-sweepline[i][1][1].point.y)*
                           (vertex.point.y-sweepline[i][1][1].point.y)+sweepline[i][1][1].point.x)
                        curr_dis-=vertex.point.x
                        curr_dis*=-1
                        if curr_dis>0 and curr_dis<distance:
                            distance=curr_dis
                            indeks=i
                    if classification[sweepline[indeks][0].point]=='laczace':
                        newEdges.append([sweepline[indeks][0],sweepline[indeks][1][1]])
                    sweepline[indeks][0]=vertex
        self.scenes.append(Scene(lines=[LinesCollection(self.sides),
                                        LinesCollection([Point.to_line(v[0].point, v[1].point) for v in newEdges],
                                                        color='crimson')]))
        return newEdges

    def __partition_into_monotone_subpolygons(self, edges = None):
        if edges is None:
            edges = self.prepare_for_triangulation()
        vertices = {}
        vertices_index = {}
        for v in self.vertices:
            vertices[v] = []
        for i in range(len(self.vertices)):
            vertices_index[self.vertices[i]] = i
        for v1,v2 in edges:
            if vertices_index[v1] < vertices_index[v2]:
                vertices[v2].append(v1)
            else:
                vertices[v1].append(v2)
        for v in self.vertices:
            if vertices[v]:
                vertices[v].sort(key=lambda v:vertices_index[v])
        ver = self.vertices.copy()
        e = len(edges)
        subpolygons = []


        def new_subpolygon(t, index, ver, vertices, subpolygons, e):
            i = index
            j = index
            first_loop = True
            while(first_loop or (e>0 and ver[j] != t)):
                curr = ver[j]
                if first_loop:
                    first_loop = False
                if not vertices[ver[j]]:
                    j += 1
                else:
                    tmp = vertices[ver[j]][-1]
                    vertices[ver[j]].pop()
                    e, ver = new_subpolygon(ver[j], vertices_index[tmp], ver, vertices, subpolygons, e)
                    j -= len(subpolygons[-1].vertices) - 2
                    ss = j
                    while (ss != len(ver)):
                        vertices_index[ver[ss]] -= len(subpolygons[-1].vertices) - 2
                        ss += 1
            if e == 0:
                current_subpolygon_vertices = [] + ver
            else:
                current_subpolygon_vertices = ver[i:j+1]
            subpolygons.append(Polygon(current_subpolygon_vertices))
            return e-1, ver[:i+1] + ver[j:]
        new_subpolygon(ver[0], 0, ver, vertices, subpolygons, e)
        sd = []
        for sp in subpolygons:
            sd = [] + sd + sp.sides.copy()
            sp.parent = self
            self.scenes.append(Scene(lines = [LinesCollection(sp.sides.copy(), color = 'green')]))
            trl = []
            for tr in sp.triangles:
                trl += tr.to_list()
            self.scenes.append(Scene(lines = [LinesCollection(trl, color = 'crimson')]))
        return subpolygons


    def triangulate(self):
        if not self.__is_y_monotone():
            print("BLAD W __triangulate()")
        if self.__is_triangulated:
            return
        if len(self.vertices) < 3:
            return
        V = sorted([[self.vertices[i], self.chain[i]] for i in range(len(self.vertices))], key=lambda v: v[0].point.y)
        if len(V) == 3:
            self.add_triangle(Triangle(V[0][0], V[1][0], V[2][0]))
            return
        def belongs(i_p, i_q, i_r) -> bool:
            p = V[i_p][0].point
            q = V[i_q][0].point
            r = V[i_r][0].point

            if V[i_r][1] == 1:
                return Point.orientation(p, q, r) > 0
            else:
                return Point.orientation(p, q, r) < 0

        S = [0, 1]  # STACK
        n = len(V)

        for i in range(2, n):
            if V[i][1] == V[S[-1]][1] or V[i][1] == 0 or V[S[-1]][1] == 0:
                nS = []
                while len(S) > 1:
                    if belongs(S[-2], S[-1], i):
                        self.add_triangle(Triangle(V[i][0], V[S[-1]][0], V[S[-2]][0]))
                        S.pop()
                    else:
                        nS.append(S.pop())
                nS.append(S.pop())
                nS.reverse()
                nS.append(i)
                S = nS
            else:
                l = len(S)
                ve = S[-1]
                for j in range(l - 1):
                    self.add_triangle(Triangle(V[i][0], V[S[-1]][0], V[S[-2]][0]))
                    S.pop()  # v
                S = [ve, i]
        ts = []
        self.scenes.append(Scene(lines = [LinesCollection([tr.to_list()[i] for tr in self.triangles for i in range(3)], color = 'crimson')]))
        self.__is_triangulated = True


    def __triangulation(self, diagonals = None):
        if self.__is_triangulated:
            return
        if self.__is_y_monotone() and diagonals == None:
            self.triangulate()
        else:
            subpolygons = self.__partition_into_monotone_subpolygons(edges=diagonals)
            for sb in subpolygons:
                sb.__triangulation()
                self.sub_polygons.append(sb)
                if diagonals is not None:
                    for tr in list(sb.triangles):
                        self.add_triangle(tr, change = False)
                else:
                    for tr in list(sb.triangles):
                        self.add_triangle(tr)
            self.scenes += sb.scenes
        s = []
        for tr in list(self.triangles):
            s += tr.to_list()
        self.scenes.append(Scene(lines= [LinesCollection(self.sides), LinesCollection(s, color='yellow')]))
        self.__is_triangulated = True


    def actions(self, diagonals = None):
        if diagonals is not None:
            self.additional_diagonals = diagonals
        self.__triangulation(diagonals=diagonals)


    def to_scene(self, triangles = True, color = 'dodgerblue', color2 = 'dodgerblue'):
        if not triangles:
            return Scene(lines=[LinesCollection(self.sides, color=color)])
        triangles = []
        for tr in self.triangles:
            triangles += tr.to_list()
        return Scene(lines=[LinesCollection(triangles, color = color2), LinesCollection(self.sides, color = color)])
