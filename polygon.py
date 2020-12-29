def compareKey1(key):
    return key.point.y;
def compareKey2(key):
    return key[0].point.y;
def binary_searchleftmost(arr,val,left,right): 
    if left==right:
        if arr[left][0].point.y==val:
            return left
        else:
            return -1
    (mid) = (int)((left+right)/2)
    if arr[mid][0].point.y < val:
        return binary_searchleftmost(arr,val,mid+1,right);
    else:
        return binary_searchleftmost(arr,val,left,mid);
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __eq__(self, other):
        return isinstance(other, Point) and self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))
    def equals(self,other):
        return isinstance(other, Point) and self.x == other.x and self.y == other.y

    @staticmethod
    def det(p, q, r):
        return (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)

    @staticmethod
    def orientation(p, q, r, epsilon=0):
        d = Point.det(p, q, r)
        if d > epsilon:
            return 1  # "clockwise" #lewa
        elif d < (-1) * epsilon:
            return -1  # "counterclockwise" #prawa
        else:
            return 0  # "collinear"
class Vertex:
    def __init__(self, point: Point):
        self.point = point
        self.triangles = []

    def add_triangle(self, triangle):
        self.triangles.append(triangle)

    #zastanowić się nad tym czy powinno się jeszcze triangles porownywac
    def __eq__(self, other):
        return isinstance(other, Vertex) and self.point == other.point

    def __hash__(self):
        return hash(self.point)
class Polygon:
    def __init__(self, vertices): #wierzchołki są zadawane w lewa strone
        self.vertices = self.__sorted_vertices(vertices)
        self.bottom_point_index = 0 #index
        self.top_point_index = self.__top_point() #index
        self.triangles = set()
        self.chain = self.__get_chain()
        self.scenes = [] #puste
        self.sides = []
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
        return vertices[i:] + vertices[:i]

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
        C[0] = 0
        C[self.top_point_index] = 0

    def add_triangle(self, triangle):
        self.triangles.add(triangle)

    def __is_y_monotone(self):
        V = self.vertices
        for i in range(self.top_point_index, len(V)):
            if V[(i+1) % len(V)].point.y > V[i % len(V)].point.y:
                return False
        for i in range(self.top_point_index):
            if V((i+1) % len(V)).point.y < V[i % len(V)].point.y:
                return False
        return True

    def classify_vertices(self, epsilon = 0):
        #kolory opisane ponizej do pozniejszej wizualizacji
        classification = {}
        V= self.vertices
        n = len(V)

        for i in range(n):
            p, q, r = V[(i-1) % n].point, V[i % n].point, V[(i+1) % n].point
            diff = (p.y - q.y, r.y - q.y)
            d = Point.det(p, q, r)
            if diff[0] > 0 and diff[1] > 0:  # oba punkty są powyzej
                if d > epsilon:  # phi > pi / clockwise
                    classification[q]='laczace'
                    continue
                elif d < epsilon:  # phi < pi / counterclockwise
                    classification[q]='koncowe'
                    continue
            elif diff[0] < 0 and diff[1] < 0:  # oba punkty są ponizej
                if d > epsilon:  # phi > pi
                    classification[q]='dzielace'
                    continue
                elif d < epsilon:  # phi < pi
                    classification[q]='poczatkowe'
                    continue
            else:  # prawidlowy
                classification[q]='prawidlowe'
        return classification;
    def PrepareForTriangulation(self):
        classification=self.classify_vertices(self)
        points=self.vertices.copy()
        edges=[]
        newEdges=[]
        for i in range(len(points)):
            edges.append([points[i-1],points[i]])
        points.sort(key=(compareKey1),reverse=True)
        edges.sort(key=(compareKey2))
        broom=[]#miotla, na pozycji 0 ma pomocnika krawedzi, a na pozycji 1 krawedz
        for point in points:
            if classification[point.point]=='prawidlowe':
                start=binary_searchleftmost(edges,point.point.y,0,len(edges)-1)
                if(start==-1):
                    print("Prepare for triangulation error")
                    return None
                for i in range(start,len(edges)):
                    if edges[i][0].point.x==point.point.x:
                        start=i
                        break
                if(edges[start][0].point.y>edges[start][1].point.y):
                    for edge in broom:
                        if edge[1][1].point.equals(point.point):
                            if classification[edge[0].point]=='laczace':
                                newEdges.append([edge[0],edge[1][1]])
                        broom.remove(edge)
                        break
                    broom.append([point,edges[start]])
                else:
                    if len(broom)==1:
                        if classification[broom[0][0].point]=='laczace':
                            newEdges.append([broom[0][0],broom[0][1][1]])
                        broom[0][0]=point
                    else:
                        indeks=0
                        distance=float('inf')
                        for i in range(len(broom)):
                            curr_dis=((broom[i][1][0].point.x-broom[i][1][1].point.x)/(broom[i][1][0].point.y-broom[i][1][1].point.y)*
                               (point.point.y-broom[i][1][1].point.y)+broom[i][1][1].point.x)
                            curr_dis-=point.point.x
                            curr_dis*=-1
                            type(curr_dis)
                            if curr_dis>0 and curr_dis<distance:
                                distance=curr_dis
                                indeks=i
                        if classification[broom[indeks][0].point]=='laczace':
                            newEdges.append([broom[indeks][0],broom[indeks][1][1]])
                        broom[indeks][0]=point
            elif classification[point.point]=='poczatkowe':
                start=binary_searchleftmost(edges,point.point.y,0,len(edges)-1)
                if(start==-1):
                    print("Prepare for triangulation error")
                    return None
                for i in range(start,len(edges)):
                    if edges[i][0].point.x==point.point.x:
                        broom.append([point,edges[i]])
                        break
            elif classification[point.point]=='koncowe':
                for edge in broom:
                    if edge[1][1].point.equals(point.point):
                        if classification[edge[0].point]=='laczace':
                            newEdges.append([edge[0],edge[1][1]])
                        broom.remove(edge)
                        break
            elif classification[point.point]=='dzielace':
                indeks=0
                distance=float('inf')
                for i in range(len(broom)):
                    curr_dis=((broom[i][1][0].point.x-broom[i][1][1].point.x)/(broom[i][1][0].point.y-broom[i][1][1].point.y)*
                       (point.point.y-broom[i][1][1].point.y)+broom[i][1][1].point.x)
                    curr_dis-=point.point.x
                    curr_dis*=-1
                    type(curr_dis)
                    if curr_dis>0 and curr_dis<distance:
                        distance=curr_dis
                        indeks=i
                newEdges.append([point,broom[indeks][0]])
                broom[indeks][0]=point
                start=binary_searchleftmost(edges,point.point.y,0,len(edges)-1)
                if(start==-1):
                    print("Prepare for triangulation error")
                    return None
                for i in range(start,len(edges)):
                    if edges[i][0].point.x==point.point.x:
                        broom.append([point,edges[i]])
                        break
            elif classification[point.point]=='laczace':
                for edge in broom:
                    if edge[1][1].point.equals(point.point):
                        if classification[edge[0].point]=='laczace':
                            newEdges.append([edge[0],edge[1][1]])
                        broom.remove(edge)
                if len(broom)==1:
                    if classification[broom[0][0].point]=='laczace':
                        newEdges.append([broom[0][0],broom[0][1][1]])
                    broom[0][0]=point
                else:
                    indeks=0
                    distance=float('inf')
                    for i in range(len(broom)):
                        curr_dis=((broom[i][1][0].point.x-broom[i][1][1].point.x)/(broom[i][1][0].point.y-broom[i][1][1].point.y)*
                           (point.point.y-broom[i][1][1].point.y)+broom[i][1][1].point.x)
                        curr_dis-=point.point.x
                        curr_dis*=-1
                        type(curr_dis)
                        if curr_dis>0 and curr_dis<distance:
                            distance=curr_dis
                            indeks=i
                    if classification[broom[indeks][0].point]=='laczace':
                        newEdges.append([broom[indeks][0],broom[indeks][1][1]])
                    broom[indeks][0]=point
#        for newEdge in newEdges:
 #           print(newEdge[0].point.x)
  #          print(newEdge[0].point.y) 
   #         print(newEdge[1].point.x)
    #        print(newEdge[1].point.y)
     #       print("\n")
        return newEdges
