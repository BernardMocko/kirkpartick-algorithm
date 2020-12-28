def compareKey1(key):
    return key.y;
def compareKey2(key):
    return key[1].y;
def binary_searchleftmost(arr,val,left,right): 
    if left==right:
        if arr[left]==val:
            return left
        else:
            return -1
    mid = (int)(left+right)/2
    if a[mid] < val:
        return binsearch(arr,val,mid+1,right);
    else:
        return binsearch(arr,val,left,mid);
def  binary_searchrightmost(arr,val, left, right):
    
    if left==right:
        if arr[left]==val:
            return left;
        else:
            return -1
    mid = (int)(left+right+1)/2
    if a[mid] < val:
        return binsearch(arr,val,mid+1,right)
    else:
        return binsearch(arr,val,left,mid)
class Polygon:
    def __init__(self, vertices): #wierzchołki są zadawane w lewa strone
        self.vertices = self.__sorted_vertices(vertices)
        self.bottom_point_index = 0 #index
        self.top_point_index = self.__top_point() #index
        self.triangles = set()
        self.chain = self.__get_chain()

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

    def __get__chain(self):
        C = [1 for _ in range(len(self.vertices))]
        i = 0
        while self.vertices[i] != self.vertices[self.top_point_index]:
            C[i] = -1
            i += 1
        C[0] = 0
        C[self.top_point_index] = 0

    def __add_triangle(self, triangle):
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

    #moze warto bedzie tu dodac epsilon
    def __classify_vertices(self, epsilon = 0):
        #kolory opisane ponizej do pozniejszej wizualizacji
        classification = {}
        V = self.vertices
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
    def __PrepareForTriangulation(self,classification):
        points=self.vertices.copy()
        edges=[]
        for i in range(len(points)):
            edges.append([points[i-1],points[i]])
        points.sort(compareKey1)
        edges.sort(compareKey2);
        helpers=[]#pomocnicy
        broom=[]#miotla
        for point in points:
            if classification[point]=='prawidlowe':
                
            elif classification[point]=='poczatkowe':
                helpers.append(point)
                miotla.append()
            elif classification[point]=='koncowe':
                
            elif classification[point]=='dzielace':
                
            elif classification[point]=='laczace':
