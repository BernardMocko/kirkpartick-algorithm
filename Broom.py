from projekt import *
from projekt.Vertex_class import Vertex

class Broom:
    def __init__(self, helper: Vertex, v1: Vertex, v2:Vertex):
        self.vertices = (v1, v2)
        self.helper=helper
    def __eq__(self, other):
        return self.vertices[0] == other
    def __gt__(self, other):
        if not isinstance(other,Broom):
            return False
        minY=self.vertices[0].point.y
        minX=self.vertices[0].point.x
        maxY=other.vertices[0].point.y
        maxX=other.vertices[0].point.x
        oo=0
        if other.vertices[1].point.y<minY:
            minY,maxY=maxY,minY
            minX,maxX=maxX,minX
            oo=1
        newX=0
        if oo==0:
            newX=((maxX-self.vertices[1].point.x)/(maxY-self.vertices[1].point.y)*
               (minY-self.vertices[1].point.y)+self.vertices[1].point.x)
        else:
            newX=((maxX-other.vertices[1].point.x)/(maxY-other.vertices[1].point.y)*
               (minY-other.vertices[1].point.y)+other.vertices[1].point.x)
        if newX-minX>0:
            return True
        return False
