# 
# WireX
#
# Copyright (c) 2006-2019 Andreas Pott
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# 
#-------------------------------------------------------------------------------
# Name:        wcStl
# Purpose:     Load workspace stl files and generate 3D plot from the vertices
#              of an stl file.
#
# Author:      asp
#
# Created:     23.08.2015
#-------------------------------------------------------------------------------
#!/usr/bin/env python

from mpl_toolkits.mplot3d import axes3d
import matplotlib
import matplotlib.colors as colors
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import numpy as np
from struct import unpack

class wcStl:
    """
    Load and process STL data files wrtitten by WireCenter. The typical use is
    to make plots of workspace objects.
    Provide helper functions to generate 3D hull plots of the workspaec.
    """

    def __init__(self):
        """
        Store the default values here rather than using local variabes
        """
        self.center = [0,0,1]
        self.filename = 'workspace.eps'

        
    def workspaceSTLplot(self,v1,v2,v3,p):
        """
        Generate a 3D plot from the vertices (v1,v2,v3) of an stl file. The function
        is intended to generate workspace diagrams from exported stl files.
        """
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax = axes3d.Axes3D(fig)
        # add the triangles
        for i in range(0,v1.shape[0]):
            # compute a triangle from the vertices
            vtx = [v1[i].tolist(),v2[i].tolist(),v3[i].tolist()]
            tri = axes3d.art3d.Poly3DCollection([vtx])
            tri.set_color('g')
            tri.set_edgecolor('k')
            ax.add_collection3d(tri)

        ax.set_xlabel('$x$ [m]')
        ax.set_ylabel('$y$ [m]')
        ax.set_zlabel('$z$ [m]')

        ax.set_xlim3d(p[:,0].min(), p[:,0].max())
        ax.set_ylim3d(p[:,1].min(), p[:,1].max())
        ax.set_zlim3d(p[:,2].min(), p[:,2].max())
        plt.savefig(self.filename)


    def workspaceSTLplot2(self, vertex, index):
        """
        Generate a 3D plot from the a vertices and index data model. The 
        function is intended to generate workspace diagrams where the internal
        data model of WireLib::CWorksapceHull is used. 
		returns the generated figure
        """
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax = axes3d.Axes3D(fig)
        # add the triangles
        for (i,j,k) in index:
            # compute a triangle from the vertices
#            vtx = [v1[i].tolist(),v2[i].tolist(),v3[i].tolist()]
            vtx = [vertex[i], vertex[j], vertex[k] ]
            tri = axes3d.art3d.Poly3DCollection([vtx])
            tri.set_color('g')
            tri.set_edgecolor('k')
            ax.add_collection3d(tri)

        ax.set_xlabel('$x$ [m]')
        ax.set_ylabel('$y$ [m]')
        ax.set_zlabel('$z$ [m]')

        # this command converts the list of tuple to a numpy matrix object
        ver = np.array(list(set(tuple(v) for v in vertex)))

        ax.set_xlim3d(ver[:,0].min(), ver[:,0].max())
        ax.set_ylim3d(ver[:,1].min(), ver[:,1].max())
        ax.set_zlim3d(ver[:,2].min(), ver[:,2].max())

        return fig

		
    def workspaceSTLDiffPlot(self,v1,v2,v3,p, dv1,dv2,dv3,dp, bDrawHull):
        """
        Generate a 3D plot from the vertices (v1,v2,v3) of an stl file. The function
        is intended to generate workspace diagrams from exported stl files.
        """
        scale = 5
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax = axes3d.Axes3D(fig)
        # add the triangles from the first hull
        if bDrawHull:
            for i in range(0,v1.shape[0]):
                # compute a triangle from the vertices
                vtx = [v1[i].tolist(),v2[i].tolist(),v3[i].tolist()]
                tri = axes3d.art3d.Poly3DCollection([vtx])
                tri.set_alpha(0.1)     # however, alpha values do not show up in eps
                tri.set_color('g')
                tri.set_edgecolor('k')
                ax.add_collection3d(tri)

        # before plotting the diff normals, we have to determine the length and
        # auto tune the scaling.
        lam = 0 # the maximum length of a ray in the plot
        diffmax = -1000 # the shortest diff
        diffmin = 1000  # the largest diff
        for (v,dv) in map(None, v1, dv1):
            v0 = (v-self.center)/np.linalg.norm(v-self.center)
            lam = max(lam,np.linalg.norm(v-self.center))

            diffmax = max(diffmax,np.dot(v0,(dv-v)))
            diffmin = min(diffmin,np.dot(v0,(dv-v)))
        print lam, diffmin,diffmax
        scale = lam/(diffmax-diffmin)/3     # rescale the diff legnth to 1/3 of the workspace radius

        # add the normals showing the difference between the first and the second hull
        for (v,dv) in map(None, v1, dv1):
            vtx = [v.tolist(),((dv-v)*scale+v).tolist()]
            tri = axes3d.art3d.Poly3DCollection([vtx])
            # compute the differential length in the interval -1,1
            v0 = (v-self.center)/np.linalg.norm(v-self.center)
            l= np.dot(v0,(dv-v)) / max(diffmax,-diffmin)
            tri.set_linewidth(4)
            tri.set_edgecolor(cm.coolwarm((l+1)/2))
            ax.add_collection3d(tri)

        for (v,dv) in map(None, v2, dv2):
            vtx = [v.tolist(),((dv-v)*scale+v).tolist()]
            tri = axes3d.art3d.Poly3DCollection([vtx])
            # compute the differential length in the interval -1,1
            v0 = (v-self.center)/np.linalg.norm(v-self.center)
            l= np.dot(v0,(dv-v)) / max(diffmax,-diffmin)
            tri.set_linewidth(2)
            tri.set_edgecolor(cm.coolwarm((l+1)/2))
            ax.add_collection3d(tri)

        ax.set_xlabel('$x$ [m]')
        ax.set_ylabel('$y$ [m]')
        ax.set_zlabel('$z$ [m]')

        ax.set_xlim3d(p[:,0].min(), p[:,0].max())
        ax.set_ylim3d(p[:,1].min(), p[:,1].max())
        ax.set_zlim3d(p[:,2].min(), p[:,2].max())
        plt.savefig('dworkspace2.eps')


    def workspaceStlLengthHistogram(self,p):
        """
        Create a histogram of the length of vertices in the Stl file (measured
        against self.center)
        """
        fig = plt.figure()
        length = np.zeros(p.shape[0])
        for i in range(0,p.shape[0]):
            length[i] = np.linalg.norm(p[i]-self.center)

        next, bins, patches = plt.hist(length,20,normed=1, histtype='bar', rwidth=0.8, log=True) #) # use for logarithmic scale in the histo
        plt.setp(patches,'facecolor', 'b')#, 'alpha', 0.75)
        plt.grid(True)
        plt.show()
        plt.close()


    def load(self,filename):
        """
        Load the STL file with filename and return a tuple (Header,Points,
        Normals,Vertex1,Vertex2,Vertex3) with the content of the file. The data
        format is the array type of numpy.
        """
        fp = open(filename, 'rb')
        Header = fp.read(80)
        nn = fp.read(4)
        Numtri = unpack('i', nn)[0]
        #print nn
        record_dtype = np.dtype([
                       ('normals', np.float32,(3,)),
                       ('Vertex1', np.float32,(3,)),
                       ('Vertex2', np.float32,(3,)),
                       ('Vertex3', np.float32,(3,)) ,
                       ('atttr', '<i2',(1,) )
        ])
        data = np.fromfile(fp , dtype = record_dtype , count =Numtri)
        fp.close()

        Normals = data['normals']
        Vertex1= data['Vertex1']
        Vertex2= data['Vertex2']
        Vertex3= data['Vertex3']

        p = np.append(Vertex1,Vertex2,axis=0)
        p = np.append(p,Vertex3,axis=0) #list(v1)
        Points =np.array(list(set(tuple(p1) for p1 in p)))

        return Header,Points,Normals,Vertex1,Vertex2,Vertex3

# unit testing
if __name__ == '__main__':
    stl = wcStl()
    # set the IEEE paper plaza-required trueType fonts rather than the default type 3 fonts
#    matplotlib.rcParams['pdf.fonttype'] = 42
#    matplotlib.rcParams['ps.fonttype'] = 42
    head,p,n,v1,v2,v3 = stl.load("endless9_TOWS_c_infty.stl") # "IPAneam1_TOWS_Dykstra_ab_5deg.stl")
    
    print "Content of STL header: ", head
    print "Length of STL file (number of vertices):  ", p.shape[0]
    print "Length of STL file (number of triangles): ", v1.shape[0]
    head,dp,dn,dv1,dv2,dv3 = stl.load('dworkspace.stl')
#    stl.workspaceSTLplot(v1,v2,v3,p)
#    stl.workspaceSTLDiffPlot(v1,v2,v3,p,dv1,dv2,dv3,dp, True)
    stl.workspaceStlLengthHistogram(p)
