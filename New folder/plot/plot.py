# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import sys
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.axes_grid1 import make_axes_locatable
import matplotlib.pyplot as plt
import numpy as np
import os
import warnings
warnings.filterwarnings('ignore') #hide the warnning when "nan" involves
import collections
import importlib
importlib.reload(sys)
#ys.setdefaultencoding('utf-8')

plt.style.use('ggplot')
#plt.rc('text', usetex=True)
plt.rcParams['font.sans-serif']=['SimHei'] #用来正常显示中文标签
plt.rcParams['axes.unicode_minus']=False #用来正常显示负号
plt.rc('font', family='sans-serif')
plt.rcParams['text.latex.preamble']=[r"\usepackage{amsmath}"]

path = "data/"
outpath = "out/"

class BEMPlot:
    
    def __init__(self, filename, M, N):
        self.filename = filename
        self.lineList = []
        tmpMesh = []
        with open(path + self.filename + '-mesh.txt', 'r') as f:
            for line in f:
                if line.strip()=="":
                    if len(tmpMesh)!=0:
                        self.lineList.append(np.array(tmpMesh));
                        tmpMesh = []
                else:
                    index = line.strip().split(' ')
                    tmpMesh.append([float(index[0]),float(index[1])])
            if len(tmpMesh)!=0:
                self.lineList.append(np.array(tmpMesh));
        #self.Vert = np.loadtxt(path + self.filename + '-mesh.txt',dtype = np.float64).reshape(-1,2)
        self.X = np.loadtxt(path + self.filename + '-x.txt',dtype = np.float64).reshape(N,M)
        self.Y = np.loadtxt(path + self.filename + '-y.txt',dtype = np.float64).reshape(N,M)
        z = np.loadtxt(path + self.filename + '-z.txt',dtype = np.float64).reshape(N,M,3)
        self.P = z[:,:,0]
        self.U = z[:,:,1]
        self.V = z[:,:,2]
        self.pmin = np.nanmin(self.P)
        self.pmax = np.nanmax(self.P)
        self.xmin = self.X[0,0]
        self.xmax = self.X[N-1,M-1]
        self.ymin = self.Y[0,0]
        self.ymax = self.Y[N-1,M-1]
        self.Norm = np.sqrt(self.U**2+self.V**2)
        self.default_camp =plt.cm.coolwarm;
    
    def _getLevelList(self, p,n):
        flatten = p.reshape(-1)
        values = flatten[~np.isnan(flatten)]
        ordered = np.sort(values)
        counter = collections.Counter(ordered);
        currentNum=0;
        gap = int(len(values)/n);
        ids = []
        ids.append(ordered.min())
        for key in counter:
            if currentNum>gap:
                ids.append(key)
                currentNum=0
            currentNum+=counter[key]
        if currentNum>gap:
            ids.append(ordered.max())
        #ids = values[(np.arange(n)*len(values)/n).astype(int)]
        #print(ids)
        #news_ids = np.unique(ids)
        return ids #news_ids
    
    def _addMesh(self, ax):
        for line in self.lineList:
            xx = np.r_[line[:,0]]
            yy = np.r_[line[:,1]]
            ax.plot(xx, yy, marker='o', color='b')
    
    ####### Batch Matplotlib Plot #######
        
    def allMatplotlibPlot(self, showWindow, saveFile, extension):
        self.batchMatplotlibPlot(["field","streamline","quiver","surface"], showWindow, saveFile, extension)
    
    def batchMatplotlibPlot(self, arr, showWindow, saveFile, extension):
        if(not(os.path.exists(outpath))):
            os.mkdir(outpath)
        for item in arr:
            if item == "field":
                self.plotField();
            elif item == "streamline":
                self.plotStreamLine()
            elif item == "quiver":
                self.plotQuiver()
            elif item == "surface":
                self.plotSurface()
            #plt.ion()
            if saveFile:
                plt.savefig(outpath + self.filename + "-" + item + "." + extension, bbox_inches='tight')
        
    ####### Single Matplotlib Plot #######
    
    def plotField(self):
        fig, ax = plt.subplots()
        ax.set_aspect(1)
        #ax.set_title(u'势场等势图')
        self._addMesh(ax)
        extent=(self.xmin,self.xmax,self.ymin,self.ymax)
        im=plt.imshow(self.P,vmin=self.pmin,vmax=self.pmax,extent=extent,origin='lower',cmap=self.default_camp)
        levels = self._getLevelList(self.P,20)
        ax.contour(self.P, levels, colors='k',origin='lower',extent=extent,linewidths=0.5)
        divider = make_axes_locatable(ax)
        cax2=divider.append_axes("right", "10%", pad=0.15)
        plt.colorbar(im, cax=cax2,format="%.2f")
    
    def plotStreamLine(self):
        fig, ax = plt.subplots()
        ax.set_aspect(1)
        #ax.set_title(u'势场流线图')
        self._addMesh(ax)
        strm=ax.streamplot(self.X, self.Y, self.U, self.V, color=self.Norm)
        im=strm.lines
        divider = make_axes_locatable(ax)
        cax3=divider.append_axes("right", "10%", pad=0.15)
        plt.colorbar(im, cax=cax3,format="%.2f")
        
    def plotQuiver(self):
        fig, ax = plt.subplots()
        ax.set_aspect(1)
        ax.set_title(u'Quiver')
        self._addMesh(ax)
        ax.quiver(self.X, self.Y, self.U/self.Norm, self.V/self.Norm)
        make_axes_locatable(ax)
    
    def plotSurface(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        #ax.set_title(u'势场三维图')
        #p_copy = p.copy()
        #p_copy[p_copy<99.9]=np.nan
        ax.plot_surface(self.X, self.Y, self.P, edgecolor='none', alpha=1, cstride=1, rstride=1, vmin=self.pmin, vmax=self.pmax,cmap=self.default_camp, linewidth=0, antialiased=False)
        ax.contour(self.X, self.Y, self.P, levels = self._getLevelList(self.P,10), colors='k', linewidths=0.5)


def main():
    if len(sys.argv)>3:
        bem_plot = BEMPlot(sys.argv[1],int(sys.argv[2]),int(sys.argv[3]))
        bem_plot.allMatplotlibPlot(True,True,"pdf")
    else:
        print("args not enough")
 
if __name__ == "__main__":
    main()
    