# -*- coding:utf8 -*-

from numpy import *
from random import randint
import numpy as np, os
from matplotlib.mlab import normpdf

defaultpath = os.path.dirname(os.path.realpath(__file__))+"/"

def readReadings(pathData):
    a = np.genfromtxt(defaultpath+pathData+"coilLch0.csv", dtype=float, delimiter=',')
    b = np.genfromtxt(defaultpath+pathData+"coilLch1.csv", dtype=float, delimiter=',')
    c = np.genfromtxt(defaultpath+pathData+"coilLch2.csv", dtype=float, delimiter=',')
    coilL = np.concatenate((a[:,:,None],b[:,:,None],c[:,:,None]), axis=2)

    a = np.genfromtxt(defaultpath+pathData+"coilMch0.csv", dtype=float, delimiter=',')
    b = np.genfromtxt(defaultpath+pathData+"coilMch1.csv", dtype=float, delimiter=',')
    c = np.genfromtxt(defaultpath+pathData+"coilMch2.csv", dtype=float, delimiter=',')
    coilM = np.concatenate((a[:,:,None],b[:,:,None],c[:,:,None]), axis=2)

    a = np.genfromtxt(defaultpath+pathData+"coilRch0.csv", dtype=float, delimiter=',')
    b = np.genfromtxt(defaultpath+pathData+"coilRch1.csv", dtype=float, delimiter=',')
    c = np.genfromtxt(defaultpath+pathData+"coilRch2.csv", dtype=float, delimiter=',')
    coilR = np.concatenate((a[:,:,None],b[:,:,None],c[:,:,None]), axis=2)

    return coilL, coilM, coilR

def modifyObject(Obj, m):
    
    r=randint(0,5)
    if r<4:
        modObj=np.rot90(Obj[0],r),np.rot90(Obj[1],r),np.rot90(Obj[2],r)
    elif r==4:
        modObj=np.flipud(Obj[0]),np.flipud(Obj[1]),np.flipud(Obj[2])
    else:
        modObj=np.fliplr(Obj[0]),np.fliplr(Obj[1]),np.fliplr(Obj[2])


    h,w = (modObj[0].T)[0].shape

    xf=w/2
    xi=xf-w
    yf=h/2
    yi=yf-h

    x0, y0 = m
    xi += x0
    xf += x0
    yi += y0
    yf += y0

#    return modObj, h, w
    return modObj, xi,xf,yi,yf

def checkObject(mask, xi, xf, yi, yf):
    return mask[yi:yf,xi:xf].max() == 0

def placeObject(Mat, Obj, xi, xf, yi, yf):
    for coil in range(3):
        for channel in range(3):
            Mat[coil*3+channel,yi:yf,xi:xf] = (Obj[coil].T)[channel]

def GenerateUsingRealDataset(posMines, posMetals1, posMetals2, width, height, flag):

    mask = zeros((height,width))
    mat = ones((9,height,width))
    mat[0]=mat[0]*-156900.0
    mat[1]=mat[1]*-139615.0
    mat[2]=mat[2]*-103957.0
    mat[3]=mat[3]*-155981.0
    mat[4]=mat[4]*-129075.0
    mat[5]=mat[5]*-98482.0
    mat[6]=mat[6]*-213120.0
    mat[7]=mat[7]*-188696.0
    mat[8]=mat[8]*-98583.0

    zeroChannel=zeros(9)
    zeroChannel[0]=-153452
    zeroChannel[1]=-135734
    zeroChannel[2]=-103976
    zeroChannel[3]=-161483
    zeroChannel[4]=-132306
    zeroChannel[5]=-98991
    zeroChannel[6]=-209630
    zeroChannel[7]=-184916
    zeroChannel[8]=-98584

    actualPosMines=[]
    actualPosMetals1=[]
    actualPosMetals2=[]

    mine01  = readReadings("Mine01/")
    numValidMines=0
    metal01 = readReadings("Metal01/")
    numValidMetals1=0
    metal02 = readReadings("Metal02/")
    numValidMetals2=0

    h,w = (mine01[0].T)[1].T.shape
    for m in posMines:
        obj,xi,xf,yi,yf=modifyObject(mine01,m)
        if xi<0 or xf>=width or yi<0 or yf>=height:
            continue
        if mask[yi:yf,xi:xf].max() == 0:
            placeObject(mat,obj,xi,xf,yi,yf)
            mask[yi:yf,xi:xf] += 1
            actualPosMines.append(m)
            numValidMines += 1
    if flag==True:
        count = 0
        while numValidMines < len(posMines) and count < 50:
            m = [randint(0,width),randint(0,height)]
            obj,xi,xf,yi,yf=modifyObject(mine01,m)
            if xi<0 or xf>=width or yi<0 or yf>=height:
                continue
            if mask[yi:yf,xi:xf].max() == 0:
                placeObject(mat,obj,xi,xf,yi,yf)
                mask[yi:yf,xi:xf] += 1
                actualPosMines.append(m)
                numValidMines += 1
            count += 1

    h,w = (metal01[0].T)[1].T.shape
    for m in posMetals1:
        obj,xi,xf,yi,yf=modifyObject(metal01,m)
        if xi<0 or xf>=width or yi<0 or yf>=height:
            continue
        if mask[yi:yf,xi:xf].max() == 0:
            placeObject(mat,obj,xi,xf,yi,yf)
            mask[yi:yf,xi:xf] += 1
            actualPosMetals1.append(m)
            numValidMetals1 += 1
    if flag==True:
        count = 0
        while numValidMetals1 < len(posMetals1) and count < 50:
            m = [randint(0,width),randint(0,height)]
            obj,xi,xf,yi,yf=modifyObject(metal01,m)
            if xi<0 or xf>=width or yi<0 or yf>=height:
                continue
            if mask[yi:yf,xi:xf].max() == 0:
                placeObject(mat,obj,xi,xf,yi,yf)
                mask[yi:yf,xi:xf] += 1
                actualPosMetals1.append(m)
                numValidMetals1 += 1
            count += 1

    h,w = (metal02[0].T)[1].T.shape
    for m in posMetals2:
        obj,xi,xf,yi,yf=modifyObject(metal02,m)
        if xi<0 or xf>=width or yi<0 or yf>=height:
            continue
        if mask[yi:yf,xi:xf].max() == 0:
            placeObject(mat,obj,xi,xf,yi,yf)
            mask[yi:yf,xi:xf] += 1
            actualPosMetals2.append(m)
            numValidMetals2 += 1
    if flag==True:
        count = 0
        while numValidMetals2 < len(posMetals2) and count < 50:
            m = [randint(0,width),randint(0,height)]
            obj,xi,xf,yi,yf=modifyObject(metal02,m)
            if xi<0 or xf>=width or yi<0 or yf>=height:
                continue
            if mask[yi:yf,xi:xf].max() == 0:
                placeObject(mat,obj,xi,xf,yi,yf)
                mask[yi:yf,xi:xf] += 1
                actualPosMetals2.append(m)
                numValidMetals2 += 1
            count += 1

    for m in range(len(posMines)):
        posMines.pop()
    for m in actualPosMines:
        posMines.append(m)

    return mat, zeroChannel

def MineMapGenerator(posMinas, width, height, var=10, noise=0.1, scale=100):
    mat = zeros((height,width))
    xx = arange(width)
    yy = arange(height)

    def gauss2D(pt):
        x0, y0 = pt
        g1 = matrix(normpdf(xx,x0,var))
        g2 = matrix(normpdf(yy,y0,var)).T
        z = (g2*g1).A1.reshape(height,width)
        return z

    #Cria as gaussinas de sinal de cada mina
    maxValue = None
    for m in posMinas:
        mat += gauss2D(m)
        if (maxValue == None):
            maxValue = mat.max()

    mat /= maxValue
    return mat*scale

if __name__ == "__main__":
    from random import randint
    from matplotlib import cm, pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    w,h = 400,400
    nMinas = 7 #Numero de minas a ser espalhadas
    nMetais1 = 3
    nMetais2 = 4

    posMinas = [] #Pode-se setar algumas posições de minas manualmente aqui.
    posMetais1 = []
    posMetais2 = []

    #Pode-se setar a posição das minas manualmente para não deixar aleatoriedade
    for i in range(nMinas):
        posMinas.append([randint(0,w),randint(0,h)])

    for i in range(nMetais1):
        posMetais1.append([randint(0,w),randint(0,h)])

    for i in range(nMetais2):
        posMetais2.append([randint(0,w),randint(0,h)])

#    mineMap = MineMapGenerator(posMinas,w,h)
    mineMap, zeroChannels = GenerateUsingRealDataset(posMinas,posMetais1,posMetais2,w,h,True)

    #Apresenta resultados
    plt.imshow(mineMap[0])
    plt.colorbar()
    plt.show()
