from ConfigParser import ConfigParser
from minemapgenerator import MineMapGenerator, GenerateUsingRealDataset
from numpy import *
import os, random

defaultpath = os.path.dirname(os.path.realpath(__file__))+"/../"

class Config(object):

    def __init__(self):
        self.load()


    def save(self,path=defaultpath+"config.ini"):
        cfile = open(path,"w")

        configFile = ConfigParser()

        configFile.add_section("MapDimensions")
        configFile.set("MapDimensions","width",self.mapWidth)
        configFile.set("MapDimensions","height",self.mapHeight)
        configFile.set("MapDimensions","cellwidth",self.cellWidth)
        configFile.set("MapDimensions","cellheight",self.cellHeight)

        configFile.add_section("Mines")
        configFile.set("Mines","numMines",self.numMines)
        configFile.set("Mines","RandomMines",self.randomMines)
        self.minesFixedPos = ""
        if not self.randomMines:
            self.minesFixedPos = "|".join( ",".join(str(v) for v in r) for r in self.mines)

        configFile.set("Mines","MinesPositions",self.minesFixedPos)

        configFile.set("Mines","DetectionMinDist",self.minDistDetection)
        configFile.set("Mines","ExplosionMaxDist",self.maxDistExplosion)
        configFile.write(cfile)

        self.generateMines()

        cfile.close()


    def load(self,path=defaultpath+"config.ini"):
        configFile = ConfigParser()
        configFile.read(path)
        self.mapWidth = configFile.getfloat("MapDimensions","width")
        self.mapHeight = configFile.getfloat("MapDimensions","height")
        self.cellWidth = configFile.getfloat("MapDimensions","cellwidth")
        self.cellHeight = configFile.getfloat("MapDimensions","cellheight")

        self.numMines = configFile.getint("Mines","numMines")
        self.randomMines = configFile.getboolean("Mines","RandomMines")

        self.minDistDetection = configFile.getfloat("Mines","DetectionMinDist")
        self.maxDistExplosion = configFile.getfloat("Mines","ExplosionMaxDist")

        self.numCellsX  = self.mapWidth/self.cellWidth
        self.numCellsY  = self.mapHeight/self.cellHeight
        self.numCells   = self.numCellsX * self.numCellsY

        self.minesFixedPos = configFile.get("Mines","MinesPositions")
        self.generateMines()


    def generateMines(self):
        if self.randomMines:
            self.mines = []
            for i in range(self.numMines):
                self.mines.append([
                                    random.randrange(self.mapWidth)  - self.mapWidth/2.,
                                    random.randrange(self.mapHeight) - self.mapHeight/2.])
        else:
            minesPos = self.minesFixedPos
            if minesPos !=  "":
                self.mines = [[float(v) for v in r.split(",")] for r in minesPos.split("|")]
            else:
                self.mines = []

        mWidth, mHeight = self.mapWidth/self.cellWidth,self.mapHeight/self.cellHeight
        mines = [ [mWidth/2. + m[0]/self.cellWidth, mHeight/2. - m[1]/self.cellHeight] for m in self.mines]

        metals1 = []
        for i in arange(0.25*self.numMines):
            metals1.append([
                                random.randrange(self.mapWidth)  - self.mapWidth/2.,
                                random.randrange(self.mapHeight) - self.mapHeight/2.])

        metals2 = []
        for i in arange(0.25*self.numMines):
            metals2.append([
                                random.randrange(self.mapWidth)  - self.mapWidth/2.,
                                random.randrange(self.mapHeight) - self.mapHeight/2.])

        metals1 = [ [mWidth/2. + m[0]/self.cellWidth, mHeight/2. - m[1]/self.cellHeight] for m in metals1]
        metals2 = [ [mWidth/2. + m[0]/self.cellWidth, mHeight/2. - m[1]/self.cellHeight] for m in metals2]

#        self.mineMap = MineMapGenerator(mines, mWidth, mHeight)
        self.mineMap, self.zeroChannel = GenerateUsingRealDataset(mines,metals1,metals2,mWidth,mHeight,True)
        self.mines = [ [(m[0]-mWidth/2.)*self.cellWidth, (mHeight/2. - m[1])*self.cellHeight] for m in mines]


