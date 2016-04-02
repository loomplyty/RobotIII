#ifndef VISION_TERRAIN0_H
#define VISION_TERRAIN0_H

#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;

enum TerrainType
{
    UnknownTerrain = 19,
    StepUpTerrain = 20,
    StepDownTerrain = 21,
    DitchTerrain = 22,
    FlatTerrain = 23,
    ObstacleTerrain = 24,
};

class TerrainAnalysis
{
public:
    TerrainAnalysis(){}
    ~TerrainAnalysis(){}
    static double CurrentHeight[4];
    static int leftedge_z[6];
    static int rightedge_z[6];
    static int leftedge_x[10];
    static int rightedge_x[10];
    static int Terrain;
    void TerrainAnalyze(const float oriGridMap[120][120]);
};

#endif // VISION_TERRAIN0_H
