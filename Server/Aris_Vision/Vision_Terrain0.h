#ifndef VISION_TERRAIN0_H
#define VISION_TERRAIN0_H

#include <string.h>
#include <sstream>
#include <fstream>

enum TerrainType
{
    UnknownTerrain = 19,

    FlatTerrain = 23,
    ObstacleTerrain = 24,
};

class TerrainAnalysis
{
public:
    TerrainAnalysis(){}
    ~TerrainAnalysis(){}
    static int Terrain;
    static float leftHeight;
    static float rightHeight;
    void TerrainAnalyze(const float oriGridMap[120][120], const float pointCloud[480][640][3]);
};

#endif // VISION_TERRAIN0_H
