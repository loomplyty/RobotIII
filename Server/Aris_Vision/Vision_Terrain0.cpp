#include "Vision_Terrain0.h"

int TerrainAnalysis::Terrain = UnknownTerrain;
float TerrainAnalysis::leftHeight = 0;
float TerrainAnalysis::rightHeight = 0;
int frame_num = 0;

void TerrainAnalysis::TerrainAnalyze(const float oriGridMap[120][120], const float pointCloud[480][640][3])
{
    float cPointCloud[480][640][3];
    memcpy(cPointCloud, pointCloud, 3*480*640*sizeof(float));

    float GridMap[120][120];
    memcpy(GridMap, oriGridMap, 120*120*sizeof(float));

    bool isObstacle = false;
    Terrain = UnknownTerrain;
    leftHeight = 0;
    rightHeight = 0;

    for(int i=40; i<=54; i++)
    {
        for(int j = 41;j<=60;j++)
        {
            if(GridMap[i][j]>=0.225)
            {
                isObstacle = true;
                rightHeight = rightHeight + GridMap[i][j];
            }
        }
    }

    for(int i=40; i<=54; i++)
    {
        for(int j = 61;j<=80;j++)
        {
            if(GridMap[i][j]>=0.225)
            {
                isObstacle = true;
                leftHeight = leftHeight + GridMap[i][j];
            }
        }
    }

    if(isObstacle == false)
    {
        Terrain = FlatTerrain;
    }
    else
    {
        Terrain = ObstacleTerrain;
    }

    std::stringstream out;
    out<<frame_num;
    std::string filename = "GridMap" + out.str() + ".txt";
    std::ofstream Gridmapfile(filename);
    if (Gridmapfile.is_open())
    {
        for(int i = 0; i < 120; i++)
        {
            for(int j = 0; j < 120; j++)
            {
                Gridmapfile<<GridMap[i][j]<<" ";
            }
            Gridmapfile<<std::endl;
        }
    }

//    std::stringstream out1;
//    out1<<frame_num;
//    std::string filename1 = "PointCloud" + out1.str() + ".txt";
//    std::ofstream pointCloudfile(filename1);
//    if (pointCloudfile.is_open())
//    {
//        for (int i = 0; i < 480; i++)
//        {
//            for(int j = 0; j < 640; j++)
//            {
//                pointCloudfile<<cPointCloud[i][j][0]<<" "<<cPointCloud[i][j][1]<<" "<<cPointCloud[i][j][2]<<std::endl;
//            }
//        }
//    }

    frame_num++;
}
