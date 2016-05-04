#include "Vision_ObstacleDetection.h"

int numberOfRuns;
vector<int> stRun;
vector<int> enRun;
vector<int> rowRun;
vector<int> runLabels;
vector<pair<int, int>> equivalences;
int offset;

ObstacleDetection::ObstacleDetection()
{
    frames_num = 0;
}
ObstacleDetection::~ObstacleDetection()
{
    ;
}

void fillRunVectors(int gridMap[120][120], int& NumberOfRuns, vector<int>& stRun, vector<int>& enRun, vector<int>& rowRun)
{ 
    for (int i = 0; i < 120; i++)
    {
        if (gridMap[i][0] == 1)
        {
            NumberOfRuns++;
            stRun.push_back(0);
            rowRun.push_back(i);
        }
        for (int j = 1; j < 120; j++)
        {
            if (gridMap[i][j - 1] == 0 && gridMap[i][j] == 1)
            {
                NumberOfRuns++;
                stRun.push_back(j);
                rowRun.push_back(i);
            }
            else if (gridMap[i][j - 1] == 1 && gridMap[i][j] == 0)
            {
                enRun.push_back(j - 1);
            }
        }
        if (gridMap[i][120 - 1])
        {
            enRun.push_back(120 - 1);
        }
    }
}

void firstPass(vector<int>& stRun, vector<int>& enRun, vector<int>& rowRun, int numberOfRuns, vector<int>& runLabels, vector<pair<int, int>>& equivalences, int offset)
{
    runLabels.assign(numberOfRuns, 0);
    int idxLabel = 1;
    int curRowIdx = 0;
    int firstRunOnCur = 0;
    int firstRunOnPre = 0;
    int lastRunOnPre = -1;
    for (int i = 0; i < numberOfRuns; i++)
    {
        if (rowRun[i] != curRowIdx)
        {
            curRowIdx = rowRun[i];
            firstRunOnPre = firstRunOnCur;
            lastRunOnPre = i - 1;
            firstRunOnCur = i;
        }
        for (int j = firstRunOnPre; j <= lastRunOnPre; j++)
        {
            if (stRun[i] <= enRun[j] + offset && enRun[i] >= stRun[j] - offset && (rowRun[i] == rowRun[j]||rowRun[i] == rowRun[j] + offset))
            {
                if (runLabels[i] == 0)
                    runLabels[i] = runLabels[j];
                else if (runLabels[i] != runLabels[j])
                    equivalences.push_back(make_pair(runLabels[i], runLabels[j]));
            }
        }
        if (runLabels[i] == 0)
        {
            runLabels[i] = idxLabel++;
        }
    }
}

void replaceSameLabel(vector<int>& runLabels, vector<pair<int, int>>& equivalence)
{
    int maxLabel = *max_element(runLabels.begin(), runLabels.end());
    vector<vector<bool>> eqTab(maxLabel, vector<bool>(maxLabel, false));
    vector<pair<int, int>>::iterator vecPairIt = equivalence.begin();

    while (vecPairIt != equivalence.end())
    {
        eqTab[vecPairIt->first - 1][vecPairIt->second - 1] = true;
        eqTab[vecPairIt->second - 1][vecPairIt->first - 1] = true;
        vecPairIt++;
    }
    vector<int> labelFlag(maxLabel, 0);
    vector<vector<int>> equaList;
    vector<int> tempList;

    for (int i = 1; i <= maxLabel; i++)
    {
        if (labelFlag[i - 1])
        {
            continue;
        }
        labelFlag[i - 1] = equaList.size() + 1;
        tempList.push_back(i);
        for (vector<int>::size_type j = 0; j < tempList.size(); j++)
        {
            for (vector<bool>::size_type k = 0; k != eqTab[tempList[j] - 1].size(); k++)
            {
                if (eqTab[tempList[j] - 1][k] && !labelFlag[k])
                {
                    tempList.push_back(k + 1);
                    labelFlag[k] = equaList.size() + 1;
                }
            }
        }
        equaList.push_back(tempList);
        tempList.clear();
    }

    for (vector<int>::size_type i = 0; i != runLabels.size(); i++)
    {
        runLabels[i] = labelFlag[runLabels[i] - 1];
    }
}

void FindObstacle(vector<int>& stRun, vector<int>& enRun, vector<int>& rowRun, int numberOfRuns, vector<int>& runLabels, int & obsNum, vector<ObstaclePosition> & obsPos, vector <Position> &nextPosition)
{
    int cobsNum = *max_element(runLabels.begin(), runLabels.end());

    vector<int> start;
    vector<int> end;
    vector<int> row;

    for (int i = 0; i < cobsNum; i++)
    {
        start.clear();
        end.clear();
        row.clear();
        for (int j = 0; j < numberOfRuns; j++)
        {
            if (runLabels[j] == i + 1)
            {
                start.push_back(stRun[j] + 1);
                end.push_back(enRun[j] + 1);
                row.push_back(rowRun[j] + 1);
            }
        }

        float down = (*min_element(row.begin(), row.end()) - 1) * 0.025;
        float up = (*max_element(row.begin(), row.end())) * 0.025;

        float left = (*min_element(start.begin(), start.end()) - 1) * 0.025;
        float right = (*max_element(end.begin(), end.end())) * 0.025;

        int obsArea = 0;
        for(int i = 0; i < row.size(); i++)
        {
            obsArea = obsArea + end.at(i) - start.at(i) + 1;
        }

        ObstaclePosition tempObsPos;
        tempObsPos.X = (left + right)/2 - 1.5;
        tempObsPos.Y = (down + up)/2;
        tempObsPos.radius = sqrt(pow((right - left)/2, 2) + pow((up - down)/2, 2));
        Position tempNextPosition = {0, 0};

        //if (row.size()> 5&&fabs(left - right) > 0.125 && obsArea > 25&&tempNextPosition.Y > 0.7&&tempNextPosition.Y < 2)
        if (row.size()> 5&&fabs(left - right) > 0.125 && obsArea > 25)
        {
            if(tempObsPos.X >= 0)
            {
                tempNextPosition.X = tempObsPos.X - tempObsPos.radius - 2;
                tempNextPosition.Y= tempObsPos.Y;
            }
            else
            {
                tempNextPosition.X = tempObsPos.X + tempObsPos.radius + 2;
                tempNextPosition.Y = tempObsPos.Y;
            }

            obsPos.push_back(tempObsPos);
            nextPosition.push_back(tempNextPosition);

            obsNum = obsNum + 1;
        }
    }
}

void ObstacleDetection::ObstacleDetecting(const int obstacleMap[120][120])
{

    int cObstacleMap[120][120];
    memcpy(cObstacleMap, obstacleMap, 120*120*sizeof(int));

    stRun.clear();
    enRun.clear();
    rowRun.clear();
    runLabels.clear();
    equivalences.clear();
    numberOfRuns = 0;
    offset = 1;
    obsNum = 0;
    obsPoses.clear();
    nextPosition.clear();

    std::stringstream out;
    out<<frames_num;
    std::string filename = "ObstacleMap" + out.str() + ".txt";
    std::ofstream Obstaclemapfile(filename);
    if (Obstaclemapfile.is_open())
    {
        for(int i = 0; i < 120; i++)
        {
            for(int j = 0; j < 120; j++)
            {
                Obstaclemapfile<<cObstacleMap[i][j]<<" ";
            }
            Obstaclemapfile<<std::endl;
        }
    }
    frames_num++;

    fillRunVectors(cObstacleMap, numberOfRuns, stRun, enRun, rowRun);

    if(numberOfRuns > 0)
    {
        firstPass(stRun, enRun, rowRun, numberOfRuns, runLabels, equivalences, offset);
        replaceSameLabel(runLabels, equivalences);
        FindObstacle(stRun, enRun, rowRun, numberOfRuns, runLabels, obsNum, obsPoses,nextPosition);
    }
}
