#include "Aris_Vision.h"
#include <vector>
#include <string>
#include <XnCppWrapper.h>
#include "math.h"
#include <sstream>
#include <fstream>

using namespace xn;

namespace aris
{

namespace sensor
{

static int frameNum = 0;

struct Point3D
{
    float X;
    float Y;
    float Z;
};

// ax + by + cz = d; a^2 + b^2 + c^2 = 1;
void CalPlane(vector<Point3D>& cPointSet, GridMap &cgridmap)
{
    int pointNum = cPointSet.size();
    MatrixXf pointSet(pointNum,3);
    Matrix3f A(3,3);
    A<<0, 0, 0, 0, 0, 0, 0, 0, 0;

    for(int i = 0; i < pointNum; i++)
    {
        pointSet(i,0) = cPointSet[i].X;
        pointSet(i,1) = cPointSet[i].Y;
        pointSet(i,2) = cPointSet[i].Z;
    }

    float xBar = pointSet.col(0).sum()/pointNum;
    float yBar = pointSet.col(1).sum()/pointNum;
    float zBar = pointSet.col(2).sum()/pointNum;

    for(int i = 0; i < pointNum; i++)
    {
        A(0, 0) += (pointSet(i,0) - xBar)*(pointSet(i,0) - xBar);
        A(0, 1) += (pointSet(i,0) - xBar)*(pointSet(i,1) - yBar);
        A(0, 2) += (pointSet(i,0) - xBar)*(pointSet(i,2) - zBar);
        A(1, 0) += (pointSet(i,1) - yBar)*(pointSet(i,0) - xBar);
        A(1, 1) += (pointSet(i,1) - yBar)*(pointSet(i,1) - yBar);
        A(1, 2) += (pointSet(i,1) - yBar)*(pointSet(i,2) - zBar);
        A(2, 0) += (pointSet(i,2) - zBar)*(pointSet(i,0) - xBar);
        A(2, 1) += (pointSet(i,2) - zBar)*(pointSet(i,1) - yBar);
        A(2, 2) += (pointSet(i,2) - zBar)*(pointSet(i,2) - zBar);
    }

    EigenSolver<MatrixXf> es(A);

    VectorXcf eigvals = es.eigenvalues();
    Vector3f eigvalues;
    eigvalues<<real(eigvals(0)), real(eigvals(1)), real(eigvals(2));

    MatrixXcf eigvect = es.eigenvectors();
    Matrix3f eigvectors;
    eigvectors <<real(eigvect(0,0)), real(eigvect(0,1)), real(eigvect(0,2)), real(eigvect(1,0)), real(eigvect(1,1)), real(eigvect(1,2)),
            real(eigvect(2,0)), real(eigvect(2,1)), real(eigvect(2,2));

    float minValue = eigvalues(0);
    int minNum = 0;

    for(int i = 1; i < 3; i++)
    {
        if(eigvalues(i) < minValue)
        {
            minValue = eigvalues(i);
            minNum = i;
        }
    }

    float planePara[4] = {0, 0, 0, 0};

    planePara[0] = eigvectors(0, minNum);
    planePara[1] = eigvectors(1, minNum);
    planePara[2] = eigvectors(2, minNum);

    planePara[3] = planePara[0]*xBar + planePara[1]*yBar + planePara[2]*zBar;

    if(planePara[0] < 0)
    {
        for(int i = 0; i < 4; i++)
        {
            cgridmap.planePara[i] = -planePara[i];
        }
    }
    else
    {
        for(int i = 0; i < 4; i++)
        {
            cgridmap.planePara[i] = planePara[i];
        }
    }

    float distance1 = 0;
    float distance2 = sqrt(cgridmap.planePara[0]*cgridmap.planePara[0] + cgridmap.planePara[1]*cgridmap.planePara[1] + cgridmap.planePara[2]*cgridmap.planePara[2]);

    for(int i = 0; i < pointNum; i++)
    {
        distance1 += fabs(cgridmap.planePara[0]*pointSet(i,0) + cgridmap.planePara[1]*pointSet(i,1) + cgridmap.planePara[2]*pointSet(i,2) - cgridmap.planePara[3]);
    }

    cgridmap.planeDegree = distance1/distance2/pointNum;
    cgridmap.normalVector = acos(cgridmap.planePara[1]/distance2)/3.1415926*180;
}

void GeneratePointCloud(DepthGenerator& rDepthGen, const XnDepthPixel* pDepth, VISION_DATA &pData)
{
    DepthMetaData mDepthMD;
    rDepthGen.GetMetaData(mDepthMD);
    pData.timeStamp = mDepthMD.Timestamp();
    unsigned int uPointNum = mDepthMD.FullXRes() * mDepthMD.FullYRes();

    XnPoint3D* pDepthPointSet = new XnPoint3D[uPointNum];
    unsigned int i, j, idxshift, idx;
    for( j = 0; j < mDepthMD.FullYRes(); ++j)
    {
        idxshift = j * mDepthMD.FullXRes();

        for(i = 0; i < mDepthMD.FullXRes(); ++i)
        {
            idx = idxshift + i;
            pDepthPointSet[idx].X = i;
            pDepthPointSet[idx].Y = j;
            pDepthPointSet[idx].Z = pDepth[idx];
        }
    }

    XnPoint3D* p3DPointSet = new XnPoint3D[uPointNum];

    rDepthGen.ConvertProjectiveToRealWorld(uPointNum, pDepthPointSet, p3DPointSet);

    memcpy(pData.pointCloud, p3DPointSet, uPointNum*3*sizeof(float));

    delete[] pDepthPointSet;

    delete[] p3DPointSet;
}

void GenerateGridMap(VISION_DATA &cdata)
{
    int cGridNum[120][120] = {0};

    for(int i = 0; i < 480; i++)
    {
        for(int j = 0; j < 640; j++)
        {
            if(cdata.pointCloud[i][j][0] > -1.5 && cdata.pointCloud[i][j][0] < 1.5&&
                    cdata.pointCloud[i][j][2] > 0 && cdata.pointCloud[i][j][2] < 3)
            {
                int m = 0, n = 0;

                n = floor(cdata.pointCloud[i][j][0]/0.025) + 60;
                m = floor(cdata.pointCloud[i][j][2]/0.025);

                //Mean
                cdata.pGridMap[m][n].Y = (cdata.pGridMap[m][n].Y*cGridNum[m][n] + cdata.pointCloud[i][j][1])/(cGridNum[m][n] + 1);

                cdata.gridMap[m][n] = cdata.pGridMap[m][n].Y;

                cGridNum[m][n] = cGridNum[m][n] + 1;

                cdata.pGridMap[m][n].pointNum = cGridNum[m][n];
                cdata.pGridMap[m][n].X = (n - 60) * 0.025;
                cdata.pGridMap[m][n].Z = m * 0.025;
            }
        }
    }
}

class KINECT_BASE::KINECT_BASE_STRUCT
{
    friend class KINECT_BASE;
private:
    XnStatus mStatus;
    Context mContext;
    DepthGenerator mDepthGenerator;
    XnMapOutputMode mapDepthMode;
    void CheckOpenNIError(XnStatus eResult, string sStatus);
};

void KINECT_BASE::KINECT_BASE_STRUCT::CheckOpenNIError(XnStatus eResult, string sStatus)
{
    if(eResult != XN_STATUS_OK)
    {
        cerr << sStatus << "  Error" << xnGetStatusString(eResult) << endl;
    }
    else
    {
        cout<< sStatus << " Successful " << xnGetStatusString(eResult) << endl;
    }
}

KINECT_BASE::KINECT_BASE():mKinectStruct(new KINECT_BASE_STRUCT)
{
    mKinectStruct->mStatus = XN_STATUS_OK;
}

KINECT::KINECT()
{
    ;
}

KINECT::~KINECT()
{
    ;
}

void KINECT::updateData(VISION_DATA &data)
{
    KINECT_BASE::updateData(data);

    //    vector<Point3D> pPointSet;

    //    ofstream ofs4;
    //    stringstream out4;
    //    out4<<frameNum;
    //    string dataname4 ="../PointCloud/PlaneData_" + out4.str() + ".txt";
    //    ofs4.open(dataname4, ios::trunc);

    //    for(int m = 0; m < 120; m++)
    //    {
    //        for(int n = 0; n < 120; n++)
    //        {
    //            pPointSet.clear();

    //            for(int p = 0; p < 480; p++)
    //            {
    //                for(int q = 0; q < 640; q++)
    //                {
    //                    if(data.pointCloud[p][q][0] != 0 && data.pointCloud[p][q][1] != 0&&data.pointCloud[p][q][2] != 0
    //                            &&floor(data.pointCloud[p][q][0]/0.025) + 60 == n&&floor(data.pointCloud[p][q][2]/0.025) == m)
    //                    {
    //                        Point3D tempPoint;
    //                        tempPoint.X = data.pointCloud[p][q][0];
    //                        tempPoint.Y = data.pointCloud[p][q][1];
    //                        tempPoint.Z = data.pointCloud[p][q][2];

    //                        pPointSet.push_back(tempPoint);
    //                    }
    //                }
    //            }
    //            if(pPointSet.size() != 0)
    //            {
    //                CalPlane(pPointSet, data.pGridMap[m][n]);
    //            }
    //            ofs4<<data.pGridMap[m][n].planePara[0]<<" "<<data.pGridMap[m][n].planePara[1]<<" "<<data.pGridMap[m][n].planePara[2]<<" "<<data.pGridMap[m][n].planePara[3]<<" "
    //                                                 <<data.pGridMap[m][n].pointNum<<" "<<data.pGridMap[m][n].planeDegree<<" "<<data.pGridMap[m][n].normalVector<<endl;
    //        }
    //    }

    frameNum++;
}

//void KINECT_BASE::init()
//{
//    mKinectStruct->mStatus = mKinectStruct->mContext.Init();
//    mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "initialize context");
//    mKinectStruct->mapDepthMode.nFPS = 30;
//    mKinectStruct->mapDepthMode.nXRes = 640;
//    mKinectStruct->mapDepthMode.nYRes = 480;

//    mKinectStruct->mStatus = mKinectStruct->mDepthGenerator.Create(mKinectStruct->mContext);
//    mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "Create depth Generator");
//    mKinectStruct->mStatus = mKinectStruct->mDepthGenerator.SetMapOutputMode(mKinectStruct->mapDepthMode);
//    mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "Map Mode Set");
//    mKinectStruct->mStatus  = mKinectStruct->mContext.StartGeneratingAll();
//    mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "Start View Cloud");
//}

void KINECT_BASE::init()
{
    mKinectStruct->mStatus = mKinectStruct->mContext.Init();
    mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "initialize context");
    mKinectStruct->mapDepthMode.nFPS = 30;
    mKinectStruct->mapDepthMode.nXRes = 640;
    mKinectStruct->mapDepthMode.nYRes = 480;

    xn::NodeInfoList liChains;
    mKinectStruct->mContext.EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, NULL, liChains, NULL);

    //    string terrainKin = "A22596V01639310A";
    //    string avoidKin = "A22596705513327A";
    string terrainKin = "A22593B00192416A";
    string avoidKin = "A22596703731327A";
    string terrainKinNum;
    string avoidKinNum;

    for(int i = 1; i <= 8; i++)
    {
        ifstream serialFile;
        ifstream devNumFile;
        stringstream filename;
        filename << i ;
        string serailFileDir ="/sys/bus/usb/devices/usb2/2-1/2-1." + filename.str() + "/2-1." + filename.str() + ".1/serial";
        string devNumFileDir ="/sys/bus/usb/devices/usb2/2-1/2-1." + filename.str() + "/2-1." + filename.str() + ".1/devnum";

        serialFile.open(serailFileDir,ios::in);
        devNumFile.open(devNumFileDir,ios::in);

        if(serialFile.is_open())
        {
            string devSerial;
            serialFile >> devSerial;
            string devNum;
            if(devSerial == terrainKin)
            {
                devNumFile >> devNum;
                int i = std::stoi(devNum) + 2;
                stringstream temp;
                temp << i;
                terrainKinNum = "045e/02bf@2/" + temp.str();
                //cout<<terrainKinNum<<endl;
            }
            if(devSerial == avoidKin)
            {
                devNumFile >> devNum;
                int i = std::stoi(devNum) + 2;
                stringstream temp;
                temp << i;
                avoidKinNum = "045e/02bf@2/" + temp.str();
                //cout<<avoidKinNum<<endl;
            }

        }
    }

    for(xn::NodeInfoList::Iterator itNode = liChains.Begin(); itNode != liChains.End(); ++ itNode)
    {

        xn::NodeInfo mNodeInf = *itNode;

        string curDevSerial = mNodeInf.GetCreationInfo();

        //cout<<curDevSerial<<endl;

        if(curDevSerial == terrainKinNum)
        {
            xn::Device devNode;
            mKinectStruct->mContext.CreateProductionTree(mNodeInf, devNode);

            xn::Query mQuery;
            mQuery.AddNeededNode(mNodeInf.GetInstanceName());

            mKinectStruct->mStatus = mKinectStruct->mDepthGenerator.Create(mKinectStruct->mContext, &mQuery);

            mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "Create depth Generator");
            mKinectStruct->mStatus = mKinectStruct->mDepthGenerator.SetMapOutputMode(mKinectStruct->mapDepthMode);
            mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "Map Mode Set");
            mKinectStruct->mStatus  = mKinectStruct->mContext.StartGeneratingAll();
            mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "Start View Cloud");
        }
    }
}

void KINECT_BASE::release()
{
    mKinectStruct->mContext.StopGeneratingAll();
    mKinectStruct->mContext.Release();
    //mKinectStruct->mContext.Shutdown();
    cout<<"Device Close!"<<endl;
}

KINECT_BASE::~KINECT_BASE()
{
    mKinectStruct->mContext.StopGeneratingAll();
    mKinectStruct->mContext.Release();
    // mKinectStruct->mContext.Shutdown();
    cout<<"Device Close!"<<endl;
}

void KINECT_BASE::updateData(VISION_DATA &data)
{
    mKinectStruct->mStatus = mKinectStruct->mContext.WaitAndUpdateAll();
    //mKinectStruct->CheckOpenNIError(mKinectStruct->mStatus, "View Cloud");

    memset(&data, 0, sizeof(data));

    const XnDepthPixel* pDepthMap = mKinectStruct->mDepthGenerator.GetDepthMap();

    memcpy(data.depthMap, pDepthMap, 480*640*sizeof(unsigned short));

    GeneratePointCloud(mKinectStruct->mDepthGenerator, pDepthMap, data);

    Matrix4f kinectAdjust;
    kinectAdjust<< -1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Matrix4f kinectToRobot;
    /*  kinectToRobot<< 0.9989836, -0.001837780, -0.0450375377, 0.017682,
            -0.038951129, 0.4676363607, -0.88306231020, 0.197015+0.038,
            0.02268406498, 0.88391903289, 0.46708947376, 0.561919,
            0, 0, 0, 1;*/
    kinectToRobot<< 0.9998, -0.0006, -0.0208, 0.0209,
            -0.0183, 0.4584, -0.8885, 0.1986+0.038,
            0.0101, 0.8887, 0.4583, 0.5634,
            0, 0, 0, 1;

    Matrix4f robotToWorld;
    //    robotToWorld << 1, 0, 0, 0,
    //            0, 1, 0, 0.85,
    //            0, 0, 1, 0,
    //            0, 0, 0, 1;

    robotToWorld << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Matrix4f kinectToWorld = robotToWorld*kinectToRobot*kinectAdjust;

    //    write point cloud data

    //    ofstream ofs1;
    //    stringstream out1;
    //    out1<<frameNum;
    //    string dataname1 ="../PointCloud/originalcloud" + out1.str() + ".txt";
    //    ofs1.open(dataname1,ios::trunc);

    //    ofstream ofs2;
    //    stringstream out2;
    //    out2<<frameNum;
    //    string dataname2 ="../PointCloud/transformedcloud_" + out2.str() + ".txt";
    //    ofs2.open(dataname2,ios::trunc);

    //    ofstream ofs3;
    //    stringstream out3;
    //    out3<<frameNum;
    //    string dataname3 ="../PointCloud/grid_" + out3.str() + ".txt";
    //    ofs3.open(dataname3,ios::trunc);

    //    for (int i = 0; i < 480; i++)
    //    {
    //        for(int j = 0; j < 640; j++)
    //        {
    //            ofs1<<data.pointCloud[i][j][0]/1000<<" "<<data.pointCloud[i][j][1]/1000<<" "<<data.pointCloud[i][j][2]/1000<<" "<<endl;
    //        }
    //    }

    for (int i = 0; i < 480; i++)
    {
        for(int j = 0; j < 640; j++)
        {
            if(data.pointCloud[i][j][2] != 0)
            {
                Point3D tempPoint = {0, 0, 0};

                tempPoint.X = kinectToWorld(0, 0)*data.pointCloud[i][j][0] + kinectToWorld(0, 1)*data.pointCloud[i][j][1]
                        + kinectToWorld(0, 2)*data.pointCloud[i][j][2] + kinectToWorld(0, 3)*1000;

                tempPoint.Y = kinectToWorld(1, 0)*data.pointCloud[i][j][0] + kinectToWorld(1, 1)*data.pointCloud[i][j][1]
                        + kinectToWorld(1, 2)*data.pointCloud[i][j][2] + kinectToWorld(1, 3)*1000;

                tempPoint.Z = kinectToWorld(2, 0)*data.pointCloud[i][j][0] + kinectToWorld(2, 1)*data.pointCloud[i][j][1]
                        + kinectToWorld(2, 2)*data.pointCloud[i][j][2] + kinectToWorld(2, 3)*1000;

                data.pointCloud[i][j][0] = tempPoint.X/1000;
                data.pointCloud[i][j][1] = tempPoint.Y/1000;
                data.pointCloud[i][j][2] = tempPoint.Z/1000;

            }
            //            ofs2<<data.pointCloud[i][j][0]<<" "<<data.pointCloud[i][j][1]<<" "<<data.pointCloud[i][j][2]<<" "<<endl;
        }
    }

    GenerateGridMap(data);
    //    for(int i = 0; i < 60; i++)
    //    {
    //        for(int j = 0; j < 60; j++)
    //        {
    //            ofs3<<data.pGridMap[i][j].X<<" "<<data.pGridMap[i][j].Y<<" "<<data.pGridMap[i][j].Z<<endl;
    //        }
    //    }
    //    cout<<"FrameNum: "<<frameNum<<endl;
    //    ofs1.close();
    //    ofs2.close();
    //    ofs3.close();
}
}
}

