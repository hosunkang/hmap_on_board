#include "quadtree.h"

// ----- QHeightmap -----

void QHeightmap::WriteMapToPCD(std::string outputPath)
{
    time_t t;
    struct tm* timeinfo;
    time(&t);
    timeinfo = localtime(&t);

    std::string hour, min;

    if (timeinfo->tm_hour < 10) hour = "0" + std::to_string(timeinfo->tm_hour);
    else hour = std::to_string(timeinfo->tm_hour);

    if (timeinfo->tm_min < 10) min = "0" + std::to_string(timeinfo->tm_min);
    else min = std::to_string(timeinfo->tm_min);

    std::string filePath = outputPath + hour + min + ".pcd";

    std::ofstream fout;
    fout.open(filePath);

    fout << "VERSION" << std::endl;
    fout << "FIELDS x y z" << std::endl;
    fout << "SIZE 4 4 4" << std::endl;
    fout << "TYPE F F F" << std::endl;
    fout << "COUNT 1 1 1" << std::endl;
    fout << "WIDTH 1" << std::endl;
    fout << "HEIGHT " << mMapPair.size() << std::endl;
    fout << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    fout << "POINTS " << mMapPair.size() << std::endl;
    fout << "DATA ascii" << std::endl;

    std::cout << mMapPair.size() << std::endl;
    for (auto iter = mMapPair.begin(); iter != mMapPair.end(); ++iter)
    {
        fout << iter->first.first << " " << iter->second.first / iter->second.second << " " << iter->first.second << std::endl;	// x y | z
    }

    fout.close();
}

void QHeightmap::MakeHeightMap(QPoint3D* points)	// { <x, z>, <y, dividNum> }
{
    if (mMapPair.find(std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ())) == mMapPair.end())	// exist
    {
        int dividNum = 1;
        mMapPair.insert({ std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ()), std::make_pair(points->GetY(), dividNum) });
    }
    else	// not exist
    {
        mMapPair.find(std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ()))->second.first += points->GetY();
        mMapPair.find(std::make_pair(points->GetEndNodeXZ().GetX(), points->GetEndNodeXZ().GetZ()))->second.second++;
    }
}

void QHeightmap::MakeMapToVector()
{
    for (auto iter = mMapPair.begin(); iter != mMapPair.end(); ++iter)
    {
        QPoint3D* pointXYZ = new QPoint3D;
        pointXYZ->SetXYZ(iter->first.first, iter->second.first / iter->second.second, iter->first.second);
        mPoints.push_back(pointXYZ);
    }
}

// ----- QNode -----

void QNode::subdivide()
{
    float x = mParent->mBoundary.GetX();
    float z = mParent->mBoundary.GetZ();
    float w = mParent->mBoundary.GetW();
    float h = mParent->mBoundary.GetH();

    QBoundary nw(x - w / 2, z + h / 2, w / 2, h / 2);
    QBoundary ne(x + w / 2, z + h / 2, w / 2, h / 2);
    QBoundary sw(x - w / 2, z - h / 2, w / 2, h / 2);
    QBoundary se(x + w / 2, z - h / 2, w / 2, h / 2);

    mNW = new QNode(nw, mDepth, mCapacity);
    mNE = new QNode(ne, mDepth, mCapacity);
    mSW = new QNode(sw, mDepth, mCapacity);
    mSE = new QNode(se, mDepth, mCapacity);

    mbDivided = true;
}

void QNode::insertNode(QPoint3D* point, QHeightmap* heightmap, int depth)	// NW | NE		1 | 2 
{																			// SW | SE		3 | 4
    mCapacityPoints.push(point);

    if (mDepth == depth)
    {
        heightmap->MakeHeightMap(point);
        return;
    }

    if (mCapacity < mCapacityPoints.size() && mDepth > depth)
    {
        subdivide();
    }

    if (mbDivided)
    {
        while (!mCapacityPoints.empty())
        {
            QPoint3D* qPoint = mCapacityPoints.front();
            mCapacityPoints.pop();
            if (mNW->mBoundary.IsConstained(qPoint))			// NW == 1
            {
                mParent = mNW;
                qPoint->SetEndNodeXZ(mParent->GetBoundary().GetX(), mParent->GetBoundary().GetZ());
                mParent->insertNode(qPoint, heightmap, ++depth);
            }
            else if (mNE->mBoundary.IsConstained(qPoint))	// NE == 2
            {
                mParent = mNE;
                qPoint->SetEndNodeXZ(mParent->GetBoundary().GetX(), mParent->GetBoundary().GetZ());
                mParent->insertNode(qPoint, heightmap, ++depth);
            }
            else if (mSW->mBoundary.IsConstained(qPoint))	// SW == 3
            {
                mParent = mSW;
                qPoint->SetEndNodeXZ(mParent->GetBoundary().GetX(), mParent->GetBoundary().GetZ());
                mParent->insertNode(qPoint, heightmap, ++depth);
            }
            else if (mSE->mBoundary.IsConstained(qPoint))	// SE == 4
            {
                mParent = mSE;
                qPoint->SetEndNodeXZ(mParent->GetBoundary().GetX(), mParent->GetBoundary().GetZ());
                mParent->insertNode(qPoint, heightmap, ++depth);
            }
        }
    }
}

std::vector<QPoint3D*> QNode::ReadPCDToVector(std::string inputPath)
{
    std::ifstream fin;
    fin.open(inputPath);

    std::vector<QPoint3D*> inputPoints;
    std::string line;

    if (fin.is_open())
    {
        int num = 1;
        while (!fin.eof())
        {
            getline(fin, line);
            if (num > 10)
            {
                float x, y, z;
                QPoint3D* pointXYZ = new QPoint3D;
                std::istringstream iss(line);
                iss >> x >> y >> z;
                pointXYZ->SetXYZ(x, y, z);
                inputPoints.push_back(pointXYZ);
            }
            num++;
        }
    }
    fin.close();
    return inputPoints;
}

std::vector<QPoint3D*> QNode::SamplingPoints(std::vector<QPoint3D*> inputPoints, int samplingNum)
{
    std::vector<QPoint3D*> samplingPoints;

    std::random_device random;
    std::uniform_int_distribution<int> range(0, inputPoints.size() - 1);

    for (int i = 0; i < samplingNum; i++)
    {
        int randomIndex = range(random);
        samplingPoints.push_back(inputPoints[randomIndex]);
    }
    return samplingPoints;
}

void QNode::InsertPoints(std::vector<QPoint3D*> points)
{
    for (int i = 0; i < points.size(); i++) //
    {
        int depth = 0;
        insertNode(points[i], mHeightmap, depth);	//
    }

    mHeightmap->MakeMapToVector();
}

void QNode::WriteVectorToPCD(std::string outputPath)
{
    time_t t;
    struct tm* timeinfo;
    time(&t);
    timeinfo = localtime(&t);

    std::string hour, min;

    if (timeinfo->tm_hour < 10) hour = "0" + std::to_string(timeinfo->tm_hour);
    else hour = std::to_string(timeinfo->tm_hour);

    if (timeinfo->tm_min < 10) min = "0" + std::to_string(timeinfo->tm_min);
    else min = std::to_string(timeinfo->tm_min);

    std::string filePath = outputPath + hour + min + ".pcd";

    std::ofstream fout;
    fout.open(filePath);

    fout << "VERSION" << std::endl;
    fout << "FIELDS x y z" << std::endl;
    fout << "SIZE 4 4 4" << std::endl;
    fout << "TYPE F F F" << std::endl;
    fout << "COUNT 1 1 1" << std::endl;
    fout << "WIDTH 1" << std::endl;
    fout << "HEIGHT " << mHeightmap->GetPoints().size() << std::endl;
    fout << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    fout << "POINTS " << mHeightmap->GetPoints().size() << std::endl;
    fout << "DATA ascii" << std::endl;

    //std::cout << mHeightmap->GetPoints().size() << " ";
    for (int i = 0; i < mHeightmap->GetPoints().size(); i++)
    {
        fout << mHeightmap->GetPoints()[i]->GetX() << " " << mHeightmap->GetPoints()[i]->GetY() << " " << mHeightmap->GetPoints()[i]->GetZ() << "\n";
    }

    fout.close();
}