#include <algorithm>
#include <ctime>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include <queue>

class QPoint2D
{
public:
    QPoint2D() : mX(0), mZ(0) {}
    QPoint2D(float x, float z) : mX(x), mZ(z) {}

    float GetX() const { return mX; }
    float GetZ() const { return mZ; }

    void SetX(float x) { this->mX = x; }
    void SetZ(float z) { this->mZ = z; }

private:
    float mX, mZ;
};

class QPoint3D
{
public:
    QPoint3D() : mX(0), mY(0), mZ(0) {}
    QPoint3D(float x, float y, float z) : mX(x), mY(y), mZ(z) {}

    float GetX() const { return mX; }
    float GetY() const { return mY; }
    float GetZ() const { return mZ; }
    std::vector<int> GetPath() const { return mPath; }
    QPoint2D GetEndNodeXZ() const { return mEndNodeXZ; }
    std::string GetStringPath() const { return mStringPath; }

    void SetX(float x) { this->mX = x; }
    void SetY(float y) { this->mY = y; }
    void SetZ(float z) { this->mZ = z; }
    void SetXYZ(float x, float y, float z) { this->mX = x; this->mY = y; this->mZ = z; }
    void SetPath(int path) { this->mPath.push_back(path); }
    void SetEndNodeXZ(QPoint2D endNodeXZ) { this->mEndNodeXZ = endNodeXZ; }
    void SetEndNodeXZ(float x, float z) { this->mEndNodeXZ.SetX(x); this->mEndNodeXZ.SetZ(z); }

private:
    float mX, mY, mZ;
    std::vector<int> mPath;
    std::string mStringPath;
    QPoint2D mEndNodeXZ;
};

class QBoundary
{
public:
    QBoundary() : mX(0), mZ(0), mW(0), mH(0) {}
    QBoundary(float x, float z, float w, float h) : mX(x), mZ(z), mW(w), mH(h) {}
    QBoundary(float minX, float maxX, float z) : mX((maxX + minX) / 2), mZ(z / 2), mW((maxX - minX) / 2), mH(z / 2) {}

    float GetX() const { return mX; }
    float GetZ() const { return mZ; }
    float GetW() const { return mW; }
    float GetH() const { return mH; }

    void SetBoundary(float x, float z, float w, float h) { this->mX = x; this->mZ = z; this->mW = w; this->mH = h; }

    bool IsConstained(QPoint3D* p) { return (p->GetX() >= mX - mW && p->GetX() < mX + mW && p->GetZ() >= mZ - mH && p->GetZ() < mZ + mH); }

private:
    float mX, mZ, mW, mH;
};

class QHeightmap
{
public:
    QHeightmap() {}

    std::vector<QPoint3D*> GetPoints() const { return mPoints; }

    void WriteMapToPCD(std::string outputPath);
    void MakeHeightMap(QPoint3D* points);
    void MakeMapToVector();

private:
    std::map<std::pair<float, float>, std::pair<float, int>> mMapPair;
    std::vector<QPoint3D*> mPoints;
};

class QNode
{
public:
    QNode() : mBoundary() {}
    QNode(QBoundary boundary, int depth, int capacity) : mBoundary(boundary), mDepth(depth), mCapacity(capacity) {}
    QNode(QBoundary boundary, int depth) : mBoundary(boundary), mDepth(depth) {}

    QHeightmap* GetHeightmap() const { return mHeightmap; }
    QBoundary GetBoundary() const { return mBoundary; }

    void SetHeightmap(QHeightmap* heightmap) { this->mHeightmap = heightmap; }
    void SetBoundary(QBoundary boundary) { this->mBoundary = boundary; }
    void SetBoundary(float x, float z, float w, float h) { this->mBoundary.SetBoundary(x, z, w, h); }
    void SetDepth(int depth) { this->mDepth = depth; }
    void SetCapacity(int capacity) { this->mCapacity = capacity; }
    void SetPoints(std::vector<QPoint3D*> points) { this->mPoints = points; }

    void InsertPoints(std::vector<QPoint3D*> points);
    std::vector<QPoint3D*> SamplingPoints(std::vector<QPoint3D*> inputPoints, int samplingNum);
    std::vector<QPoint3D*> ReadPCDToVector(std::string inputPath);
    void WriteVectorToPCD(std::string outputPath);

private:
    void subdivide();
    void insertNode(QPoint3D* p, QHeightmap* heightmap, int depth);

    QHeightmap* mHeightmap = new QHeightmap();
    QBoundary mBoundary;
    int mCapacity = 1;
    int mDepth = 0;
    bool mbDivided = false;

    std::vector<QPoint3D*> mPoints;
    std::queue<QPoint3D*> mCapacityPoints;

    QNode* mParent = this;
    QNode* mNW = nullptr;
    QNode* mNE = nullptr;
    QNode* mSW = nullptr;
    QNode* mSE = nullptr;
};
