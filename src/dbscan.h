#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

using namespace std;

typedef struct Point_
{
    float x, y, z;  // X, Y, Z position
    int clusterID;  // clustered ID
}PointCluster;

class DBSCAN {
public:    
    DBSCAN(unsigned int minPts, float eps, vector<PointCluster> points){
        m_minPoints = minPts;
        m_epsilon = eps;
        m_points = points;
        m_pointSize = points.size();
    }
    ~DBSCAN(){}

    int run();
    vector<int> calculateCluster(PointCluster point);
    int expandCluster(PointCluster point, int clusterID);
    inline double calculateDistance(PointCluster pointCore, PointCluster pointTarget);

    int getTotalPointSize() {return m_pointSize;}
    int getMinimumClusterSize() {return m_minPoints;}
    int getEpsilonSize() {return m_epsilon;}
    vector<PointCluster> m_points;
private:
    //vector<PointCluster> m_points;
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float m_epsilon;
};

#endif // DBSCAN_H
