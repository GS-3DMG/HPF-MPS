#include <stdbool.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <functional>
#include <cstring>
using namespace std;

typedef struct C3DPointInformation
{
	int x;
	int y;
	int z;
	double values;
	double distances;

	C3DPointInformation(){}
	C3DPointInformation(int xx, int yy, int zz, double valuess,double dis)
	{
		x = xx;
		y = yy;
		z = zz;
		values = valuess;
		distances = dis;
	}
}C3DPoint;

enum PropertyFileType			//ÊôÐÔÎÄŒþžñÊœ
{
	SGEMS,
	ATT
};

enum DistanceType
{
	Different,					//ÇóÒì
	Variance,					//·œ²î
	WeightedVariance			//ŒÓÈš·œ²î
};

enum SimulatingPathType
{
	Random,						//Random
	Relative_to_Data_Density,	//ºÍÃÜ¶ÈÏà¹Ø
	XYZ,						//unilateral +X, +Y, +Z,
	YXZ,						//unilateral +Y, +X, +Z,
	ZXY							//unilateral +Z, +X, +Y, 
};

struct SearchAreaCuboid
{
	int HalfX;				//³€·œÌåµÄXÒ»°ë
	int HalfY;				//³€·œÌåµÄYÒ»°ë
	int HalfZ;				//³€·œÌåµÄZÒ»°ë
};

struct SearchAreaSphere
{
	double SearchRadius;				//ËÑË÷°ëŸ¶£¬ÓÃÓÚÕÒµœÊýŸÝÑù°åÖÐÓÐÐ§µÄnžöµã×÷ÎªÊýŸÝÊÂŒþ
	int MaxPoints;						//×îŽóµÄÓÐÐ§µãžöÊý
	bool IsSampleFirst;					//ÌõŒþÊýŸÝÊÇ·ñÓÅÏÈ
	bool IsUseSamePathSize;				//ÊÇ·ñÊ¹ÓÃÍ¬Ò»ÉšÃè³ßŽç
	enum PartitionType
	{
		No,				//²»·Ö
		Cross,			//Ê®×ÖËÄ·Ö/ÈýÎ¬°Ë·Ö
		Fork			//X×ÖËÄ·Ö/ÈýÎ¬°Ë·Ö
	} MyPartitionType;					//ËÑË÷·ÖÇøÀàÐÍ
};

struct SearchArea
{
	enum SearchAreaType
	{
		Cuboid,		//Õý·œÌå
		Sphere		//ÇòÌå
	} MySearchAreaType;			//ËÑË÷ÇøÓòÀàÐÍ

	union SearchAreaData
	{
		SearchAreaCuboid Cuboid;		//Õý·œÌå
		SearchAreaSphere Sphere;		//ÇòÌå
	} MySearchAreaData;		//ËÑË÷ÇøÓò²ÎÊý
};

struct PropertyInformation
{
	string PropertyName;			//ÊôÐÔÃû³Æ
	double MinValue;				//ÊôÐÔ×îÐ¡Öµ
	double MaxValue;				//ÊôÐÔ×îŽóÖµ
};

class CSimulation
{
public:
public:
	vector<vector<vector<double>>> m_Ti;		//trianing image			Ë³ÐòZ X Y
	vector<vector<vector<double>>> m_Sim;		//simulation grid		Ë³ÐòZ X Y
	vector<vector<vector<bool>>> m_SimMark;		//sim mark		Z X Y

	vector<C3DPoint> m_Samples;					//sample data

	PropertyInformation m_Property;			//information of property

	bool IsHaveTi;						//if have training image		
	bool IsHaveSamples;					//if have samples

	bool IsSimulation;						//if simulation	

	vector<vector<int>> p_PathSim;		//path[i][0] index, path[i][1] density	
	vector<int> sim_path; //parallel simpath
	

	int m_SimX, m_SimY, m_SimZ;				//²ÉÑùœá¹ûµÄ³ßŽç
	int m_TiX, m_TiY, m_TiZ;				//ÑµÁ·ÍŒÏñµÄ³ßŽç

	int m_SamplesMinX, m_SamplesMaxX;		//³õÊŒÑùÆ··¶Î§
	int m_SamplesMinY, m_SamplesMaxY;
	int m_SamplesMinZ, m_SamplesMaxZ;

	double m_F;					//Ã¿ŽÎÔÚÑµÁ·ÍŒÏñÉÏÉšÃèµÄ°Ù·Ö±È,(0,1]
	double m_T;					//¶Ô±ÈŸàÀëµÄãÐÖµ

	SearchArea m_SearchArea;				//ËÑË÷ÇøÓòÐÅÏ¢

	DistanceType m_DistanceType;	//ŸàÀëŒÆËã·œÊœ

	SimulatingPathType m_Simul_Path_Type;	//Ä£ÄâÂ·Ÿ¶·œÊœ    

	int *p_ScanCouts;			//Ã¿ŽÎœÓÊÕµÄÊ±ºòÉšÃèÑµÁ·ÍŒÏñµÄŽÎÊý
	int *p_Bestmin;				//Ã¿ŽÎœÓÊÕµÄÊ±ºòµÄ×îÐ¡ŸàÀë
	double m_Time;				//²ÉÑùºÄÊ±


	CSimulation(int simx, int simy, int simz, double f, double thr, DistanceType distanceType, SearchArea searchArea, SimulatingPathType simulPathType, bool isHaveSamples = false );

	//f (0,1] 
	void SetParameter(int simx, int simy, int simz, double f, double thr, DistanceType distanceType, SearchArea searchArea,
				SimulatingPathType simulPathType, bool isHaveSamples = false );
	bool LoadTi(string FilePath);
	bool LoadSamples(string FilePath);
	bool SaveSimulation(string FilePath, PropertyFileType ext);
	string StartSimulation();
	void ResetSim();
	~CSimulation();

private:
	int _xSim, _ySim, _zSim;		//current simulate node index
	int _x0, _x1;			//x region
	int _y0, _y1;			//y region
	int _z0, _z1;			//z region

	string InsertSamples();
	int Compare3DPoint(C3DPoint a, C3DPoint b);
	int ComparePath(vector<int> a, vector<int> b);
	int CaculateSimDensity(int path,int xRadius,int yRadius,int zRadius);
	void ChangeSimDensity(int x,int y,int z, int xRadius, int yRadius, int zRadius);
	int XYZToPath(int x, int y, int z,int xMax,int yMax);
	void PathToXYZ(int path,int &x, int &y, int &z, int xMax, int yMax);
	//use this function when using sphere search area
	void InsertEffectivePoint(vector<C3DPoint> &EffectivePoint, C3DPoint point, int nodeX, int nodeY, int nodeZ);
	//get effective points
	void GetEffectivePoint(vector<C3DPoint> &EffectivePoint, int nodeX, int nodeY, int nodeZ);
	//distances of data events
	double GetDistances(vector<C3DPoint> EffectivePoint, int x, int y, int z, int nodeX, int nodeY, int nodeZ);

	int *SetPath_TI(int X, int Y, int Z);
	void SetPath_Simul(int xRadius, int yRadius, int zRadius);
	bool isConflictExists(vector<C3DPoint> &EffectivePoint, vector<int> Usim, int x, int y, int z);//if conflict exists
};
