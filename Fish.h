#include<vector>

using namespace std;

class Fish{
	
public:
	Fish();
	Fish(float x, float y, float r,int i);
	int num;
	int attNeighbor;
	int oriNeighbor;
	int nearNeighbor;
	float pos_x;
	float pos_y;
	float vp_x;
	float vp_y;
	bool inBound;
	float radius;
	float repel_distance;
	float orient_distance;
	float attract_distance;
	float Leader_attraction_distance;
	bool Leader;

	float orderX;
	float orderY;
	float vision;
	float vision1_PosX;
	float vision1_PosY;
	float vision2_PosX;
	float vision2_PosY;
	float angular_velocity;
	float angrad;
	float vel;
	float urgentVel;
	float velocity;
	float angleforRotate;
	int tick;
	float noise;

	float VectorX;
	float VectorY;
	int numberOfOrientNeighbor;
	int numberOfAttractNeighbor;
	bool hasLeader;
	long *matx;



	void setPos(float x, float y);
	float Distance(float calx, float caly);
	void setLeader();
	void setOrder(int x, int y);
	float toDegree(float radian);
	float toRadian(float degree);
	void setAngle(float a);
	float getVectorHeadX();
	float getVectorHeadY();
	float getPosX();
	float getPosY();
	float getVectorX();
	float getVectorY();
	void NextMove(vector<Fish> *group);
	void NearestNeighborMove(vector<Fish> *group);
	void TopologicalDistance(vector<Fish> *group);
	void LeaderMove();
	void drawFish();
	bool inOrientation(float x, float y, float gX, float gY);
	bool inAttraction(float x, float y, float gX, float gY);
	bool inRepulsion(float x, float y, float gX, float gY);
	bool inSight(float lower, float upper, float cal_angle);

	int getNumberofOri();
	int getNumberofAtt();

};