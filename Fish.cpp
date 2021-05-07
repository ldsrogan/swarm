#define _USE_MATH_DEFINES
#include "Fish.h"
#include <iostream>
#include <math.h>
#include <glut.h>
#include <vector>

Fish::Fish(){

}

Fish::Fish(float x, float y, float r,int i){
	num = i;
	numberOfOrientNeighbor=0;
	numberOfAttractNeighbor=0;
	attNeighbor=10; 
	oriNeighbor=10;
	nearNeighbor =10;
	Leader = false;
	hasLeader = false; 
	inBound = false;
	radius =r;
	angular_velocity =40.0f;
	pos_x = x;
	pos_y = y;
	orderX =0;
	orderY=0;
	repel_distance = radius*1;
	orient_distance =  repel_distance+(radius*3);
	attract_distance = orient_distance+(radius*10);

	vision =270;
	vel = 0; 
	velocity =3.0f * radius*0.1;

	angrad = (float) ((angular_velocity)*(M_PI/180))*0.1;
	vp_x =(float) cos(angleforRotate*(180/M_PI));
	vp_y =(float) sin(angleforRotate*(180/M_PI));	
	tick =0;
	noise =0.0f;
	VectorX =0.0f;
	VectorY =0.0f;

}


float Fish::toRadian(float degree){
	return degree *(M_PI/180);
}

float Fish::toDegree(float radian){
	return radian *(180/M_PI);
}

void Fish::setAngle(float a){
	angleforRotate =a;
}

float Fish::getVectorHeadX(){
	return vp_x;
}

float Fish::getVectorHeadY(){
	return vp_y;
}

float Fish::getPosX(){
	return pos_x;
}

float Fish::getPosY(){
	return pos_y;
}

void Fish::setLeader(){
	Leader = true;
}

int Fish::getNumberofOri(){
	return numberOfOrientNeighbor;
}
int Fish::getNumberofAtt(){
	return numberOfAttractNeighbor;
}
void Fish::LeaderMove(){
	double vector_X_head=0.0;
	double vector_Y_head=0.0;
	double angle_head=toRadian(angleforRotate);

	angle_head =(atan2(orderY - pos_y, orderX - pos_x));

	if(angle_head < 0){	angle_head+= (M_PI*2); }	

	vector_X_head= cos(angle_head);
	vector_Y_head= sin(angle_head);

	double degree =angleforRotate;

	angle_head =(atan2(vector_Y_head, vector_X_head));

	degree = toDegree(angle_head);
	if(degree <0){ degree+=360; }
	if(angleforRotate <0){ angleforRotate+=360; }
	if(degree != angleforRotate){
		float angvel = angrad;
		if(degree < angleforRotate){
			if(angrad > angleforRotate -degree){angvel = angleforRotate - degree;}
			if((360-angleforRotate)+degree < angleforRotate-degree){ 
				angleforRotate = angleforRotate+toDegree(angvel);
			} else{
				angleforRotate = angleforRotate-toDegree(angvel);
			}
		}else if (degree > angleforRotate){
			if(angrad > degree-angleforRotate){angvel = angleforRotate - degree;}
			if((360-degree)+angleforRotate < degree-angleforRotate){
				angleforRotate =  angleforRotate-toDegree(angvel);
			} else{
				angleforRotate =   angleforRotate+toDegree(angvel);
			}
		}
	}


	angleforRotate = fmod(angleforRotate,360);

	vp_x = cos(toRadian(angleforRotate));
	vp_y = sin(toRadian(angleforRotate));

	double angle_body=(atan2(vp_y, vp_x));
	VectorX= cos(angle_body);
	VectorY= sin(angle_body);


	//vector for position

	pos_x += VectorX*vel;
	pos_y += VectorY*vel;

}

void Fish::TopologicalDistance(vector<Fish> *group){
	float calX_head =0.0f; 
	float calY_head =0.0f;
	float cal_angle =0.0f;
	float vector_X_head=0.0f;
	float vector_Y_head=0.0f;
	float repulsion_vector_X = 0.0f;
	float repulsion_vector_Y = 0.0f;
	float angle_head= toRadian(angleforRotate);
	inBound=false;
	bool noRepulsion = true;

	int size = group->size();
	Fish *nearest = new Fish [size];
	float *VX = new float[size];
	float *VY = new float[size];
	int nearcount =0;
	for(int i =0; i < group->size(); i++){
		Fish neighbor = group->at(i);	
		if(neighbor.num != num){
			float neighborPosX = neighbor.getPosX();
			float neighborPosY = neighbor.getPosY();
			calX_head = (pos_x)-neighborPosX;
			calY_head =(pos_y)-neighborPosY;
			cal_angle =(float) (atan2(neighborPosY - (pos_y), neighborPosX - (pos_x)));
			angle_head =cal_angle;
			double lower =0;
			double upper =0;
			//normalize the degree (0~360)
			if(cal_angle < 0){ cal_angle+= (M_PI*2); }	
			if(angleforRotate <0){ angleforRotate+=360; }
			if(angleforRotate-(vision/2) < 0){ lower = 360+(angleforRotate-(vision/2)); }
			else{ lower = angleforRotate-(vision/2); }
			float c = cos(angle_head);
			float s = sin(angle_head);
			bool canSee = inSight(lower,upper,cal_angle);
			upper = fmod((360+(angleforRotate+(vision/2))),360);
			if(canSee){
				if(inRepulsion(pos_x, pos_y, neighborPosX,neighborPosY)){
					repulsion_vector_X+= (float) -c;
					repulsion_vector_Y+= (float) -s;
					noRepulsion = false;
					inBound = true;
				} 
				if(noRepulsion){
					if(inAttraction(pos_x, pos_y, neighborPosX,neighborPosY)){
						nearest[nearcount] = neighbor;
						VX[nearcount] = c;
						VY[nearcount] = s;
						inBound = true;
						nearcount++;
					} else if (inOrientation(pos_x, pos_y, neighborPosX,neighborPosY)) {
						nearest[nearcount] =neighbor;
						inBound = true;
						float norm = sqrt(pow(neighbor.getVectorHeadX(),2)+pow(neighbor.getVectorHeadY(),2));
						VX[nearcount] = neighbor.getVectorHeadX()/norm;
						VY[nearcount] = neighbor.getVectorHeadY()/norm;
						nearcount++;
					} 
				}
			} 
		}
	}
	
	if(noRepulsion){
			for(int i=0; i < nearcount; i++){
				Fish n1 = nearest[i];
				for(int j=i; j< nearcount; j++){
					Fish n2 = nearest[j];
					if(Distance(n1.getPosX()-pos_x,n1.getPosY()-pos_y) > Distance(n2.getPosX()-pos_x,n2.getPosY()-pos_y)){
						nearest[i] = n2;
						nearest[j] = n1;
						float temp = VX[i];
						VX[i] = VX[j];
						VX[j] = temp;
						temp = VY[i];
						VY[i] = VX[j];
						VY[j] = temp;
					}
				}
			}	
		for(int i=0; i < nearNeighbor; i++){
			vector_X_head+=VX[i];
			vector_Y_head+=VY[i];
		}
	}

	vel = velocity;
	if(!noRepulsion && inBound){
		vector_X_head = repulsion_vector_X;
		vector_Y_head = repulsion_vector_Y;
	} 


	/*
	*   angle calculation
	*/
	float degree =angleforRotate;
	float norm = (sqrt(pow(vector_X_head,2)+pow(vector_Y_head,2)));
	vector_X_head = vector_X_head / norm;
	vector_Y_head = vector_Y_head / norm;
	if(inBound){
		angle_head =(float) (atan2(vector_Y_head, vector_X_head));
		degree =toDegree(angle_head);
		if(degree <0){ degree+=360;}
	}


	if(degree != angleforRotate){
		float angvel = angrad;
		float angveldegree = toDegree(angvel);
		if(degree < angleforRotate){
			if(angrad > angleforRotate -degree){angvel = angleforRotate - degree;}
			if((360-angleforRotate)+degree < angleforRotate-degree){ 
				angleforRotate =  angleforRotate+angveldegree;
			} else{
				angleforRotate =  angleforRotate-angveldegree;
			}
		}else if (degree > angleforRotate){
			if(angrad > degree-angleforRotate){angvel = angleforRotate - degree;}
			if((360-degree)+angleforRotate < degree-angleforRotate){
				angleforRotate = angleforRotate-angveldegree;
			} else{
				angleforRotate = angleforRotate+angveldegree;
			}
		}
	}

	noise =3.0f*(((float)rand()/RAND_MAX)-0.5);
	angleforRotate+=noise;


	angleforRotate = fmod(angleforRotate, 360);

	vp_x = (float) (cos(toRadian(angleforRotate)));
	vp_y = (float) (sin(toRadian(angleforRotate)));

	float angle_body=(float) (atan2(vp_y, vp_x));
	VectorX= (float) cos(angle_body);
	VectorY= (float) sin(angle_body);	

	//vector for position
	pos_x += VectorX*vel;
	pos_y += VectorY*vel;
}

void Fish::NearestNeighborMove(vector<Fish> *group){
	float calX_head =0.0f; 
	float calY_head =0.0f;
	float cal_angle =0.0f;
	float vector_X_head=0.0f;
	float vector_Y_head=0.0f;
	float repulsion_vector_X = 0.0f;
	float repulsion_vector_Y = 0.0f;
	float angle_head= 0.0;
	inBound=false;
	bool noRepulsion = true;

	int size = group->size();
	float *attnearest =  new float[size];
	float *orinearest =  new float[size];
	float *Att_angle =  new float[size];
	float *Ori_vec_x =  new float[size];
	float *Ori_vec_y =  new float[size];
	int attcount =0;
	int oricount =0;
	for(int i =0; i < group->size(); i++){
		Fish neighbor = group->at(i);	
		if(neighbor.num != num){
			float neighborPosX = neighbor.getPosX();
			float neighborPosY = neighbor.getPosY();
			calX_head = (pos_x)-neighborPosX;
			calY_head =(pos_y)-neighborPosY;
			cal_angle =(float) (atan2(neighborPosY - (pos_y), neighborPosX - (pos_x)));
			angle_head =cal_angle;
			double lower =0;
			double upper =0;
			//normalize the degree (0~360)
			if(cal_angle < 0){ cal_angle+= (M_PI*2); }	
			if(angleforRotate <0){ angleforRotate+=360; }
			float angleCal =angleforRotate-(vision/2);
			if(angleCal < 0){ lower = 360+(angleCal); }
			else{ lower = angleCal; }
			upper = fmod((360+(angleforRotate+(vision/2))),360);
			float c = cos(angle_head);
			float s = sin(angle_head);
			bool canSee = inSight(lower,upper,cal_angle);
			if(canSee){
				if(inRepulsion(pos_x, pos_y, neighborPosX,neighborPosY)){
					repulsion_vector_X+= (float) -cos(angle_head);
					repulsion_vector_Y+= (float) -sin(angle_head);
					noRepulsion = false;
					inBound = true;
				}  
				if(noRepulsion){
					if(inAttraction(pos_x, pos_y, neighborPosX,neighborPosY)){
						attnearest[attcount] = Distance(neighborPosX-pos_x,neighborPosY-pos_y);
						Att_angle[attcount] =angle_head;
						inBound = true;
						attcount++;
					} else if (inOrientation(pos_x, pos_y, neighbor.getPosX(),neighborPosY)) {
						orinearest[oricount] = Distance(neighborPosX-pos_x,neighborPosY-pos_y);
						float norm = sqrt(pow(neighbor.getVectorHeadX(),2)+pow(neighbor.getVectorHeadY(),2));
						Ori_vec_x[oricount] = neighbor.getVectorHeadX()/norm;
						Ori_vec_y[oricount] = neighbor.getVectorHeadY()/norm;
						inBound = true;
						oricount++;
					} 
				}
			}
		} 
	}

	if(noRepulsion){
		if(attcount > attNeighbor){	
			for(int i=0; i < attcount; i++){
				float a1 =attnearest[i];
				float an1 = Att_angle[i];
				for(int j=i; j< attcount; j++){ 
					float a2 =attnearest[j];
					float an2 = Att_angle[j];
					if( a1> a2){
						attnearest[i] = a2;
						attnearest[j] = a1;
						Att_angle[i] = an2;
						Att_angle[j] = an1;
					}
				}
			}	
		}
		int total = attNeighbor;
		if(attNeighbor > attcount){
			total = attcount;
		}
		
		for(int i=0; i < total; i++){
			float temp =Att_angle[i];
			vector_X_head+= (float) cos(temp);  
			vector_Y_head+= (float) sin(temp);
		}
		if(oricount > oriNeighbor){
			for(int i=0; i < oricount; i++){
				float t1 = orinearest[i];
				float xt1 = Ori_vec_x[i];
				float yt1 = Ori_vec_y[i];
				for(int j=i; j< oricount; j++){
					float t2 = orinearest[j];
					float xt2 = Ori_vec_x[j];
					float yt2 = Ori_vec_y[j];
					if(t1 > t2){
						orinearest[i] = t2;
						orinearest[j] = t1;
						Ori_vec_x[i] = xt2;
						Ori_vec_x[j] = xt1;
						Ori_vec_y[i] = yt2;
						Ori_vec_y[j] = yt1;
					}
				}
			}
		}

		total = oriNeighbor;
		if(oriNeighbor > oricount){
			total = oricount;
		}
		for(int i=0; i < total; i++){
			vector_X_head+=Ori_vec_x[i];
			vector_Y_head+=Ori_vec_y[i];
		}
	}

	vel = velocity;
	if(!noRepulsion && inBound){
		vector_X_head = repulsion_vector_X;
		vector_Y_head = repulsion_vector_Y;
	} 


	/*
	*   angle calculation
	*/
	float degree =angleforRotate;
	vector_X_head = vector_X_head / (sqrt(pow(vector_X_head,2)+pow(vector_Y_head,2)));
	vector_Y_head = vector_Y_head / (sqrt(pow(vector_X_head,2)+pow(vector_Y_head,2)));
	if(inBound){
		angle_head =(float) (atan2(vector_Y_head, vector_X_head));
		degree =toDegree(angle_head);
		if(degree <0){ degree+=360;}
	}


	if(degree != angleforRotate){
		float angvel = angrad;
		float angveldegree = toDegree(angvel);
		if(degree < angleforRotate){
			if(angrad > angleforRotate -degree){angvel = angleforRotate - degree;}
			if((360-angleforRotate)+degree < angleforRotate-degree){ 
				angleforRotate =  angleforRotate+angveldegree;
			} else{
				angleforRotate =  angleforRotate-angveldegree;
			}
		}else if (degree > angleforRotate){
			if(angrad > degree-angleforRotate){angvel = angleforRotate - degree;}
			if((360-degree)+angleforRotate < degree-angleforRotate){
				angleforRotate = angleforRotate-angveldegree;
			} else{
				angleforRotate = angleforRotate+angveldegree;
			}
		}
	}

	noise =3.0f*(((float)rand()/RAND_MAX)-0.5);
	angleforRotate+=noise;


	angleforRotate = fmod(angleforRotate, 360);
	float calRad =toRadian(angleforRotate);
	vp_x = (float) (cos(calRad));
	vp_y = (float) (sin(calRad));

	float angle_body=(float) (atan2(vp_y, vp_x));
	VectorX= (float) cos(angle_body);
	VectorY= (float) sin(angle_body);	

	//vector for position
	pos_x += VectorX*vel;
	pos_y += VectorY*vel;

	delete [] attnearest;
	delete [] orinearest;
	delete [] Att_angle;
	delete [] Ori_vec_x;
	delete [] Ori_vec_y;
}

void Fish::NextMove(vector<Fish> *group){
	float calX_head =0.0f; 
	float calY_head =0.0f;
	float cal_angle =0.0f;
	float vector_X_head=0.0f;
	float vector_Y_head=0.0f;
	float leader_vector_X =0.0f;
	float leader_vector_Y =0.0f;
	float repulsion_vector_X = 0.0f;
	float repulsion_vector_Y = 0.0f;
	float angle_head= toRadian(angleforRotate);

	inBound=false;
	hasLeader = false;
	bool noRepulsion = true;
	//matx = new long[group->size()];
	for(int i =0; i < group->size(); i++){
		Fish neighbor = group->at(i);	
		if(neighbor.num != num){
			float neighborPosX = neighbor.getPosX();
			float neighborPosY = neighbor.getPosY();
			calX_head = neighborPosX-(pos_x);
			calY_head = neighborPosY-(pos_y);
			cal_angle =(float) (atan2(calY_head, calX_head));
			angle_head =cal_angle;
			double lower =0;
			double upper =0;
			//normalize the degree (0~360)
			if(cal_angle < 0){ cal_angle+= (M_PI*2); }	
			if(angleforRotate <0){ angleforRotate+=360; }
			float angleCal =angleforRotate-(vision/2);
			if(angleCal < 0){ lower = 360+(angleCal); }
			else{ lower = angleCal; }
			upper = fmod((360+(angleforRotate+(vision/2))),360);
			float c = cos(angle_head); 
			float s = sin(angle_head);
			bool canSee = inSight(lower,upper,cal_angle);
			if(canSee){
				if(neighbor.Leader && inAttraction(pos_x, pos_y, neighborPosX,neighborPosY)){
					leader_vector_X+= c;
					leader_vector_Y+= s;
					hasLeader =true;
					inBound = true;
					//cout << "following leader" << endl; 
				} else if(inRepulsion(pos_x, pos_y, neighborPosX,neighborPosY)){
					//repulsion_vector_X+= -c;
					//repulsion_vector_Y+= -s;
					repulsion_vector_X -= (calX_head/ sqrt(pow(calX_head,2)+pow(calY_head,2)));
					repulsion_vector_Y -= (calY_head/ sqrt(pow(calX_head,2)+pow(calY_head,2)));
					inBound = true;
					noRepulsion = false;
					//cout << "only repulsion" << endl;
				} else if(!neighbor.Leader && inAttraction(pos_x, pos_y, neighborPosX,neighborPosY)){
					inBound = true;
					//vector_X_head+= c;
					//vector_Y_head+= s;
					vector_X_head+=  (calX_head/ sqrt(pow(calX_head,2)+pow(calY_head,2)));  
					vector_Y_head+=  (calY_head/ sqrt(pow(calX_head,2)+pow(calY_head,2)));
				} else if (inOrientation(pos_x, pos_y, neighborPosX,neighborPosY)) {
					vector_X_head+= (neighbor.getVectorHeadX());
					vector_Y_head+= (neighbor.getVectorHeadY());
					inBound = true;
				} 
				//if(Distance(neighbor.getPosX()-pos_x,neighbor.getPosY()-pos_y) <= attract_distance){
				//	matx[i] =1;
				//} else{
				//	matx[i] =0;
				//}
			}
		} /*else {
		  matx[i] =0;
		  }*/
	}
	//ignore all the vector if there is a leader.
	vel = velocity;
	if(noRepulsion && hasLeader && inBound){
		vector_X_head = leader_vector_X;
		vector_Y_head = leader_vector_Y;
	} else if(!noRepulsion && inBound){
		vector_X_head = repulsion_vector_X;
		vector_Y_head = repulsion_vector_Y;
	} 



	/*
	*   angle calculation
	*/
	float degree =angleforRotate;

	if(inBound){
		angle_head =(float) (atan2(vector_Y_head, vector_X_head));
		degree =toDegree(angle_head);
		if(degree <0){ degree+=360;}
	}


	if(degree != angleforRotate){
		float angvel = angrad;
		float angveldegree = toDegree(angvel);
		if(degree < angleforRotate){
			if(angrad > angleforRotate -degree){angvel = angleforRotate - degree;}
			if((360-angleforRotate)+degree < angleforRotate-degree){ 
				angleforRotate =  angleforRotate+angveldegree;
			} else{
				angleforRotate =  angleforRotate-angveldegree;
			}
		}else if (degree > angleforRotate){
			if(angrad > degree-angleforRotate){angvel = angleforRotate - degree;}
			if((360-degree)+angleforRotate < degree-angleforRotate){
				angleforRotate = angleforRotate-angveldegree;
			} else{
				angleforRotate = angleforRotate+angveldegree;
			}
		}
	}

	noise =3.0f*(((float)rand()/RAND_MAX)-0.5);
	angleforRotate+=noise;


	angleforRotate = fmod(angleforRotate, 360);
	
	vp_x = (float) (cos(toRadian(angleforRotate)));
	vp_y = (float) (sin(toRadian(angleforRotate)));

	float angle_body=(float) (atan2(vp_y, vp_x));
	VectorX= (float) cos(angle_body);
	VectorY= (float) sin(angle_body);

	//vector for position
	pos_x += VectorX*vel;
	pos_y += VectorY*vel;

}

float Fish::getVectorX(){
	return VectorX;
}

float Fish::getVectorY(){
	return VectorY;
}

float Fish::Distance(float calx, float caly){
	return (float) sqrt(pow(calx, 2)+pow(caly, 2));
}

bool Fish::inRepulsion(float x, float y, float gX, float gY){
	bool itis = false;
	float distance=0.0f;
	distance = Distance(gX-x,y-gY);
	if(distance <= repel_distance){itis = true;}
	return itis;
}

bool Fish::inAttraction(float x, float y, float gX, float gY){
	float distance= Distance(gX-x,y-gY);
	if(distance > orient_distance && distance <= attract_distance){return true;}
	return false;
}


bool Fish::inOrientation(float x, float y, float gX, float gY){
	float distance= Distance(gX-x,y-gY);
	if(distance > repel_distance && distance <= orient_distance){return true;}
	return false;
}


bool Fish::inSight(float lower, float upper, float cal_angle){
	if(vision == 360){
		return true;
	}else{
		float deg =toDegree(cal_angle);
		if(lower > upper){
			if((lower<= deg && deg <= 359.99) 
				|| (0 <= deg && deg <= upper)){return true;}
		}else{
			if(lower<= deg && deg <= upper){return true;}
		}
	}
	return false;
}

void Fish::setOrder(int x, int y){
	orderX = (float)x;
	orderY = (float)y;
}

void Fish::drawFish(){

	if(Leader){
		glColor3ub(250,100,50);
	} else {
		glColor3ub(0,0,0);
	}
	//body
	glBegin(GL_LINE_LOOP);
	for(int angle=0; angle < 360; angle++){
		glVertex2f(pos_x + cos((float)angle)*(radius-1),  (pos_y + sin((float)angle)*(radius-1)));
	}
	glEnd();
	/*glPointSize(5.0);
	glBegin(GL_POINTS);
	glVertex2f(pos_x,pos_y);
	glEnd();
	glEnable(GL_LINE_SMOOTH);
	glLineWidth(1.0);
	glDisable(GL_LINE_SMOOTH);*/
	//zone of attraction
	if(Leader){
		glColor3ub(50,150,0);
		glBegin(GL_LINE_LOOP);
		for(int angle=0; angle < 360; angle++){
			glVertex2f(pos_x + cos((float)angle)*attract_distance,  (pos_y + sin((float)angle)*attract_distance));
		}
		glEnd();
	}
	//head
	glBegin(GL_LINES);
	glVertex2f(pos_x,pos_y);
	glVertex2f(pos_x+ (vp_x*10),pos_y+(vp_y*10));
	glEnd();
	glPointSize(5.0);
	glBegin(GL_POINTS);
	glVertex2f(orderX,orderY);
	glEnd();
}
