#define _USE_MATH_DEFINES
#include<glut.h>
#include<gl/GL.h>
#include<gl/GLU.h>
#include<vector>
#include<math.h>
#include<stdio.h>
#include<iostream>
#include "Fish.h"
#include "matrix.h"
#include <time.h>
#include <windows.h>
#include <fstream>

using namespace std; 

float width = 800.0f;
float height = 800.0f;
float radius = 3.0f;
int sec =3000000000;
int runningTime = 30;
const int size =70;
bool plotting = false;
bool NearestNeighbor = false;
bool Topology = false;
bool Metric = true;
int elapse= clock();
int counting = 100;
string name = "Swarms"; 
vector<Fish> flock;
Matrix50l matrix;

void init(){
	srand(time(NULL));
	float x=0.0f;
	float y=0.0f;
	float angle=0.0f;
	flock.clear();
	for(int i=0; i< size; i++){
		x = fmod(rand(),70.0f)+400; 
		y = fmod(rand(),70.0f)+400;
		flock.push_back(*(new Fish(x,y,radius,i)));
		angle = fmod(rand(),360.0f);
		flock.at(i).setAngle(angle);
	}
		
	glMatrixMode(GL_MODELVIEW);

	//long *temp = new long[size];
	//for(int i=0; i < size; i++){
	//	temp[i] = 0;
	//}
	//matrix = Matrix50l(temp);
	//		flock.push_back(*(new Fish(400,400,radius,0)));
	//		flock.at(0).setAngle(90);
	//flock.push_back(*(new Fish(140,240,radius,1)));
	//flock.at(1).setAngle(120);
}

float calculatePolar(){
	float Pg =0.0f;
	float vecX=0.0f;
	float vecY=0.0f;
	for(int i=0; i < flock.size(); i++){
		vecX+=flock.at(i).getVectorX();
		vecY+=flock.at(i).getVectorY();
	}
	Pg = sqrt(pow(vecX,2)+pow(vecY,2));
	Pg = abs(Pg)/flock.size();
	//cout << Pg << endl;
	return Pg;
}

float calculateAngluarMomentum(){
	//calculate the r for angular momentum
	float centerX=0.0f;
	float centerY=0.0f;
	for(int i=0; i < flock.size(); i++){
		centerX +=  flock.at(i).getPosX();
		centerY +=  flock.at(i).getPosY();
	}
	centerX/=flock.size();
	centerY/=flock.size();
	glColor3ub(250,100,50);
	glPointSize(5.0);
	glBegin(GL_POINTS);
	glVertex2i(centerX,centerY);
	glEnd();

	float *Rx = new float[flock.size()];
	float *Ry = new float[flock.size()];
	float max_Rx =0.0f;
	float max_Ry =0.0f;

	for(int i=0; i < flock.size(); i++){
		Rx[i]= flock.at(i).getPosX() - centerX;
		Ry[i]= flock.at(i).getPosY() - centerY;
		if(max_Rx < Rx[i]){	max_Rx = Rx[i];}
		if(max_Ry < Ry[i]){	max_Ry = Ry[i];}
		Rx[i] /= sqrt(pow(Rx[i],2)+pow(Ry[i],2));
		Ry[i] /= sqrt(pow(Rx[i],2)+pow(Ry[i],2));
	}


	float Mt =0.0f;
	float RVx =0.0f;
	float RVy = 0.0f;
	for(int i =0; i < flock.size(); i++){
		float vx=flock.at(i).getVectorX()/(sqrt(pow(flock.at(i).getVectorX(), 2)+pow(flock.at(i).getVectorY(), 2)));
		float vy=flock.at(i).getVectorY()/(sqrt(pow(flock.at(i).getVectorX(), 2)+pow(flock.at(i).getVectorY(), 2)));
		Mt += Rx[i]*vy -Ry[i]*vx;
	}

	Mt = abs(Mt)/flock.size();

	return Mt;
}

void publish(float x, float y){
	ofstream outfile;
	outfile.open("pm.txt", ios::app);
	if(outfile.is_open()){
		outfile << x << "," << y <<"\n";
	} else{ cout << "file not found" << endl; }

	outfile.close();

}

void neighborpub(float x, float y){
	ofstream outfile;
	outfile.open("pm.txt", ios::app);
	if(outfile.is_open()){
		outfile << x << "," << y <<"\n";
	} else{ cout << "file not found" << endl; }

	outfile.close();

}

bool connection(Matrix50l mx){
	Matrix50l result1;
	result1.operator=(mx);
	Matrix50l result2 = result2.zeroMatrix();
	for(int i=0; i < size; i++){
		//multiplication
		result1.operator*=(mx);
		//addition
		result2.operator+=(result1);
	}
	//check the connectivity
	for(int i=0; i < size; i++){
		for(int j=0; j < size; j++){
			if(result2[i][j] == 0){
				return false;
			}
		}
	}
	return true;
}
void draw(){
	glClear(GL_COLOR_BUFFER_BIT);
	//matrix.zeroMatrix();
	for(int i=0; i < flock.size(); i++){
		if(flock.at(i).Leader){
			flock.at(i).LeaderMove();
		}else{
			if(Metric){
				flock.at(i).NextMove(&flock);
			} else if (NearestNeighbor) {
				flock.at(i).NearestNeighborMove(&flock);
			} else if (Topology) {
				flock.at(i).TopologicalDistance(&flock);
			}
		}

		flock.at(i).drawFish();
	}	
	


	srand(time(NULL));
	int n = rand()%flock.size();
	if(plotting){
		//if(connection(matrix)){
		float polar = calculateAngluarMomentum();
		float agm = calculatePolar();
		publish(polar, agm);
		//neighborpub(flock.at(n).getNumberofOri(), flock.at(n).getNumberofAtt());
			//cout << "running" << endl;
		//} else {
			//cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! failed !!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
		//}
	}
	glutSwapBuffers();
}

void handleResize(int w, int h) {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0,w,h,0);
	glViewport(0.0, 0.0, w, h);
}

bool chosen(int x, int y, int mousex, int mousey){
	if(radius >= sqrt(pow((double)mousex-x, 2)+pow((double)mousey-y, 2))){
		return true;
	}
	return false;
}

void mouseHandler(GLint button, GLint action, GLint x, GLint y) {

	if (button == GLUT_LEFT_BUTTON && action == GLUT_DOWN) {
		for(int i=0; i < flock.size(); i++){
			if(chosen(flock.at(i).getPosX(),flock.at(i).getPosY(),x,y)){
				if(flock.at(i).Leader){
					flock.at(i).Leader = false;
				}else{
					flock.at(i).setLeader();
				}
				break;
			} 
		}
	} else if (button == GLUT_RIGHT_BUTTON && action == GLUT_DOWN){
		//cout << "clicked" << endl;
		for(int i=0; i < flock.size(); i++){
			if(flock.at(i).Leader){
				flock.at(i).setOrder(x, y);
			}
		}
	} 
}

void KeyPressed(unsigned char key, int x, int y) {
	if(key == 27){
		for(int i=0; i < flock.size(); i++){
			if(flock.at(i).Leader){
				flock.at(i).Leader = false;
			}
		}
	} else if (key =='c'){
		init();
		cout << "pressed" << endl;
	} else if(key == 'p'){
	
	} else if (key == 'a'){
		for(int i=0; i < flock.size(); i++){
			flock.at(i).orderX += 3;
		}
	} else if (key == 'z'){
		for(int i=0; i < flock.size(); i++){
			flock.at(i).orderX -= 3;
		}
	}


}


void KeyUp(unsigned char key, int x, int y){
	cout << "release" << endl;
}

void mouseMotionHandler(GLint x, GLint y) {

}

void KeyBoardController(){

}

void myTimer(int value) { 
	//KeyBoardController();
	glutPostRedisplay();
	if(clock()-elapse > sec*1000){
		plotting = true;
	}
	if(clock()-elapse > (sec+runningTime)*1000){
		cout << counting << endl;
		init();
		elapse = clock();
		plotting = false;

		ofstream outfile;
		outfile.open("pm.txt", ios::app);
		if(outfile.is_open()){
			outfile << name <<"\n";
		} else{ cout << "file not found" << endl; }

		outfile.close();

		if(counting  == 0){
			exit(0);
		} else {
			counting--;
		}
	}
	glutTimerFunc(24, myTimer, 1);
}

 
int main()
{  
	//glutInitDisplayMode(GLUT_RGB | GLUT_SINGLE);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize(width, height);    
	glutInitWindowPosition(200, 100);   
	glutCreateWindow("Virtual Pond");    // title of the window
	glClearColor(0.95, 1.0, 1.0, 0.0);
	glMatrixMode (GL_PROJECTION);
	gluOrtho2D(0,width,height,0);
	init();
	glutTimerFunc(24, myTimer, 1); 
	glutDisplayFunc(draw);
	glutReshapeFunc(handleResize);
	glutMouseFunc(mouseHandler);
	glutMotionFunc(mouseMotionHandler);
	glutKeyboardFunc(KeyPressed);
	glutKeyboardUpFunc(KeyUp);
	int elapse= clock();
	glutMainLoop();
	return 0;

}