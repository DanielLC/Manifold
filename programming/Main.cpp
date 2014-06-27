#include <stdio.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <vector>
#include <iostream>
#include <unistd.h>
#include "Compound.h"
//#include "Euclidean.h"
//#include "Hyperbolic.h"
//#include "R2xS1.h"
#include "PortalSpace2d2.h"
#include "Manifold.h"
#include "SurfaceOfRevolution.h"
//#include "Cylinder2d.h"
//#include "BlackHole2d.h"
#include <Eigen/Core>
#include <cstdlib>
using Eigen::Matrix3d;
using Eigen::Vector3d;

//It might be a good idea to have a separate matrix for rotation and for the distortion of space, since rotation can be inverted by transposing, and if the distortion is just a function of the position the inverse could presumably be calculated easily, but their product would have to be calculated the hard way.

#define KEY_ESCAPE 27

std::tr1::shared_ptr<Manifold> space;
Manifold::PointOfReferencePtr por;
std::vector<Manifold::Triangle> triangleList;
const double SPEED = 0.05;

typedef struct {
    int width;
	int height;
	char* title;

	float field_of_view_angle;
	float z_near;
	float z_far;
} glutWindow;

glutWindow win;


void display() 
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);		     // Clear Screen and Depth Buffer
	glLoadIdentity();
	glTranslatef(0.0f,0.0f,-3.0f);
	glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
	//std::cout << "test\n" << std::flush;
	/*
	 * Triangle code starts here
	 * 3 verteces, 3 colors.
	 */
	/*glBegin(GL_TRIANGLES);					
		glColor3f(0.0f,0.0f,1.0f);			
		glVertex3f( 0.0f, 1.0f, 0.0f);		
		glColor3f(0.0f,1.0f,0.0f);			
		glVertex3f(-1.0f,-1.0f, 0.0f);		
		glColor3f(1.0f,0.0f,0.0f);			
		glVertex3f( 1.0f,-1.0f, 0.0f);		
	glEnd();*/
	/*glBegin(GL_TRIANGLES);					
		glColor3f(0.0f,0.0f,1.0f);			
		glVertex2f( 0.0f, 1.0f);		
		glColor3f(0.0f,1.0f,0.0f);			
		glVertex2f(-1.0f,-1.0f);		
		glColor3f(1.0f,0.0f,0.0f);			
		glVertex2f( 1.0f,-1.0f);		
	glEnd();*/
	for(int i=0; i<triangleList.size(); ++i) {
	Vector3d v0 = space->vectorFromPoint(por, triangleList[i][0]);
	Vector3d v1 = space->vectorFromPoint(por, triangleList[i][1]);
	Vector3d v2 = space->vectorFromPoint(por, triangleList[i][2]);
	//std::cout << v0list[0] << "\n" << std::flush;
		if(v0[1] < 0 || v1[1] < 0 || v2[1] < 0)
			continue;
		v0 /= v0[1];
		v1 /= v1[1];
		v2 /= v2[1];
		//std::cout << v0list[j0][0] << "," << v0list[j0][1] << "," << v0list[j0][2] << "\n";
		//std::cout << v1list[j1][0] << "," << v1list[j1][1] << "," << v1list[j1][2] << "\n";
		//std::cout << v2list[j2][0] << "," << v2list[j2][1] << "," << v2list[j2][2] << "\n";
		glBegin(GL_TRIANGLES);			
			glVertex2f(v0[0],v0[2]);
			glVertex2f(v1[0],v1[2]);
			glVertex2f(v2[0],v2[2]);
		glEnd();
	}
 
	glutSwapBuffers();
	//usleep(30000);
}


void initialize () 
{
    glMatrixMode(GL_PROJECTION);												// select projection matrix
    glViewport(0, 0, win.width, win.height);									// set the viewport
    glMatrixMode(GL_PROJECTION);												// set matrix mode
    glLoadIdentity();															// reset projection matrix
    GLfloat aspect = (GLfloat) win.width / win.height;
	gluPerspective(win.field_of_view_angle, aspect, win.z_near, win.z_far);		// set up a perspective projection matrix
    glMatrixMode(GL_MODELVIEW);													// specify which matrix is the current matrix
    glShadeModel( GL_SMOOTH );
    glClearDepth( 1.0f );														// specify the clear value for the depth buffer
    glEnable( GL_DEPTH_TEST );
    glDepthFunc( GL_LEQUAL );
    glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );						// specify implementation-specific hints
	glClearColor(0.0, 0.0, 0.0, 1.0);											// specify clear values for the color buffers						

	//The part where the space is defined		TODO
	/*Compound* compound = new Compound();
	Euclidean* euclidean = new Euclidean();
	space = std::tr1::shared_ptr<Manifold>(compound);
	Manifold::PointOfReferencePtr euclidPOR(new Euclidean::PointOfReference(euclidean));
	por = Manifold::PointOfReferencePtr(new Compound::PointOfReference(euclidPOR, compound));*/
	SurfaceOfRevolution<PortalSpace2d>* sor = new SurfaceOfRevolution<PortalSpace2d>();
	space = std::tr1::shared_ptr<Manifold>(sor);
	por = Manifold::PointOfReferencePtr(new SurfaceOfRevolution<PortalSpace2d>::PointOfReference(sor));
	/*std::tr1::shared_ptr<Hyperbolic> hyperbolic(new Hyperbolic());
	space = std::tr1::static_pointer_cast<Manifold>(hyperbolic);
	por = std::tr1::shared_ptr<Hyperbolic::PointOfReference>(new Hyperbolic::PointOfReference(hyperbolic.get()));*/
	/*Euclidean* euclidean = new Euclidean();
	space = std::tr1::shared_ptr<Manifold>(euclidean);
	por = Manifold::PointOfReferencePtr(new Euclidean::PointOfReference(euclidean));*/
	/*R2xS1* r2xs1 = new R2xS1();
	space = r2xs1;
	por = new R2xS1::PointOfReference(r2xs1);*/
	triangleList = space->icosahedron(por, 0.2);
	glColor3f(1.0f,1.0f,1.0f);
	
	display();
	
	/*srand(5);
	while(1) {				//Randomly moves for debugging. This way, I don't have to keep trying to crash it myself.
		switch(rand() % 4) {
			case 0:
				por->move((rand()*2-1.0)/RAND_MAX*Vector3d(1,0,0));
				break;
			case 1:
				por->move((rand()*2-1.0)/RAND_MAX*Vector3d(0,1,0));
				break;
			case 2:
				por->move((rand()*2-1.0)/RAND_MAX*Vector3d(0,0,1));
				break;
			case 3:
				por->move(Vector3d::Random());
				break;
		}
		display();
	}*/
}


void keyboard(unsigned char key, int mousePositionX, int mousePositionY)		
{ 
	//std::cout << key;
	//std::cout.flush();
	switch(key) {
		case 'w':
			//por->move((Vector3d() << 0,01,0).finished());
			por = space->getPointOfReference(por,Vector3d(0,SPEED,0));
			break;
		case 'e':
			//por->move(Vector3d(1/3.,2/3.,2/3.));
			//por->move(Vector3d(0.6,0,0.8));
			por = space->getPointOfReference(por,Vector3d(0,0,SPEED));
			break;
		case 'q':
			//por->move(Vector3d(-1/3.,-2/3.,-2/3.));
			//por->move(Vector3d(-0.6,0,-0.8));
			por = space->getPointOfReference(por,Vector3d(0,0,-SPEED));
			break;
		case 's':
			por = space->getPointOfReference(por,Vector3d(0,-SPEED,0));
			break;
		case 'a':
			por = space->getPointOfReference(por,Vector3d(-SPEED,0,0));
			break;
		case 'd':
			por = space->getPointOfReference(por,Vector3d(SPEED,0,0));
			break;
		case KEY_ESCAPE:
			exit(0);
			break;

		default:
			break;
	}
	
	display();
}

int main(int argc, char **argv) 
{
	// set window values
	win.width = 640;
	win.height = 480;
	win.title = (char*)"Manifold";
	win.field_of_view_angle = 45;
	win.z_near = 1.0f;
	win.z_far = 500.0f;

	// initialize and run program
	glutInit(&argc, argv);                                      // GLUT initialization
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );  // Display Mode
	glutInitWindowSize(win.width,win.height);					// set window size
	glutCreateWindow(win.title);								// create Window
	glutDisplayFunc(display);									// register Display Function
	//glutIdleFunc( display );									// register Idle Function
	glutKeyboardFunc( keyboard );								// register Keyboard Handler
	initialize();
	glutMainLoop();												// run GLUT mainloop
	return 0;
}

