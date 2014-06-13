#include <stdio.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <vector>
#include <iostream>
#include <unistd.h>
#include "Assert.h"
#include "Euclidean.h"
//#include "Hyperbolic.h"
#include "SurfaceOfRevolution.h"
#include "PortalSpace2d2.h"
#include "Compound.h"
#include <Eigen/Core>
#include <cstdlib>
using Eigen::Matrix3d;
using Eigen::Vector3d;

//It might be a good idea to have a separate matrix for rotation and for the distortion of space, since rotation can be inverted by transposing, and if the distortion is just a function of the position the inverse could presumably be calculated easily, but their product would have to be calculated the hard way.

#define KEY_ESCAPE 27

Compound* space;
Compound::PointOfReference* por;
//std::tr1::shared_ptr<Manifold> space;
//std::tr1::shared_ptr<Manifold::PointOfReference> por;
std::vector<Compound::Triangle> triangleList;

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
	Vector3d zero(0,0,0);
	for(int i=0; i<triangleList.size(); ++i) {
		Vector3d v0 = por->vectorFromPointAndNearVector(triangleList[i][0],zero);
		Vector3d v1 = por->vectorFromPointAndNearVector(triangleList[i][1],zero);
		Vector3d v2 = por->vectorFromPointAndNearVector(triangleList[i][2],zero);
		/*if(v0.squaredNorm() < EPSILON) {
			std::cout << "Vertex missed." << std::endl;
		} else {
			std::cout << "Vertex hit." << std::endl;
		}*/
		if(v0[1] < 0 || v1[1] < 0 || v2[1] < 0)
			continue;
		v0 /= v0[1];
		v1 /= v1[1];
		v2 /= v2[1];
		/*std::cout << v0list[j0][0] << "," << v0list[j0][1] << "," << v0list[j0][2] << "\n";
		std::cout << v1list[j1][0] << "," << v1list[j1][1] << "," << v1list[j1][2] << "\n";
		std::cout << v2list[j2][0] << "," << v2list[j2][1] << "," << v2list[j2][2] << "\n";*/
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

	space = new Compound();
	
	/*SurfaceOfRevolution<PortalSpace2d>* wormhole = new SurfaceOfRevolution<PortalSpace2d>();
	por = new Compound::PointOfReference(Manifold::PointOfReferencePtr(new SurfaceOfRevolution<PortalSpace2d>::PointOfReference(wormhole)));
	triangleList = por->icosahedron(0.1);*/
	
	Euclidean* euclidean = new Euclidean();
	euclidean = new Euclidean();
	//std::cout << "Euclidean:\t" << euclidean << std::endl;
	SurfaceOfRevolution<PortalSpace2d>* wormhole = new SurfaceOfRevolution<PortalSpace2d>();
	std::cout << "Main3.cpp wormhole bottleneck:\t" << wormhole->getBottleneckCircumference() << std::endl;
	//wormhole = new SurfaceOfRevolution<PortalSpace2d>();
	//std::cout << "Wormhole:\t" << wormhole << std::endl;
	por = new Compound::PointOfReference(Manifold::PointOfReferencePtr(new Euclidean::PointOfReference(euclidean)));
	//por = new Compound::PointOfReference(std::tr1::shared_ptr<Manifold::PointOfReference>(new SurfaceOfRevolution<PortalSpace2d>::PointOfReference(wormhole)));
	//por->move(Vector3d(1,0,0));
	SurfaceOfRevolution<PortalSpace2d>::PortalPtr wormholePortal = SurfaceOfRevolution<PortalSpace2d>::PortalPtr(new SurfaceOfRevolution<PortalSpace2d>::Portal(false, wormhole));
	wormhole->addPortal(wormholePortal);
	//std::cout << "Main3.cpp portal radius:\t" << wormholePortal->getRadiusOfCurvature() << std::endl;
	Euclidean::PortalPtr euclideanPortal = Euclidean::PortalPtr(new Euclidean::Portal(Vector3d(0,-3,0),-wormholePortal->getRadiusOfCurvature(),euclidean));
	//Euclidean::PortalPtr euclideanPortal = Euclidean::PortalPtr(new Euclidean::Portal(Vector3d(0,-2.5,0),1,euclidean));
	/*std::cout << "Main3.cpp wormholePortal circumference:\t" << wormholePortal->getCircumference() << std::endl;
	std::cout << "Main3.cpp euclideanPortal circumference:\t" << euclideanPortal->getCircumference() << std::endl;
	std::cout << "Main3.cpp difference:\t" << fabs(wormholePortal->getCircumference()-euclideanPortal->getCircumference()) << std::endl;*/
	assert(fabs(wormholePortal->getCircumference() - euclideanPortal->getCircumference()) < EPSILON);
	euclidean->addPortal(euclideanPortal);
	euclideanPortal->setExit(wormholePortal.get());
	wormholePortal->setExit(euclideanPortal.get());
	
	//Compound::PointOfReferencePtr tempPor = Compound::PointOfReferencePtr(new Compound::PointOfReference(Manifold::PointOfReferencePtr(new SurfaceOfRevolution<PortalSpace2d>::PointOfReference(wormhole))));
	triangleList = por->icosahedron(0.1);
	
	/*SurfaceOfRevolution<PortalSpace2d>* wormhole = new SurfaceOfRevolution<PortalSpace2d>();
	por = new Compound::PointOfReference(std::tr1::shared_ptr<Manifold::PointOfReference>(new SurfaceOfRevolution<PortalSpace2d>::PointOfReference(wormhole)));
	SurfaceOfRevolution<PortalSpace2d>::PortalPtr portal = SurfaceOfRevolution<PortalSpace2d>::PortalPtr(new SurfaceOfRevolution<PortalSpace2d>::Portal(0.1, wormhole));
	portal->setExit(portal.get());
	wormhole->addPortal(portal);
	
	Euclidean* euclidean = new Euclidean();
	por = new Compound::PointOfReference(std::tr1::shared_ptr<Manifold::PointOfReference>(new Euclidean::PointOfReference(euclidean)));
	
	Manifold::Portal* portal0 = new Euclidean::Portal(Vector3d(0,0,0),1,euclidean);
	Manifold::Portal* portal1 = new Euclidean::Portal(Vector3d(0,0,0),2,euclidean);
	portal0->setExit(portal1);
	portal1->setExit(portal0);
	euclidean->addPortal(Manifold::PortalPtr(portal0));
	euclidean->addPortal(Manifold::PortalPtr(portal1));*/
	
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
			por->move(Vector3d(0,0.05,0));
			break;
		case 'e':
			//por->move(Vector3d(1/3.,2/3.,2/3.));
			//por->move(Vector3d(0.6,0,0.8));
			//por->move(Vector3d(0,0,0.051));
			por->rotate((Matrix3d() <<
					cos(0.1),	sin(0.1),	0,
					-sin(0.1),	cos(0.1),	0,
					0,			0,			1).finished());
			break;
		case 'q':
			//por->move(Vector3d(-1/3.,-2/3.,-2/3.));
			//por->move(Vector3d(-0.6,0,-0.8));
			//por->move(Vector3d(0,0,-0.051));
			por->rotate((Matrix3d() <<
					cos(0.1),	-sin(0.1),	0,
					sin(0.1),	cos(0.1),	0,
					0,			0,			1).finished());
			break;
		case 's':
			por->move(Vector3d(0,-0.05,0));
			break;
		case 'a':
			por->move(Vector3d(-0.05,0,0));
			break;
		case 'd':
			por->move(Vector3d(0.05,0,0));
			break;
		case KEY_ESCAPE:
			exit(0);
			break;

		default:
			break;
	}
	//std::cout << "Main3.cpp space:\t" << por->getPosition()->getSpace()->getType() << std::endl;
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

