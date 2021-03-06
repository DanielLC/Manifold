#include <stdio.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <vector>
#include <iostream>
#include <unistd.h>
//#include "PortalSpace2d2.h"
#include "Hyperbolic2d.h"
#include <Eigen/Core>
using Eigen::Matrix2d;
using Eigen::Vector2d;

//It might be a good idea to have a separate matrix for rotation and for the distortion of space, since rotation can be inverted by transposing, and if the distortion is just a function of the position the inverse could presumably be calculated easily, but their product would have to be calculated the hard way.

#define KEY_ESCAPE 27

Hyperbolic2d* space;
Hyperbolic2d::Point position;
double rotation;
//std::tr1::shared_ptr<Manifold> space;
//std::tr1::shared_ptr<Manifold::PointOfReference> por;
std::vector<Hyperbolic2d::Point> pointList;

Matrix2d rotate(double theta) {
	Matrix2d out;
	out << cos(theta), -sin(theta), sin(theta), cos(theta);
	return out;
}

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
	for(int i=0; i<pointList.size(); ++i) {
		//::cout << "Point:	(" << pointList[i].getCoordinates()[0] << ",	" << pointList[i].getCoordinates()[1] << ")\n";
		Vector2d point = position.vectorFromPoint(pointList[i]);
		//std::cout << "Direction before rotation:\n" << point << "\n";
		//std::cout << "Rotation:\n" << rotate(rotation) << "\n";
		point = rotate(-rotation)*point;
		//std::cout << "Direction after rotation:\n" << point << "\n";
		glBegin(GL_POINTS);
			glVertex2f(point[0], point[1]);
		glEnd();
	}
 
	glutSwapBuffers();
	usleep(30000);
}

void constructPointList() {
	double theta = M_PI*(sqrt(5)-1);
	Matrix2d m;
	m << cos(theta), -sin(theta),
		sin(theta), cos(theta);
	m *= 1.05;
	Vector2d v(0.1,0.);
	for(int i=0; i<50; ++i) {
		pointList.push_back(position.pointFromVector(v));
		v = m*v;
	}
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
	Hyperbolic2d* hyperbolic = new Hyperbolic2d();
	space = hyperbolic;
	constructPointList();
	glColor3f(1.0f,1.0f,1.0f);
}

void move(double x, double y) {
	std::pair<Hyperbolic2d::Point, double> pointAndRot = position.pointAndRotFromVector(rotate(rotation)*Vector2d(x,y));
	position = pointAndRot.first;
	rotation += pointAndRot.second;
}

void keyboard(unsigned char key, int mousePositionX, int mousePositionY)		
{
	//std::cout << key;
	//std::cout.flush();
	switch(key) {
		case 'w':
			move(0,0.1);
			break;
		case 'e':
			rotation -= 0.1;
			break;
		case 'q':
			rotation += 0.1;
			break;
		case 's':
			move(0,-0.1);
			break;
		case 'a':
			move(-0.1,0);
			break;
		case 'd':
			move(0.1,0);
			break;
		case KEY_ESCAPE:
			exit(0);
			break;

		default:
			break;
	}
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
	glutIdleFunc( display );									// register Idle Function
	glutKeyboardFunc( keyboard );								// register Keyboard Handler
	initialize();
	glutMainLoop();												// run GLUT mainloop
	return 0;
}

