#include <stdio.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <vector>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include "Assert.h"
#include "Euclidean.h"
//#include "Hyperbolic.h"
#include "SurfaceOfRevolution.h"
#include "PortalSpace2d2.h"
#include "Compound.h"
#include "Image.h"
#include "ImageOut.h"
#include "Color.h"
#include <Eigen/Core>
#include <cstdlib>
using Eigen::Matrix3d;
using Eigen::Vector3d;

//Raytracer

Compound* space;
Compound::PointOfReference* por;

int main(int argc, char* argv[]){
	
    std::clock_t start;
    double duration;

    start = std::clock();

	space = new Compound();
	
	/*SurfaceOfRevolution<PortalSpace2d>* wormhole = new SurfaceOfRevolution<PortalSpace2d>();
	por = new Compound::PointOfReference(Manifold::PointOfReferencePtr(new SurfaceOfRevolution<PortalSpace2d>::PointOfReference(wormhole)));
	triangleList = por->icosahedron(0.1);*/
	
	Euclidean* euclidean0 = new Euclidean();
	Euclidean* euclidean1 = new Euclidean();
	SurfaceOfRevolution<PortalSpace2d>* wormhole = new SurfaceOfRevolution<PortalSpace2d>();
	por = new Compound::PointOfReference(Manifold::PointOfReferencePtr(new Euclidean::PointOfReference(euclidean0)));
	por->move(Vector3d(0,-2,0));
	SurfaceOfRevolution<PortalSpace2d>::PortalPtr wormholePortal0 = SurfaceOfRevolution<PortalSpace2d>::PortalPtr(new SurfaceOfRevolution<PortalSpace2d>::Portal(false, wormhole));
	wormhole->addPortal(wormholePortal0);
	SurfaceOfRevolution<PortalSpace2d>::PortalPtr wormholePortal1 = SurfaceOfRevolution<PortalSpace2d>::PortalPtr(new SurfaceOfRevolution<PortalSpace2d>::Portal(true, wormhole));
	wormholePortal1->setInvert(true);
	wormhole->addPortal(wormholePortal1);
	Euclidean::PortalPtr euclideanPortal0 = Euclidean::PortalPtr(new Euclidean::Portal(Vector3d(0,0,0),fabs(wormholePortal0->getRadiusOfCurvature()),euclidean0));
	Euclidean::PortalPtr euclideanPortal1 = Euclidean::PortalPtr(new Euclidean::Portal(Vector3d(0,0,0),fabs(wormholePortal1->getRadiusOfCurvature()),euclidean1));
	assert(fabs(wormholePortal0->getCircumference() - euclideanPortal0->getCircumference()) < EPSILON);
	assert(fabs(wormholePortal1->getCircumference() - euclideanPortal1->getCircumference()) < EPSILON);
	euclidean0->addPortal(euclideanPortal0);
	euclidean1->addPortal(euclideanPortal1);
	euclideanPortal0->setMutualExits(wormholePortal0.get());
	euclideanPortal1->setMutualExits(wormholePortal1.get(), -Matrix3d::Identity());
	
	/*Matrix3d randrot;		//Randomizes the rotation.
	do {
		randrot.setRandom();
		while((randrot*randrot.transpose()-Matrix3d::Identity()).norm() > EPSILON) {
			randrot += randrot.inverse().transpose();
			randrot /= 2;
		}
	} while(randrot.determinant() <= 0);
	por->rotate(randrot);*/
	
	ImagePtr image(new Image);
	for(int i=0; i<WIDTH; ++i) {
		double x = (i*2.0/WIDTH-1.0);
		for(int j=0; j<HEIGHT; ++j) {
			double z = (1.0-j*2.0/HEIGHT);
			//(*image)[j][i] = Color(0.0,0.0,0.0);
			
			Vector3d dir = Vector3d(x,1.0,z).normalized();
			Compound::PointOfReferencePtr a = por->pointOfReferenceFromVector(100*dir);
			Manifold::PointOfReferencePtr b = a->getPointOfReference();
			if(b->getSpace() != euclidean0 && b->getSpace() != euclidean1) {
				(*image)[j][i] = Color(0.5,0.5,0.5);
				//std::cout << "Main4.cpp other" << std::endl;
			} else {
				Euclidean::PointOfReference* c = (Euclidean::PointOfReference*) b.get();
				Vector3d outDir = c->getOrientation()*dir;

				double theta = acos(outDir[2]);
				double phi = atan2(outDir[1],outDir[0]);
				double theta01 = theta/M_PI;
				double phi01 = phi/(2*M_PI);
				if((int)(floor(10*theta01) + floor(10*phi01)) % 2 == 0) {
					if(b->getSpace() == euclidean0) {
						(*image)[j][i] = Color(0.0,0.0,0.0);
					} else {
						(*image)[j][i] = Color(1.0,0.0,0.0);
						//std::cout << "Main4.cpp euclidean1" << std::endl;
					}
				} else {
					if(b->getSpace() == euclidean0) {
						(*image)[j][i] = Color(1.0,1.0,1.0);
					} else {
						(*image)[j][i] = Color(0.0,0.0,1.0);
						//std::cout << "Main4.cpp euclidean1" << std::endl;
					}
				}
			}
		}
	}
	ImageOut::draw(image,"output.png");

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    
    std::cout << "Running time: " << duration << " seconds." << std::endl;
	return 0;
}

