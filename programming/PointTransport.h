#ifndef point_transport_h
#define point_transport_h

#include <tr1/memory>
#include <Eigen/Core>
using Eigen::Matrix3d;
using Eigen::Vector3d;

class PointTransport {
	public:
		Vector3d getPosition();
		double getDistance();
		PointTransport(Vector3d position, double distance);
	private:
		Vector3d position;
		double distance;
};

typedef std::tr1::shared_ptr<PointTransport> PointTransportPtr;

#endif
