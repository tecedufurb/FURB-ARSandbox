/*
 * Thing.h
 *
 *  Created on: 9 de mai de 2016
 *      Author: nb-0289a
 */

#ifndef THING_H_
#define THING_H_

#include <vector>
#include <Threads/Mutex.h>
#include <Threads/Cond.h>
#include <Threads/TripleBuffer.h>
#include <USB/Context.h>
#include <Math/Matrix.h>
#include <Geometry/Point.h>
#include <Geometry/AffineCombiner.h>
#include <Geometry/Plane.h>
#include <GL/gl.h>
#include <GL/GLColor.h>
#include <GL/GLObject.h>
#include <Images/ExtractBlobs.h>
#include <Vrui/Application.h>
#include <Vrui/Tool.h>
#include <Vrui/GenericToolFactory.h>
#include <Kinect/FrameBuffer.h>
#include <Kinect/Camera.h>


unsigned int* blobIdImage; // An image of blob IDs
int blobMergeDepth; // Maximum depth difference between neighboring pixels in the same blob
bool capturingBackground; // Flag if the Kinect camera is currently capturing a background frame
unsigned int numBackgroundFrames; // Number of frames to capture for background removal
typedef Kinect::FrameSource::DepthCorrection::PixelCorrection PixelDepthCorrection; // Type for per-pixel depth correction factors
typedef double Scalar; // Scalar type for points
typedef Geometry::Plane<Scalar,3> OPlane; // Plane in 3D object space
typedef Geometry::Point<Scalar,2> PPoint; // Point in 2D projection space
typedef Geometry::Point<Scalar,3> OPoint; // Point in 3D object space
typedef Kinect::FrameSource::DepthPixel DepthPixel; // Type for depth image pixels


	struct DepthCentroidBlob:public Images::BboxBlob<Images::Blob<DepthPixel> > // Structure to calculate 3D centroids of blobs in depth image space
		{
		/* Embedded classes: */
		public:
		typedef DepthPixel Pixel;
		typedef Images::BboxBlob<Images::Blob<DepthPixel> > Base;

		struct Creator:public Base::Creator
			{
			/* Elements: */
			public:
			unsigned int frameSize[2]; // Size of depth images
			PixelDepthCorrection* pixelDepthCorrection; // Buffer of per-pixel depth correction coefficients
			Kinect::FrameSource::IntrinsicParameters::PTransform depthProjection; // Transformation from depth image space to camera space
			};

		/* Elements: */
		Kinect::FrameSource::IntrinsicParameters::PTransform::HVector c; // Accumulated centroid components (x, y, z) and total weight

		/* Constructors and destructors: */
		DepthCentroidBlob(unsigned int x,unsigned int y,const Pixel& pixel,const Creator& creator)
			:Base(x,y,pixel,creator)
			{
			/* Calculate the pixel's corrected depth value: */
			double px=double(x)+0.5;
			double py=double(y)+0.5;
			double pz=creator.pixelDepthCorrection[y*creator.frameSize[0]+x].correct(float(pixel));

			/* Unproject the pixel to calculate its centroid accumulation weight as camera-space z coordinate to the fourth: */
			const Kinect::FrameSource::IntrinsicParameters::PTransform::Matrix& m=creator.depthProjection.getMatrix();
			double weight=Math::sqr(Math::sqr((m(2,0)*px+m(2,1)*py+m(2,2)*pz+m(2,3))/(m(3,0)*px+m(3,1)*py+m(3,2)*pz+m(3,3))));

			/* Accumulate the pixel: */
			c[0]=px*weight;
			c[1]=py*weight;
			c[2]=pz*weight;
			c[3]=weight;
			}

		/* Methods: */
		void addPixel(unsigned int x,unsigned int y,const Pixel& pixel,const Creator& creator)
			{
			Base::addPixel(x,y,pixel,creator);

			/* Calculate the pixel's corrected depth value: */
			double px=double(x)+0.5;
			double py=double(y)+0.5;
			double pz=creator.pixelDepthCorrection[y*creator.frameSize[0]+x].correct(float(pixel));

			/* Unproject the pixel to calculate its centroid accumulation weight as camera-space z coordinate to the fourth: */
			const Kinect::FrameSource::IntrinsicParameters::PTransform::Matrix& m=creator.depthProjection.getMatrix();
			double weight=Math::sqr(Math::sqr((m(2,0)*px+m(2,1)*py+m(2,2)*pz+m(2,3))/(m(3,0)*px+m(3,1)*py+m(3,2)*pz+m(3,3))));

			/* Accumulate the pixel: */
			c[0]+=px*weight;
			c[1]+=py*weight;
			c[2]+=pz*weight;
			c[3]+=weight;
			}
		void merge(const DepthCentroidBlob& other,const Creator& creator)
			{
			Base::merge(other,creator);

			for(int i=0;i<4;++i)
				c[i]+=other.c[i];
			}
		OPoint getCentroid(const Kinect::FrameSource::IntrinsicParameters::PTransform& depthProjection) const // Returns the blob's centroid in camera space
			{
			return depthProjection.transform(c).toPoint();
			}
		};

	struct TiePoint // Tie point between 3D object space and 2D projector space
			{
			/* Elements: */
			public:
			PPoint p; // Projection-space point
			OPoint o; // Object-space point
			};

PixelDepthCorrection* pixelDepthCorrection; // Buffer of per-pixel depth correction coefficients
DepthCentroidBlob* currentBlob; // The currently selected target blob
OPoint currentCentroid; // Centroid of the currently selected target blob in camera space
OPlane basePlane; // Base plane of the configured sandbox area
OPoint basePlaneCorners[4]; // Corners of the configured sandbox area

std::vector<TiePoint> tiePoints; // List of already captured tie points
int numTiePoints[2]; // Number of tie points in x and y
unsigned int numCaptureFrames; // Number of tie point frames still to capture
Geometry::AffineCombiner<double,3> tiePointCombiner; // Combiner to average multiple tie point frames
int imageSize[2]; // Size of projector image

#endif /* THING_H_ */
