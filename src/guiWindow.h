#pragma once
#include "ui_guiWindow.h"

#include <qtimer.h>
#include <qstring.h>
//VTK
#include <QElapsedTimer>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPlaneWidget.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkPlaneWidget.h>
#include <vtkImplicitPlaneWidget.h>
#include <vtkAxesActor.h>
#include <vtkPropAssembly.h>
#include <vtkFollower.h>
#include <vtkVectorText.h>
#include <vtkProp3DCollection.h>
#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>
#include <vtkLineSource.h>
#include <vtkMatrix4x4.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPlane.h>
#include <vtkCamera.h>
#include <vtkCornerAnnotation.h>
#include <vtkTextActor.h>
#include <vtkVectorText.h>
#include <vtkLinearExtrusionFilter.h>
#include <vtkTextActor3D.h>
#include <vtkFollower.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkRegularPolygonSource.h>

#include "expm_boost.h"
#include <boost/algorithm/string.hpp>

#define EIGEN2_SUPPORT
#include <Eigen/dense>
#include <Eigen/Geometry>
#include <Eigen/Eigen2Support>

//#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/LeastSquares>
#include <Eigen/LU>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/base_object.hpp>

#include "DataStrUDPTip.h"
#include "ASIOUDPDevice.h"
#include <stdlib.h>

#define PLOT_REFSPLINE
#define PLOT_PLANESTEER

using namespace std;

class guiWindow : public QMainWindow, private Ui::guiWindow
{
	Q_OBJECT
	
public:
	guiWindow( QMainWindow *parent = 0 );
	~guiWindow();

private:
	
	TipData *_lastTipData;
	bool once_;
	Eigen::Matrix3d TipRot__,RefRot__;
	Eigen::Vector3d TipPos__,RefPos__, CenterS__;
	Eigen::Vector4d c_;
	double EuclideanDist_;
	vector<Eigen::Vector3d> *tip_list_;
	vector<Eigen::Vector3d> *tip_refList_;
	std::string Time_elapsed_,CurvatureY_,CurvatureZ_,CurvatureMean_,BishopY,BishopZ,arcL_,BishopY2,BishopZ2,arcL2_,inclination_;

	ASIOUDPDevice *UDPReceiver,*UDPReceiverInfo;
	boost::function<void(const unsigned char*,size_t)> stringFunction,stringFunctionInfo;
	void getMessageUDP(const unsigned char *data,size_t l);
	void getMessageUDPInfo(const unsigned char *datax,size_t l);
	
	
	void DisplayCentroid(Eigen::Vector4d &point,unsigned int R, unsigned int G, unsigned int B,unsigned int sizep);
	void DisplayPoints(std::vector<Eigen::Vector3d> &tip_, unsigned int sensor_name, unsigned int R, unsigned int G, unsigned int B,unsigned int sizep, const std::string name);
	void DisplaySplineTip(std::vector<Eigen::Vector3d> &point_, double R, double G, double B);
	void DisplaySplineRef(std::vector<Eigen::Vector3d> &point_, double R, double G, double B);
	void DisplayPlaneS(Eigen::Vector4d &CenterPoint,Eigen::Matrix3d &TransfM,float R, float G, float B);
	//void showSteeringPlane(bool k);
	void Display();

	bool showSteeringPlane_;
	 //VTK
	 vtkSmartPointer< vtkRenderer > ren_;
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor_ ;
	QVector<vtkSmartPointer<vtkPoints>> *SensorPoint_;
	QVector<vtkSmartPointer<vtkPolyData>> *PolydataPoints_,*PolydataPointsGlyph_;
	QVector<vtkSmartPointer<vtkPolyDataMapper>> *MapperPoints_,*sensorNameMapper_;
	QVector<vtkSmartPointer<vtkActor>> *actorPoints_;
	QVector<vtkSmartPointer<vtkFollower>> *sensorNameTextActor_;
	QVector<vtkSmartPointer<vtkPoints>> *points_;
	QVector<vtkSmartPointer<vtkVertexGlyphFilter>> *Glypfilter_;
	QVector<vtkSmartPointer<vtkVectorText>> *sensorNameText_;
	
	
	vtkSmartPointer<vtkActor> ActorPlaneR_,ActorPlaneR2_,ActorCircle_,actorCentroid_;
	vtkSmartPointer<vtkPlaneWidget> SteerPlane_,RefPlane_;
	vtkSmartPointer<vtkPolyData> SteerPlaneMapper_,RefPlaneMapper_,polydataCentroid_;
	vtkSmartPointer<vtkPolyDataMapper> SteerPlanePolyMapper_,RefPlanePolyMapper_,circlemapper_,mapperCentroid_;
	vtkSmartPointer<vtkRegularPolygonSource> circleS_;
	vtkSmartPointer<vtkPoints> Centroidpoints_;
	vtkSmartPointer<vtkCellArray> Centroidvertices;
	vtkSmartPointer<vtkAxesActor> AuroraAxes_, RefAxes_, LocalFrameAxes_, SteeringPlaneAxes_,RefFrameAxes_;
	vtkSmartPointer<vtkCamera> camera_;
	vtkSmartPointer<vtkMatrix4x4> transform_view;

	vtkIdType pid[1];
	vtkSmartPointer<vtkCornerAnnotation>  TextInfoError_;
	vtkSmartPointer<vtkParametricSpline> Refspline;
	vtkSmartPointer<vtkParametricSpline> referenceTrack_;
	vtkSmartPointer<vtkParametricFunctionSource> referenceTrack_functionSource_;
	vtkSmartPointer<vtkPolyDataMapper> referenceTrackMapper_;
	vtkSmartPointer<vtkActor> referenceTrackactor_;
	vtkSmartPointer<vtkPoints> Points_;
	
	//tip and bottom coordiante system orientation
	vtkSmartPointer<vtkOrientationMarkerWidget> Tip_,probe_origin_;

	//spline
	vtkSmartPointer<vtkActor> TrjActorTip_,TrjActorRef_;

	std::vector<Eigen::Vector3d> *RefSplineGenerated_;
	unsigned int Num_sensors;
	bool isProbePause_;
	QTimer *refresh_;
	private slots:
		void plot();
		void getData();
		void startProbe();
		void stopProbe();
		void pauseProbe();
		void generateBishop();
	signals:
			void valueChanged();
			void InfovalueChanged();
		
};
