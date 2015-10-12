#include "guiWindow.h"
#include <iostream>
#include <sstream>
#include <qdatetime.h>
#include <QFileDialog>
#include <QtSerialPort/QSerialPortInfo>
#include <qdebug.h>

guiWindow::guiWindow( QMainWindow *parent ) : QMainWindow( parent )
{
	Num_sensors = 1;
	EuclideanDist_ = 0.0;
	_lastTipData = new TipData();

	UDPReceiver = new ASIOUDPDevice(3001, "127.0.0.1",3000);
	stringFunction = boost::function<void(const unsigned char*,size_t)>(boost::bind(&guiWindow::getMessageUDP,this,_1,_2)); 
	UDPReceiver->SetReadCallback(stringFunction);

	UDPReceiverInfo = new ASIOUDPDevice(4002, "127.0.0.1",4003);
	stringFunctionInfo = boost::function<void(const unsigned char*,size_t)>(boost::bind(&guiWindow::getMessageUDPInfo,this,_1,_2)); 
	UDPReceiverInfo->SetReadCallback(stringFunctionInfo);

	RefSplineGenerated_ = new std::vector<Eigen::Vector3d>();

	TipRot__ = Eigen::Matrix3d::Identity();
	TipPos__ = Eigen::Vector3d::Identity();
	RefRot__ = Eigen::Matrix3d::Identity();
	RefPos__ = Eigen::Vector3d::Identity();
	CenterS__= Eigen::Vector3d::Identity();

	tip_list_ = new vector<Eigen::Vector3d>();
	tip_refList_ = new vector<Eigen::Vector3d>();
	setupUi(this);
	refresh_ = new QTimer();
	connect(refresh_,SIGNAL(timeout()),this,SLOT(getData()));
	connect(this, SIGNAL(valueChanged()),this, SLOT(plot()));
	connect(this, SIGNAL(InfovalueChanged()),this, SLOT(generateBishop()));
	connect(Start_button,SIGNAL(clicked(bool)),this,SLOT(startProbe()));
	connect(Stop_button,SIGNAL(clicked(bool)),this,SLOT(stopProbe()));
	connect(Pause_button,SIGNAL(clicked(bool)),this,SLOT(pauseProbe()));
	Sensor_File_Save_Box->setText(QString("Exp#1"));
	isProbePause_ = false;
	

	showSteeringPlane_ = false;
	//vtk
	ren_ = vtkSmartPointer<vtkRenderer>::New();

	qvtkWidget->GetRenderWindow()->AddRenderer(ren_);
	ren_->SetBackground(0,0,0);
	ren_->SetLightFollowCamera(1);
	ren_->ResetCameraClippingRange();
	AuroraAxes_ =  vtkSmartPointer<vtkAxesActor>::New();
	AuroraAxes_->SetTotalLength(20,20,20);
	AuroraAxes_->SetOrigin(0.0,0.0,0.0);
	RefAxes_ =  vtkSmartPointer<vtkAxesActor>::New();
	RefAxes_->SetTotalLength(3,3,3);
	LocalFrameAxes_ =  vtkSmartPointer<vtkAxesActor>::New();
	LocalFrameAxes_->SetTotalLength(10,10,10);
	LocalFrameAxes_->SetOrigin(0.0,0.0,0.0);
	LocalFrameAxes_->SetShaftTypeToCylinder();
	LocalFrameAxes_->SetAxisLabels(0);

	RefFrameAxes_ =  vtkSmartPointer<vtkAxesActor>::New();
	RefFrameAxes_->SetTotalLength(10,10,10);
	RefFrameAxes_->SetOrigin(0.0,0.0,0.0);
	RefFrameAxes_->SetShaftTypeToCylinder();
	RefFrameAxes_->SetAxisLabels(0);


	SteeringPlaneAxes_=  vtkSmartPointer<vtkAxesActor>::New();
	SteeringPlaneAxes_->SetTotalLength(2,2,2);
	SteeringPlaneAxes_->SetOrigin(0.0,0.0,0.0);
	ren_->AddActor(LocalFrameAxes_);
	ren_->AddActor(AuroraAxes_);
	ren_->AddActor(RefFrameAxes_);
	//ren_->AddActor(SteeringPlaneAxes_);

	//inint
	SensorPoint_ = new QVector<vtkSmartPointer<vtkPoints>>();
	PolydataPoints_ = new QVector<vtkSmartPointer<vtkPolyData>>();
	PolydataPointsGlyph_ = new QVector<vtkSmartPointer<vtkPolyData>>();
	MapperPoints_ = new QVector<vtkSmartPointer<vtkPolyDataMapper>>();
	sensorNameMapper_ = new QVector<vtkSmartPointer<vtkPolyDataMapper>>();
	actorPoints_ = new QVector<vtkSmartPointer<vtkActor>>();
	sensorNameTextActor_ = new QVector<vtkSmartPointer<vtkFollower>>();
	points_ = new QVector<vtkSmartPointer<vtkPoints>>();
	Glypfilter_ = new QVector<vtkSmartPointer<vtkVertexGlyphFilter>>();
	sensorNameText_ = new QVector<vtkSmartPointer<vtkVectorText>>();


	for(auto i=0;i<Num_sensors+2;i++){
		sensorNameText_->push_back(vtkSmartPointer<vtkVectorText>::New());
		sensorNameText_->at(i)->SetText("");
		sensorNameMapper_->push_back(vtkSmartPointer<vtkPolyDataMapper>::New());
		sensorNameMapper_->at(i)->SetInputConnection(sensorNameText_->at(i)->GetOutputPort());
		sensorNameTextActor_->push_back(vtkSmartPointer<vtkFollower>::New());
		sensorNameTextActor_->at(i)->SetMapper( sensorNameMapper_->at(i) );
		ren_->AddActor(sensorNameTextActor_->at(i));


		actorPoints_->push_back(vtkSmartPointer<vtkActor>::New());
		PolydataPoints_->push_back(vtkSmartPointer<vtkPolyData>::New());
		PolydataPointsGlyph_->push_back(vtkSmartPointer<vtkPolyData>::New());
		MapperPoints_->push_back(vtkSmartPointer<vtkPolyDataMapper>::New());
		Glypfilter_->push_back(vtkSmartPointer<vtkVertexGlyphFilter>::New());
		//Glypfilter_->at(i)->SetInputConnection(PolydataPointsGlyph_->at(i)->GetProducerPort());
		//Glypfilter_->at(i)->AddInput(PolydataPointsGlyph_->at(i));
		points_->push_back(vtkSmartPointer<vtkPoints>::New());
		Glypfilter_->at(i)->AddInput(PolydataPointsGlyph_->at(i));

		//MapperPoints_->at(i)->SetInputConnection(PolydataPointsGlyph_->at(i)->GetProducerPort());
		MapperPoints_->at(i)->SetInputConnection(Glypfilter_->at(i)->GetOutputPort());
		actorPoints_->at(i)->SetMapper(MapperPoints_->at(i));
		ren_->AddActor(actorPoints_->at(i));
	}


	//steering plane display
	SteerPlane_ = vtkSmartPointer<vtkPlaneWidget>::New();;
	SteerPlane_->PlaceWidget(-50.0,50.0,-50.0,50.0,-50.0,50.0);
	SteerPlane_->GetPlaneProperty()->SetLineWidth(2);
	SteerPlane_->GetPlaneProperty()->SetRepresentationToSurface();
	SteerPlane_->SetPlaceFactor(1.0);
	SteerPlaneMapper_ = vtkSmartPointer<vtkPolyData>::New();
	SteerPlanePolyMapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
	ActorPlaneR_ = vtkSmartPointer<vtkActor>::New();
	ActorPlaneR_->SetMapper(SteerPlanePolyMapper_);

	RefPlane_ = vtkSmartPointer<vtkPlaneWidget>::New();
	RefPlane_->PlaceWidget(-60.0,60.0,-60.0,60.0,-60.0,60.0);
	RefPlane_->GetPlaneProperty()->SetLineWidth(2);
	RefPlane_->GetPlaneProperty()->SetRepresentationToSurface();
	RefPlane_->SetPlaceFactor(1.0);
	RefPlaneMapper_ = vtkSmartPointer<vtkPolyData>::New();
	RefPlanePolyMapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
	ActorPlaneR2_ = vtkSmartPointer<vtkActor>::New();
	ActorPlaneR2_->SetMapper(RefPlanePolyMapper_);

	ActorCircle_ = vtkSmartPointer<vtkActor>::New();
	ActorCircle_->GetProperty()->SetOpacity(0.3);
	circleS_ = vtkSmartPointer<vtkRegularPolygonSource>::New();
	circleS_->SetNumberOfSides(50);
	circlemapper_ =vtkSmartPointer<vtkPolyDataMapper>::New();
	circlemapper_->SetInputConnection(circleS_->GetOutputPort());
	ActorCircle_->SetMapper(circlemapper_);
	ren_->AddActor(ActorCircle_);

	Centroidpoints_ = vtkSmartPointer<vtkPoints>::New();
	Centroidvertices = vtkSmartPointer<vtkCellArray>::New();
	polydataCentroid_ = vtkSmartPointer<vtkPolyData>::New();
	mapperCentroid_ = vtkSmartPointer<vtkPolyDataMapper>::New();
	actorCentroid_ =vtkSmartPointer<vtkActor>::New();
	actorCentroid_->SetMapper(mapperCentroid_);
	ren_->AddActor(actorCentroid_);

	//textInfo
	TextInfoError_ = vtkSmartPointer<vtkCornerAnnotation>::New();
	TextInfoError_->SetLinearFontScaleFactor( 2 );
	TextInfoError_->SetNonlinearFontScaleFactor( 1 );
	TextInfoError_->SetMaximumFontSize( 20 );
	ren_->AddViewProp(TextInfoError_);
	Tip_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
	Tip_->SetOutlineColor( 0.9300, 0.5700, 0.1300 );
	probe_origin_ = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
	probe_origin_->SetOutlineColor( 1.0,0.0, 0.0 );

	referenceTrack_ = vtkSmartPointer<vtkParametricSpline>::New();
	referenceTrack_functionSource_= vtkSmartPointer<vtkParametricFunctionSource>::New();
	referenceTrackMapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
	referenceTrackMapper_->SetInputConnection(referenceTrack_functionSource_->GetOutputPort());
	referenceTrackactor_ = vtkSmartPointer<vtkActor>::New();
	referenceTrackactor_->SetMapper(referenceTrackMapper_);

	TrjActorTip_ = vtkSmartPointer<vtkActor>::New();
	ren_->AddActor(TrjActorTip_);

	TrjActorRef_ = vtkSmartPointer<vtkActor>::New();
	ren_->AddActor(TrjActorRef_);

	qvtkWidget->GetRenderWindow()->Render();
	c_ = Eigen::Vector4d::Zero();
	Time_elapsed_ = "0";
	CurvatureY_="0";
	CurvatureZ_="0";
	CurvatureMean_="0.0";
	BishopY = "";
	BishopZ = "";
	arcL_="";
	BishopY2 = "";
	BishopZ2 = "";
	arcL2_="";
	refresh_->start(1);
	once_=false;
}

guiWindow::~guiWindow()
{
	UDPReceiver->Close();
	UDPReceiver->~ASIOUDPDevice();
	UDPReceiverInfo->Close();
	UDPReceiverInfo->~ASIOUDPDevice();
	delete tip_list_;
	delete _lastTipData;
	delete SensorPoint_;
	delete PolydataPoints_;
	delete PolydataPointsGlyph_;
	delete MapperPoints_;
	delete sensorNameMapper_;
	delete actorPoints_;
	delete sensorNameTextActor_;
	delete points_;
	delete Glypfilter_;
	delete sensorNameText_;

}


/*
void guiWindow::showSteeringPlane(bool k){
showSteeringPlane_ = k;
ren_->AddActor(ActorPlaneR_);
}*/

void guiWindow::generateBishop(){

	if((RefSplineGenerated_->size()==0)&&(tip_list_->size()!=0)){
		double step_ = 0.1;
		double max1_ = floor(atof(arcL_.c_str())/step_);
		if(BishopY2=="")
			RefSplineGenerated_->reserve(max1_);
		boost::numeric::ublas::matrix<double> M_= boost::numeric::ublas::zero_matrix<double>(4,4);
		boost::numeric::ublas::matrix<double> M2_= boost::numeric::ublas::zero_matrix<double>(4,4);
		M_(1,0)=atof(BishopY.c_str());
		M_(2,0)=atof(BishopZ.c_str());
		M_(0,1)=-atof(BishopY.c_str());
		M_(0,2)=-atof(BishopZ.c_str());
		M_(0,3)=1.0; 
		 double max2_=0.0;
		if(BishopY2!=""){
			max2_ = floor(atof(arcL2_.c_str())/step_);
			RefSplineGenerated_->reserve(max1_+max2_);
			M2_(1,0)=atof(BishopY2.c_str());
			M2_(2,0)=atof(BishopZ2.c_str());
			M2_(0,1)=-atof(BishopY2.c_str());
			M2_(0,2)=-atof(BishopZ2.c_str());
			M2_(0,3)=1.0; 
		}

		Eigen::Matrix4d output = Eigen::Matrix4d::Identity();
		Eigen::Matrix3d rot_ = Eigen::Matrix3d::Identity();
		Eigen::Vector3d tr_ = Eigen::Vector3d::Zero();
		Eigen::Vector3d tr2_ = Eigen::Vector3d::Zero();
		Eigen::Vector3d TexpM = Eigen::Vector3d::Zero();
		Eigen::Matrix3d RexpM = Eigen::Matrix3d::Identity();

		double tmp_=0.01;
		auto i=1;
		for(i=1;i<max1_;i++){
			tmp_=i*step_;
			boost::numeric::ublas::matrix<double> expM = boost::numeric::ublas::expm_pad(M_,tmp_);
			if(i==max1_){
				for(auto i=0;i<3;i++)
				for(auto j=0;j<3;j++)
				RexpM(i,j) = expM(i,j);
			}

			TexpM(0)=expM(0,3)+tip_list_->at(0).x();
			TexpM(1)=expM(1,3)+tip_list_->at(0).y();
			TexpM(2)=expM(2,3)+tip_list_->at(0).z();
			RefSplineGenerated_->push_back(TexpM);
			if(i==(max1_-1))
					tr2_= TexpM;
		}
		if(BishopY2!=""){
			tmp_=0.01;
			for(auto j=1;j<max2_;j++){
				tmp_=j*step_;
				boost::numeric::ublas::matrix<double> expM = boost::numeric::ublas::expm_pad(M2_,tmp_);
				tr_(0)= expM(0,3);tr_(1)= expM(1,3);tr_(2)= expM(2,3);
				TexpM = RexpM*tr_+tr2_;
				RefSplineGenerated_->push_back(TexpM);
			}
		}
		once_=true;
	}
}
void guiWindow::getMessageUDP(const unsigned char *data,size_t l){

	std::string datas(reinterpret_cast<const char*>(data), l);
	std::istringstream archive_stream(datas);
	boost::archive::text_iarchive archive(archive_stream);
	archive >> *_lastTipData;
	Eigen::Map<Eigen::Matrix3d> TipRot_(_lastTipData->MRot_); 
	TipRot__=TipRot_;
	Eigen::Map<Eigen::Vector3d> TipPos_(_lastTipData->Postip_); 
	TipPos__= TipPos_;
	tip_list_->push_back(TipPos_);

	Eigen::Map<Eigen::Matrix3d> RefRot_(_lastTipData->Mref_); 
	RefRot__=RefRot_;
	Eigen::Map<Eigen::Vector3d> RefPos_(_lastTipData->Posref_); 
	RefPos__= RefPos_;
	tip_refList_->push_back(RefPos__);

	EuclideanDist_ =sqrt((RefPos__-TipPos_).cwiseAbs().sum());
	Eigen::Map<Eigen::Vector3d> CenterS_(_lastTipData->centerS_);
	CenterS__=CenterS_;
	//DEBUG
	//std::cout << " Ref TIP: " << RefPos__ << std::endl;
	//std::cout << " Real TIP: " <<TipPos__  << std::endl;
	emit valueChanged();
}

void guiWindow::getMessageUDPInfo(const unsigned char *datax,size_t l){

	std::string datac(reinterpret_cast<const char*>(datax), l);
	vector<string>  SplitVec = vector<string>();
	boost::split( SplitVec, datac,  boost::is_any_of("##"), boost::token_compress_on );
	Time_elapsed_ =  SplitVec.at(1);
	CurvatureMean_ = SplitVec.at(2);
	BishopY = SplitVec.at(3);
	BishopZ = SplitVec.at(4);
	arcL_ = SplitVec.at(5);
	BishopY2 = SplitVec.at(6);
	BishopZ2 = SplitVec.at(7);
	arcL2_ = SplitVec.at(8);
	inclination_ = SplitVec.at(9);
	
	//CurvatureY_ = SplitVec.at(2);
	//CurvatureZ_ = SplitVec.at(3);
	
		emit InfovalueChanged();
	emit valueChanged();
}



void guiWindow::getData(){
	UDPReceiver->Read();
	UDPReceiverInfo->Read();
}

void guiWindow::startProbe(){
	std::string cmd_="Start-*"+Sensor_File_Save_Box->text().toStdString();
	UDPReceiver->Write(cmd_.c_str());
}

void guiWindow::stopProbe(){
	UDPReceiver->Write("Stop-*");
}

void guiWindow::pauseProbe(){
	if(isProbePause_){
		isProbePause_=false;
		Pause_button->setText("Pause OFF");
		UDPReceiver->Write("Pause0-*");
	}else{
		isProbePause_=true;
		Pause_button->setText("Pause ON");
		UDPReceiver->Write("Pause1-*");
	}
}

void guiWindow::plot(){

	//###### Add main axes to the viewer

	
	c_.head<3>()=TipPos__;
	DisplayCentroid(c_,0,1,0,12);
	qvtkWidget->GetRenderWindow()->Render();

	//LocalFrameAxes_->SetXAxisLabelText("Tangent");
	vtkSmartPointer<vtkMatrix4x4> orient = vtkSmartPointer<vtkMatrix4x4>::New();
	orient->Identity();
	for(auto i=0;i<3;i++)
		for(auto j=0;j<3;j++)
			orient->SetElement(i,j,TipRot__(i,j));
	orient->SetElement(0,3,c_(0));
	orient->SetElement(1,3,c_(1));
	orient->SetElement(2,3,c_(2));
	LocalFrameAxes_->SetUserMatrix(orient);
	qvtkWidget->GetRenderWindow()->Render();

	vtkSmartPointer<vtkMatrix4x4> orient2 = vtkSmartPointer<vtkMatrix4x4>::New();
	orient2->Identity();
	for(auto i=0;i<3;i++)
		for(auto j=0;j<3;j++)
			orient2->SetElement(i,j,RefRot__(i,j));
	orient2->SetElement(0,3,RefPos__(0));
	orient2->SetElement(1,3,RefPos__(1));
	orient2->SetElement(2,3,RefPos__(2));
	RefFrameAxes_->SetUserMatrix(orient2);
	qvtkWidget->GetRenderWindow()->Render();
	

	if(tip_list_->size()>1){
		DisplaySplineTip(*tip_list_,0,1,0);
		//DisplayPoints(*tip_list_,2,1,0,0,22,"S1");
		qvtkWidget->GetRenderWindow()->Render();
		#ifdef PLOT_REFSPLINE
			if(once_){
				once_=false;
				DisplaySplineRef(*RefSplineGenerated_,1,0,0);
				qvtkWidget->GetRenderWindow()->Render();
			}
		#endif
	}

#ifdef PLOT_PLANESTEER
	if (QString(CurvatureMean_.c_str()).toDouble()>0){
		DisplayPlaneS(c_,TipRot__,1.0,1.0,1.0);
		qvtkWidget->GetRenderWindow()->Render();
	}
#endif
	EuclideanDist_ = (floor(EuclideanDist_*10.0))/10.0;
	TextInfoError_->SetText( 0,("Inclination: " + inclination_ + " Curvature Mean: " + CurvatureMean_ + " --- Euclidean Error: " + (QString::number(EuclideanDist_)).toStdString()).c_str());
	//TextInfoError_->SetText( 0,("Curvature Y: " + CurvatureY_+ " Curvature Z: " + CurvatureZ_).c_str());
	TextInfoError_->SetText( 2,("Time elapsed: " + Time_elapsed_+" ms.").c_str());
	qvtkWidget->GetRenderWindow()->Render();
	//}
	/*
	if(showRefPlane_){
	DisplayPlaneR(ReferencePlaneParam_,CentroidRef_,P1,P2,RefPlaneMatrix_,1.0,1.0,0.0);
	ui.qvtkWidget->GetRenderWindow()->Render();
	showRefPlane_ = false;
	}
	*/
	//if(showSteeringPlane_){
	//DisplayPlaneS(SteeringPlaneParam_,centerS_,P3,P4,SteerPlaneMatrix_,0.0,1.0,0.0);
	//vtkSmartPointer<vtkMatrix4x4> orient2 = vtkSmartPointer<vtkMatrix4x4>::New();
	//orient2->Identity();
	//for(auto i=0;i<3;i++)
	//	for(auto j=0;j<3;j++)
	//		orient2->SetElement(i,j,SteerPlaneMatrix_(i,j));
	//orient2->SetElement(0,3,MeanCenterofSteering_(0));
	//orient2->SetElement(1,3,MeanCenterofSteering_(1));
	//orient2->SetElement(2,3,MeanCenterofSteering_(2));
	//SteeringPlaneAxes_->SetUserMatrix(orient2);
	//SteeringPlaneAxes_->SetAxisLabels(0);
	//qvtkWidget->GetRenderWindow()->Render();
	//}
	/*
	TextInfoError_->SetText( 0, ( "Position Error X: " + QString::number(PerrorX_) + " Y: " + QString::number(PerrorY_) + " Z: " + QString::number(PerrorZ_)).toStdString().c_str());
	TextInfoError_->SetText( 2,("RefY: " + QString::number(projYRef) + " RefZ: " + QString::number(projZRef)).toStdString().c_str());
	ui.qvtkWidget->GetRenderWindow()->Render();
	*/
}
void guiWindow::DisplayCentroid(Eigen::Vector4d &point,unsigned int R, unsigned int G, unsigned int B,unsigned int sizep){


	sensorNameMapper_->at(Num_sensors)->Update();
	sensorNameTextActor_->at(Num_sensors)->SetCamera(ren_->GetActiveCamera());
	sensorNameTextActor_->at(Num_sensors)->SetScale(1.0);
	sensorNameTextActor_->at(Num_sensors)->SetPosition(point(0), point(1), point(2));

	points_->at(Num_sensors)->Reset();
	points_->at(Num_sensors)->InsertNextPoint (point(0), point(1), point(2));

	// Set the points and vertices we created as the geometry and topology of the polydata
	PolydataPointsGlyph_->at(Num_sensors)->SetPoints(points_->at(Num_sensors));
	PolydataPointsGlyph_->at(Num_sensors)->Update();

	Glypfilter_->at(Num_sensors)->Update();
	PolydataPointsGlyph_->at(Num_sensors)->ShallowCopy(Glypfilter_->at(Num_sensors)->GetOutput());

	actorPoints_->at(Num_sensors)->GetProperty()->SetColor(R,G,B);
	actorPoints_->at(Num_sensors)->GetProperty()->SetPointSize(sizep);

}
void guiWindow::DisplayPoints(std::vector<Eigen::Vector3d> &tip_, unsigned int sensor_name, unsigned int R, unsigned int G, unsigned int B,unsigned int sizep, const std::string name){

	points_->at(sensor_name)->Reset();
	points_->at(sensor_name)->SetNumberOfPoints(tip_.size());
	for(auto i=0;i<tip_.size();i++)
		points_->at(sensor_name)->SetPoint(i,tip_.at(tip_.size()-1).x(),tip_.at(tip_.size()-1).y(), tip_.at(tip_.size()-1).z());

	PolydataPoints_->at(sensor_name)->SetPoints(points_->at(sensor_name));
	PolydataPoints_->at(sensor_name)->Update();

	Glypfilter_->at(sensor_name)->Update();
	PolydataPointsGlyph_->at(sensor_name)->ShallowCopy(Glypfilter_->at(sensor_name)->GetOutput());

	actorPoints_->at(sensor_name)->GetProperty()->SetColor(R,G,B);
	actorPoints_->at(sensor_name)->GetProperty()->SetPointSize(sizep);
}

void guiWindow::DisplaySplineTip(std::vector<Eigen::Vector3d> &point_, double R, double G, double B){
	vtkSmartPointer<vtkPoints> XYZpoints = vtkSmartPointer<vtkPoints>::New();
	if(point_.size()<60)
		for(unsigned int i=0;i<point_.size();i++)
			XYZpoints->InsertNextPoint(point_.at(i).x(),point_.at(i).y(),point_.at(i).z());
	else
		for(unsigned int i=point_.size()-60;i<point_.size();i++)
			XYZpoints->InsertNextPoint(point_.at(i).x(),point_.at(i).y(),point_.at(i).z());
	vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();
	spline->SetPoints(XYZpoints);

	//create parametric spline
	vtkSmartPointer<vtkParametricFunctionSource> functionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
	functionSource->SetParametricFunction(spline);
	functionSource->Update();

	vtkSmartPointer<vtkPolyDataMapper> splinemapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	splinemapper->SetInputConnection(functionSource->GetOutputPort());	
	  
	TrjActorTip_->SetMapper(splinemapper);
	TrjActorTip_->GetProperty()->SetColor(R,G,B);
	TrjActorTip_->GetProperty()->SetLineWidth(3.0);
	
}

void guiWindow::DisplaySplineRef(std::vector<Eigen::Vector3d> &point_, double R, double G, double B){
	vtkSmartPointer<vtkPoints> XYZpoints = vtkSmartPointer<vtkPoints>::New();
	//if(point_.size()<60)
		for(unsigned int i=0;i<point_.size();i++)
			XYZpoints->InsertNextPoint(point_.at(i).x(),point_.at(i).y(),point_.at(i).z());
	//else
		//for(unsigned int i=point_.size()-60;i<point_.size();i++)
			//XYZpoints->InsertNextPoint(point_.at(i).x(),point_.at(i).y(),point_.at(i).z());
	vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();
	spline->SetPoints(XYZpoints);

	//create parametric spline
	vtkSmartPointer<vtkParametricFunctionSource> functionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
	functionSource->SetParametricFunction(spline);
	functionSource->Update();

	vtkSmartPointer<vtkPolyDataMapper> splinemapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	splinemapper->SetInputConnection(functionSource->GetOutputPort());	
	
	TrjActorRef_->SetMapper(splinemapper);
	TrjActorRef_->GetProperty()->SetColor(R,G,B);
	TrjActorRef_->GetProperty()->SetLineWidth(3.0);
	
}


void guiWindow::DisplayPlaneS(Eigen::Vector4d &TipPos,Eigen::Matrix3d &TransfM,float R, float G, float B){
	Eigen::Vector3d tmp_ = Eigen::Vector3d::Zero();
	double meanC_=QString(CurvatureMean_.c_str()).toDouble();
	double radis_ = 1.0/meanC_;
	tmp_(0) = TipPos(0)+radis_*TransfM(0,1);
	tmp_(1) = TipPos(1)+radis_*TransfM(1,1);
	tmp_(2) = TipPos(2)+ radis_*TransfM(2,1);
	circleS_->SetRadius(radis_);
	circleS_->SetNormal(TransfM(0,2),TransfM(1,2),TransfM(2,2));
	circleS_->SetCenter(tmp_(0),tmp_(1),tmp_(2));
	//circleS_->SetCenter(CenterS__(0),CenterS__(1),CenterS__(2));
	circleS_->Update();
	circlemapper_->Update();
}
