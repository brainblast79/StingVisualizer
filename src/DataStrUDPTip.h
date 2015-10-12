#ifndef DATASTRUDPTIP_H
#define DATASTRUDPTIP_H
#pragma once

#include <Eigen/dense>
#include <boost\serialization\access.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/serialization/vector.hpp>
#include <iostream>
/// Structure to hold information about a single stock.
class TipData
{
public:
	TipData(){
		Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
		Eigen::Vector3d v = Eigen::Vector3d::Zero();
		TipData(m,v);
	};
	TipData(Eigen::Matrix3d &MRot,Eigen::Vector3d &Postip){
		newTipData(MRot,Postip);
		newRefData(MRot,Postip);
	}
	void newTipData(Eigen::Matrix3d &MRot,Eigen::Vector3d &Postip){
		int m=0;
		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++){
				MRot_[m] = MRot(j,i);
				m++;
			}
		for (int k=0;k<3;k++)
			Postip_[k] = Postip[k];
	};
	void newRefData(Eigen::Matrix3d &MRref,Eigen::Vector3d &Reftip){
		int m=0;
		for (int i=0;i<3;i++)
			for (int j=0;j<3;j++){
				Mref_[m] = MRref(j,i);
				m++;
			}
		for (int k=0;k<3;k++)
			Posref_[k] = Reftip[k];
	};
	void newCenterRot(Eigen::Vector4d &CenterS){
		for (int k=0;k<3;k++)
			centerS_[k] = CenterS(k);
	};
	void initFrame(Eigen::Matrix4d &initFr){
		int m=0;
		for (int i=0;i<4;i++)
			for (int j=0;j<4;j++){
				initFrame_[m] = initFr(j,i);
				m++;
			}
	};
	~TipData(){};
	double MRot_[9]; //riempi
	double Postip_[3];
	double Mref_[9];
	double Posref_[3];
	double centerS_[3];
	double initFrame_[16];
private:
    friend class boost::serialization::access;
	

  template <typename Archive>
  void serialize(Archive& ar, const unsigned int version)
  {
    ar & MRot_;
    ar & Postip_;
	ar & Mref_;
	ar & Posref_;
	ar & centerS_;
	ar & initFrame_;
  };
};

#endif