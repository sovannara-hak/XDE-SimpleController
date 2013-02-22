#ifndef __XDE_SIMPLE_CONTROLLER__H__
#define __XDE_SIMPLE_CONTROLLER__H__

#include<Eigen/Core>
#include<Eigen/Lgsm>


#include <rtt/os/main.h>
#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>

#include <rtt/Property.hpp>
#include <rtt/Port.hpp>

#include <xdecore/gvm.h>
#include <xdecore/gvm/DynamicModel.h>

class XDE_SimpleController: public RTT::TaskContext{

	public:
		XDE_SimpleController(const std::string& name);
		~XDE_SimpleController();

		bool startHook();
		void stopHook();
		bool configureHook();
		void updateHook();

		void setDynModelPointerStr(const std::string& strPtr);

		void loadAgent(std::string name);

	private:
		RTT::InputPort< Eigen::VectorXd > in_q;
		RTT::InputPort< Eigen::VectorXd > in_qdot;
		RTT::InputPort< Eigen::Displacementd > in_d;
		RTT::InputPort< Eigen::Twistd > in_t;

		RTT::OutputPort< Eigen::VectorXd > out_tau;

		Eigen::VectorXd q;
		Eigen::VectorXd qdot;
		Eigen::Displacementd d;
		Eigen::Twistd t;

		xde::gvm::extra::DynamicModel* dynModel;
};

#include <rtt/Component.hpp>
ORO_CREATE_COMPONENT( XDE_SimpleController );

#endif


