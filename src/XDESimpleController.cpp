#include "xde-simple-controller/XDESimpleController.h"
#include <rtt/FlowStatus.hpp>
#include <rtt/TaskContext.hpp>
#include<orocos/ocl/DeploymentComponent.hpp>


XDE_SimpleController::XDE_SimpleController(const std::string& name)
    : TaskContext(name)
    , dynModel(NULL)
{
    this->addPort("q", in_q);
    this->addPort("qdot", in_qdot);
    this->addPort("d", in_d);
    this->addPort("t", in_t);

    this->addPort("tau", out_tau);

	this->addOperation("setDynModel", &XDE_SimpleController::setDynModelPointerStr, this, RTT::OwnThread);
	this->addOperation("loadPhy", &XDE_SimpleController::loadAgent, this, RTT::OwnThread);
}

XDE_SimpleController::~XDE_SimpleController()
{
    //if (dynModel != NULL) delete dynModel;
}

void XDE_SimpleController::loadAgent(std::string name){
	OCL::DeploymentComponent deploy;
	bool loaded = deploy.import(name);
	if(loaded == true){
		std::cout << "loaded" << loaded << std::endl;
	}
}

bool XDE_SimpleController::startHook(){
    return true;
}

void XDE_SimpleController::stopHook(){
    Eigen::VectorXd output = Eigen::VectorXd::Zero(dynModel->nbDofs());
    out_tau.write(output);
}

bool XDE_SimpleController::configureHook(){
    return true;
}

void XDE_SimpleController::updateHook(){
	RTT::FlowStatus flowStatus;

    flowStatus = in_q.read(q);
    if (flowStatus == RTT::NewData)
    {
        dynModel->setJointPositions(q);
    }

    flowStatus = in_qdot.read(qdot);
    if (flowStatus == RTT::NewData)
    {
        dynModel->setJointVelocities(qdot);
    }

	flowStatus = in_d.read(d);
	if (flowStatus == RTT::NewData)
	{
		dynModel->setFreeFlyerPosition(d);
	}

	flowStatus = in_t.read(t);
	if (flowStatus == RTT::NewData)
	{
		dynModel->setFreeFlyerVelocity(t);
	}

	Eigen::VectorXd tau = dynModel->getGravityTerms();
	Eigen::VectorXd tau2(7);
    for(int i=0; i<7; i++) tau2[i] = tau[i+6];

    out_tau.write(tau2);
}


/////////////////////////////////////////////////////////////////////////////////////////
// OPERATIONS
/////////////////////////////////////////////////////////////////////////////////////////

void XDE_SimpleController::setDynModelPointerStr(const std::string& strPtr)
{
    long long dyn_ptr = atoll(strPtr.c_str());
	std::cout << "string is: "<<strPtr<<"\n";
	std::cout <<"pointer is: "<<dyn_ptr<<"\n";
    dynModel = reinterpret_cast<xde::gvm::extra::DynamicModel*>(dyn_ptr);
	std::cout << dynModel->getJointPositions() << std::endl ;
}

