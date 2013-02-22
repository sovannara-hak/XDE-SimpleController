import xde_world_manager as xwm
import xde_resources as xr
import xde_robot_loader as xrl

import dsimi.interactive
shell = dsimi.interactive.shell()

import deploy.deployer as ddeployer

import xde_resources as xr

import xde_robot_loader as xrl

import physicshelper

TIME_STEP = 0.01

wm = xwm.WorldManager()
wm.createAllAgents(TIME_STEP)


import xde_spacemouse as spacemouse

sphereWorld = xrl.createWorldFromUrdfFile(xr.sphere, "sphere", [0,0.6,1.2, 1, 0, 0, 0], False, 0.2, 0.005)# , "material.concrete")
kukaWorld = xrl.createWorldFromUrdfFile(xr.kuka, "k1g", [0,0,0.4, 1, 0, 0, 0], True, 1, 0.005) #, "material.concrete")

wm.addMarkers(sphereWorld, ["sphere.sphere"], thin_markers=False)
wm.addWorld(sphereWorld, True)
wm.addWorld(kukaWorld, True)

#Controller
control = dsimi.rtt.Task(ddeployer.load("control", "XDE_SimpleController", "XDESimpleController-gnulinux", "", libdir="/home/shak/src/xde/XDE-SimpleController/_build/src/"))

model = physicshelper.createDynamicModel(kukaWorld, "k1g")
control.s.setDynModel(str(model.this.__long__()))

#create connectors to get robot k1g state 'k1g_q', 'k1g_qdot', 'k1g_Hroot', 'k1g_Troot', 'k1g_H'
robot_name = "k1g"
wm.phy.s.Connectors.OConnectorRobotState.new("ocpos"+robot_name, robot_name+"_", robot_name)
wm.phy.s.Connectors.IConnectorRobotJointTorque.new("ict"+robot_name, robot_name+"_", robot_name)

wm.phy.getPort(robot_name+"_q").connectTo(control.getPort("q"))
wm.phy.getPort(robot_name+"_qdot").connectTo(control.getPort("qdot"))
wm.phy.getPort(robot_name+"_Troot").connectTo(control.getPort("t"))
wm.phy.getPort(robot_name+"_Hroot").connectTo(control.getPort("d"))
control.getPort("tau").connectTo(wm.phy.getPort(robot_name+"_tau"))


# Configure the robot
import lgsm
kuka = wm.phy.s.GVM.Robot("k1g")
kuka.enableGravity(True)
kuka.setJointPositions(lgsm.vector([0.4]*7))
kuka.setJointVelocities(lgsm.vector([0.0]*7))

control.s.setPeriod(TIME_STEP)
control.s.start()

#PDC Control mode
#sm = spacemouse.createTask("smi", TIME_STEP, wm.phy, wm.graph, "sphere.sphere", pdc_enabled=True, body_name="k1g.07")

# Normal mode
#sm = spacemouse.createTask("smi", TIME_STEP, wm.phy, wm.graph, "sphere.sphere", pdc_enabled=False)

#sm.s.start()

wm.startSimulation()
