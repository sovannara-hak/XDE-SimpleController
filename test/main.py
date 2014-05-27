import xde_world_manager as xwm
import xde_resources as xr
import xde_robot_loader as xrl

import xdefw.interactive
shell = xdefw.interactive.shell()

import deploy.deployer as ddeployer

import xde_resources as xr

import xde_robot_loader as xrl
import xde_spacemouse as spacemouse
import physicshelper

TIME_STEP = 0.01

wm = xwm.WorldManager()
wm.createAllAgents(TIME_STEP)


groundWorld = xrl.createWorldFromUrdfFile(xr.ground, "ground", [0,0,0.0, 0.2, 0, 0, 0], True, 0.1, 0.05)
wm.addWorld(groundWorld)

sphereWorld = xrl.createWorldFromUrdfFile(xr.sphere, "sphere", [0,0.6,1.2, 1, 0, 0, 0], False, 0.2, 0.005)# , "material.concrete")
kukaWorld = xrl.createWorldFromUrdfFile(xr.kuka, "k1g", [0,0,0.1, 1, 0, 0, 0], False, 1, 0.005) #, "material.concrete")

wm.addMarkers(sphereWorld, ["sphere.sphere"], thin_markers=False)
wm.addWorld(sphereWorld)
wm.addWorld(kukaWorld)

#Controller
control = xdefw.rtt.Task(ddeployer.load("control", "XDE_SimpleController", "XDESimpleController-gnulinux", "", libdir="/tmp/XDE-SimpleController/_build/src/"))

import xde.desc.physic.physic_pb2
model = xde.desc.physic.physic_pb2.MultiBodyModel()
model.kinematic_tree.CopyFrom(kukaWorld.scene.physical_scene.nodes[0])
model.meshes.extend(kukaWorld.library.meshes)
model.mechanism.CopyFrom(kukaWorld.scene.physical_scene.mechanisms[0])
model.composites.extend(kukaWorld.scene.physical_scene.collision_scene.meshes)
dynmodel = physicshelper.createDynamicModel(model)
control.s.setDynModel(str(dynmodel.this.__long__()))

#create connectors to get robot k1g state 'k1g_q', 'k1g_qdot', 'k1g_Hroot', 'k1g_Troot', 'k1g_H'
robot_name = "k1g"
wm.phy.s.Connectors.OConnectorRobotState.new("ocpos"+robot_name, robot_name+"_", robot_name)
wm.phy.s.Connectors.IConnectorRobotJointTorque.new("ict"+robot_name, robot_name+"_", robot_name)

wm.phy.getPort(robot_name+"_q").connectTo(control.getPort("q"))
wm.phy.getPort(robot_name+"_qdot").connectTo(control.getPort("qdot"))
wm.phy.getPort(robot_name+"_Troot").connectTo(control.getPort("t"))
wm.phy.getPort(robot_name+"_Hroot").connectTo(control.getPort("d"))
control.getPort("tau").connectTo(wm.phy.getPort(robot_name+"_tau"))

# Normal mode
sm = spacemouse.createTask("smi", TIME_STEP, wm.phy, wm.graph, "sphere.sphere", pdc_enabled=False)
sm.s.start()

# Configure the robot
import lgsm
kuka = wm.phy.s.GVM.Robot("k1g")
kuka.enableGravity(True)
kuka.setJointPositions(lgsm.vector([0.4]*7))
kuka.setJointVelocities(lgsm.vector([0.0]*7))
kuka.enableContactWithBody("sphere.sphere", True)
kuka.enableContactWithBody("ground.ground", True)

control.s.setPeriod(TIME_STEP)
control.s.start()

wm.startAgents()
