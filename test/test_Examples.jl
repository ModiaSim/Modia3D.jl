module TestExamples

import Modia3D

# Test Modia3D - Examples

# test/visual
collisionPath = joinpath(Modia3D.path, "test", "collision")
include(joinpath(collisionPath, "Collision_3Elements.jl"))
# include(joinpath(collisionPath, "Collision_Bars.jl")) # is not working needs to be fixed
include(joinpath(collisionPath, "Test_Collision.jl"))
include(joinpath(collisionPath, "Test_Collision_moreRevolutes.jl"))
include(joinpath(collisionPath, "Test_Collision_StarSetting.jl"))
include(joinpath(collisionPath, "Test_MiniBsp.jl"))
include(joinpath(collisionPath, "Test_Solids.jl"))


# test/SignalToFlange
dynamicsPath = joinpath(Modia3D.path, "test", "dynamics")
include(joinpath(dynamicsPath, "Simulate_ControllerDamper.jl"))
include(joinpath(dynamicsPath, "Simulate_DamperMacro.jl"))
include(joinpath(dynamicsPath, "Simulate_FourBar.jl"))

# test/revoluteJoints
testJointsPath = joinpath(Modia3D.path, "test", "joints")
include(joinpath(testJointsPath, "Test_KinematicRevoluteJoints.jl"))
include(joinpath(testJointsPath, "Test_Dynamic_Pendulum_xAxis.jl"))
include(joinpath(testJointsPath, "Test_Dynamic_Pendulum_yAxis.jl"))
include(joinpath(testJointsPath, "Test_Dynamic_Pendulum_zAxis.jl"))
include(joinpath(testJointsPath, "Test_Prismatic_xAxis.jl"))
include(joinpath(testJointsPath, "Test_Prismatic_yAxis.jl"))
include(joinpath(testJointsPath, "Test_Prismatic_zAxis.jl"))
include(joinpath(testJointsPath, "Move_FourBar_xAxis.jl"))
include(joinpath(testJointsPath, "Move_FourBar_yAxis.jl"))
include(joinpath(testJointsPath, "Move_FourBar_zAxis.jl"))

# test/kinematics
kinematicsPath = joinpath(Modia3D.path, "test", "kinematics")
include(joinpath(kinematicsPath, "Move_FourBar_noMacros.jl"))


# test/signalToFlange
signalToFlangePath = joinpath(Modia3D.path, "test", "signalToFlange")
include(joinpath(signalToFlangePath, "Test_Signal1Assembly.jl"))
include(joinpath(signalToFlangePath, "Test_Signal4Assemblies.jl"))

# test/signalToFlange
volCompPath = joinpath(Modia3D.path, "test", "volumeComputation")
include(joinpath(volCompPath, "volume_computation3D_obj.jl"))

# test/withoutAssembly/kinematics
wAKinematicsPath = joinpath(Modia3D.path, "test", "withoutAssembly","kinematics")
include(joinpath(wAKinematicsPath, "Move_Pendulum.jl"))

# examples/withoutAssembly/visual
wAVisualPath = joinpath(Modia3D.path, "test", "withoutAssembly","visual")
include(joinpath(wAVisualPath, "Visualize_Beam.jl"))



println("\n test_Examples finished!")
end
