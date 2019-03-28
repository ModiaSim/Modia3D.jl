module runexamples

import Modia3D

# Test Modia3D - Examples

# examples/visual

visualPath = joinpath(Modia3D.path, "examples", "visual")
include(joinpath(visualPath, "Move_AllVisualObjects.jl"))
include(joinpath(visualPath, "Move_SolidFileMesh.jl"))
include(joinpath(visualPath, "Visualize_AllVisualObjects.jl"))
include(joinpath(visualPath, "Visualize_Assembly.jl"))
include(joinpath(visualPath, "Visualize_GeometriesWithMaterial.jl"))
include(joinpath(visualPath, "Visualize_GeometriesWithoutMaterial.jl"))
include(joinpath(visualPath, "Visualize_SolidFileMesh.jl"))
include(joinpath(visualPath, "Visualize_Solids.jl"))
include(joinpath(visualPath, "Visualize_Text.jl"))
include(joinpath(visualPath, "Visualize_TextFonts.jl"))


# examples/dynamics
dynamicsPath = joinpath(Modia3D.path, "examples", "dynamics")
include(joinpath(dynamicsPath, "Simulate_DoublePendulum.jl"))
include(joinpath(dynamicsPath, "Simulate_DoublePendulumWithDampers.jl"))
include(joinpath(dynamicsPath, "Simulate_FallingBall1.jl"))
include(joinpath(dynamicsPath, "Simulate_FallingBall2.jl"))
include(joinpath(dynamicsPath, "Simulate_FallingBall3.jl"))
include(joinpath(dynamicsPath, "Simulate_FallingBall4.jl"))
include(joinpath(dynamicsPath, "Simulate_FallingBall4.jl"))
#include(joinpath(dynamicsPath, "Simulate_FallingBodies.jl"))
include(joinpath(dynamicsPath, "Simulate_Pendulum.jl"))
include(joinpath(dynamicsPath, "Simulate_Pendulum_withoutInertia.jl"))
include(joinpath(dynamicsPath, "Simulate_PendulumWithController.jl"))
include(joinpath(dynamicsPath, "Simulate_PendulumWithDamper.jl"))


# examples/kinematics
kinematicsPath = joinpath(Modia3D.path, "examples", "kinematics")
include(joinpath(kinematicsPath, "Move_DoublePendulum.jl"))
include(joinpath(kinematicsPath, "Move_FourBar.jl"))
include(joinpath(kinematicsPath, "Move_FourBar2.jl"))
include(joinpath(kinematicsPath, "Move_FourBar_Collision1.jl"))
include(joinpath(kinematicsPath, "Move_FourBar_Collision2.jl"))

# examples/signalToFlange
signalToFlangePath = joinpath(Modia3D.path, "examples", "SignalToFlange")
include(joinpath(signalToFlangePath, "Test_SignalAngle.jl"))
include(joinpath(signalToFlangePath, "Test_SignalTorque.jl"))

println("\n... success of runexamples.jl")
end
