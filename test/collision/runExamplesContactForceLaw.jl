module runExamplesContactForceLaw

import Modia3D

# Test Modia3D - Examples



# examples/dynamics
collContactForceLawPath = joinpath(Modia3D.path, "test", "collision", "contactForceLaw")
include(joinpath(collContactForceLawPath, "contactForceLaw_2_boxes.jl"))
include(joinpath(collContactForceLawPath, "contactForceLaw_2_boxes_Prismatic.jl"))
include(joinpath(collContactForceLawPath, "contactForceLaw_2_boxes2.jl"))
include(joinpath(collContactForceLawPath, "contactForceLaw_Ball.jl"))
include(joinpath(collContactForceLawPath, "contactForceLaw_ballWithBall.jl"))
include(joinpath(collContactForceLawPath, "contactForceLaw_ballWithBox_45Deg.jl"))
include(joinpath(collContactForceLawPath, "contactForceLaw_ballWithBox_Prismatic.jl"))
include(joinpath(collContactForceLawPath, "contactForceLaw_Billard.jl"))
include(joinpath(collContactForceLawPath, "contactForceLaw_Billard_1Ball.jl"))
include(joinpath(collContactForceLawPath, "contactForceLaw_newtons_cradle.jl"))
include(joinpath(collContactForceLawPath, "contactForceLaw_rollingBall.jl"))
include(joinpath(collContactForceLawPath, "contactForceLaw_rollingBall_2balls.jl"))
include(joinpath(collContactForceLawPath, "contactForceLaw_rollingBall_2ballsb.jl"))


println("\n... success of runExamplesContactForceLaw.jl")
end
