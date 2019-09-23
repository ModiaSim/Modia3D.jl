module TestExamples

import Modia3D

# Test Modia3D - Examples

collisionPath = joinpath(Modia3D.path, "examples", "collisions")
include(joinpath(collisionPath, "Simulate_Billiards_OneBall.jl"))
include(joinpath(collisionPath, "Simulate_BouncingBall.jl"))
include(joinpath(collisionPath, "Simulate_NewtonsCradle.jl"))
include(joinpath(collisionPath, "Simulate_SlidingAndRollingBall.jl"))
include(joinpath(collisionPath, "Simulate_TwoCollidingBalls.jl"))
include(joinpath(collisionPath, "Simulate_YouBot.jl"))


# test/collision

collisionPath = joinpath(Modia3D.path, "test", "collision")
include(joinpath(collisionPath, "Collision_3Elements.jl"))
# include(joinpath(collisionPath, "Collision_Bars.jl")) # is not working needs to be fixed
include(joinpath(collisionPath, "Plot_cor.jl"))
include(joinpath(collisionPath, "Plot_SlidingFriction.jl"))
include(joinpath(collisionPath, "Test_Collision.jl"))
include(joinpath(collisionPath, "Test_Collision_moreRevolutes.jl"))
include(joinpath(collisionPath, "Test_Collision_StarSetting.jl"))
include(joinpath(collisionPath, "Test_MiniBsp.jl"))
include(joinpath(collisionPath, "Test_Solids.jl"))


# test/collision/boxOnTable
collisionPath = joinpath(Modia3D.path, "test", "collision","boxOnTable")
# include(joinpath(collisionPath, "Simulate_BoxOnTable.jl"))  # is not working needs to be fixed
# include(joinpath(collisionPath, "Simulate_ContactBox3OnTable.jl")) # is not working needs to be fixed
include(joinpath(collisionPath, "Simulate_ContactBoxOnTable.jl"))
include(joinpath(collisionPath, "Simulate_YouBotBoxOnTable.jl"))

# test/collision/dynamic_collision
collisionPath = joinpath(Modia3D.path, "test", "collision","dynamic_collision")
include(joinpath(collisionPath, "collision_2_boxes.jl"))
include(joinpath(collisionPath, "collision_ballWithBall.jl"))
include(joinpath(collisionPath, "collision_ballWithBox.jl"))
include(joinpath(collisionPath, "collision_ballWithBox_45Deg.jl"))
include(joinpath(collisionPath, "collision_BallWithBox_Prismatic.jl"))
include(joinpath(collisionPath, "collision_ballWithBox_45Deg.jl"))
include(joinpath(collisionPath, "collision_newtons_cradle.jl"))

# test/collision/contactForceLaw
collisionPath = joinpath(Modia3D.path, "test", "collision","contactForceLaw")
include(joinpath(collisionPath, "bouncingBallComparedWithImpulsesjl.jl"))


println("\n ...test_Examples_Collision finished!")
end
