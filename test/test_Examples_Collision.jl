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
# include(joinpath(collisionPath, "Plot_cor.jl"))             # not included since PyPlot required
#include(joinpath(collisionPath, "Plot_SlidingFriction.jl"))  # not included since PyPlot required
include(joinpath(collisionPath, "Test_Collision.jl"))
include(joinpath(collisionPath, "Test_Collision_moreRevolutes.jl"))
include(joinpath(collisionPath, "Test_Collision_StarSetting.jl"))
include(joinpath(collisionPath, "Test_MiniBsp.jl"))
include(joinpath(collisionPath, "Test_Solids.jl"))


# test/collision/boxOnTable
collisionPath = joinpath(Modia3D.path, "test", "collision","boxOnTable")
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
# include(joinpath(collisionPath, "bouncingBallComparedWithImpulses.jl"))  # not included since PyPlot required
include(joinpath(collisionPath, "contactForceLaw_2_boxes.jl"))
include(joinpath(collisionPath, "contactForceLaw_2_boxes_Prismatic.jl"))
include(joinpath(collisionPath, "contactForceLaw_2_boxes2.jl"))
include(joinpath(collisionPath, "contactForceLaw_Ball.jl"))
include(joinpath(collisionPath, "contactForceLaw_ballWithBall.jl"))
include(joinpath(collisionPath, "contactForceLaw_ballWithBox_45Deg.jl"))
include(joinpath(collisionPath, "contactForceLaw_ballWithBox_Prismatic.jl"))
include(joinpath(collisionPath, "contactForceLaw_newtons_cradle.jl"))


# test/collision/Billard_Simulation
collisionPath = joinpath(Modia3D.path, "test", "collision","Billard_Simulations", "BallCushion")
include(joinpath(collisionPath, "BillardBall1_Cushion1_directHit.jl"))
include(joinpath(collisionPath, "BillardBall1_Cushion4_arbitraryHit.jl"))
# include(joinpath(collisionPath, "BillardBall1_Cushion4_directHit.jl"))  # Successful on Windows and Linux, buf fails on Mac with
                                                                          # mxstep steps taken before reaching tout.
                                                                          # Try to set maxNumberOfSteps to a value > 500


# collisionPath = joinpath(Modia3D.path, "test", "collision","Billard_Simulations", "rollingBall")
# include(joinpath(collisionPath, "colliding2rollingBalls_plot1.jl"))   # not included since PyPlot required
# include(joinpath(collisionPath, "colliding2rollingBalls_plot2.jl"))   # not included since PyPlot required
# include(joinpath(collisionPath, "rollingBall1.jl"))                   # not included since PyPlot required


collisionPath = joinpath(Modia3D.path, "test", "collision","Billard_Simulations", "billard")
# include(joinpath(collisionPath, "Billard_Balls4.jl")) # takes not too long, but still to long for an example test
# include(joinpath(collisionPath, "Billard_Balls4_Prismatic.jl")) # takes not too long, but still to long for an example test
# include(joinpath(collisionPath, "Billard_Balls16.jl")) # takes too long

println("\n ...test_Examples_Collision finished!")
end
