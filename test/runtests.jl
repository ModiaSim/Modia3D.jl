module Runtests

import Test
using  ModiaLang

@time Test.@testset verbose=true "Modia3D" begin
    usePlotPackage("SilentNoPlot")

    Test.@testset "Basic" begin
        include(joinpath("Basic", "AllShapes.jl"))
        include(joinpath("Basic", "PendulumWithBar1.jl"))
        include(joinpath("Basic", "PendulumWithBar2.jl"))
        include(joinpath("Basic", "PendulumWithDamper.jl"))
        include(joinpath("Basic", "PendulumWithFix.jl"))
        include(joinpath("Basic", "PendulumWithParameterizedDamper.jl"))
        include(joinpath("Basic", "PendulumWithSpring.jl"))
        include(joinpath("Basic", "DoublePendulumWithDampers.jl"))
        include(joinpath("Basic", "Mobile.jl"))
        include(joinpath("Basic", "ShaftFreeMotion.jl"))
        Test.@test_throws LoadError include(joinpath("Basic", "Object3DWithoutParentError.jl"))
    end

    Test.@testset "Robot" begin
        include(joinpath("Robot", "ServoWithRamp.jl"))
        include(joinpath("Robot", "ServoWithRampAndPrismatic.jl"))
        include(joinpath("Robot", "ServoWithRampAndRevolute.jl"))
        include(joinpath("Robot", "ServoWithPathAndRevolute.jl"))
        include(joinpath("Robot", "YouBotWithSphere.jl"))
        Test.@test_skip include(joinpath("Robot", "YouBotPingPong.jl")) # too long computation time
        include(joinpath("Robot", "YouBotGripping.jl"))
        Test.@test_skip include(joinpath("Robot", "YouBotsGripping.jl")) # too long computation time
    end

    Test.@testset "Collision" begin
        include(joinpath("Collision", "BouncingSphere.jl"))
        include(joinpath("Collision", "BouncingSphereFreeMotion.jl"))
        include(joinpath("Collision", "BouncingEllipsoid.jl"))
        include(joinpath("Collision", "BouncingCones.jl"))
        include(joinpath("Collision", "BouncingCapsules.jl"))
        include(joinpath("Collision", "BouncingBeams.jl"))
        include(joinpath("Collision", "TwoCollidingBalls.jl"))
        include(joinpath("Collision", "TwoCollidingBoxes.jl"))
        include(joinpath("Collision", "CollidingCylinders.jl"))
        include(joinpath("Collision", "CollidingSphereWithBunnies.jl"))
        include(joinpath("Collision", "NewtonsCradle.jl"))
        include(joinpath("Collision", "Billard4Balls.jl"))
        Test.@test_skip include(joinpath("Collision", "Billard16Balls.jl"))  # too long computation time
    end

    Test.@testset "Tutorial" begin
        include(joinpath("Tutorial", "Pendulum1.jl"))
        include(joinpath("Tutorial", "Pendulum2.jl"))
        include(joinpath("Tutorial", "Pendulum3.jl"))
        include(joinpath("Tutorial", "BouncingSphere.jl"))
    end

    Test.@testset "old" begin
        include(joinpath("old", "Move_Pendulum.jl"))
        include(joinpath("old", "Plot_cor.jl"))
        include(joinpath("old", "Plot_SlidingFriction.jl"))
        include(joinpath("old", "test_Shapes.jl"))
        include(joinpath("old", "Test_PathPlanning.jl"))
        include(joinpath("old", "test_Solids.jl"))
        include(joinpath("old", "Visualize_Beam.jl"))
    end

    usePreviousPlotPackage()
end

end
