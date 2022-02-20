import Test

Test.@testset "Basic" begin
    include(joinpath("Basic", "AllShapes.jl"))
    include(joinpath("Basic", "PendulumWithBar1.jl"))
    include(joinpath("Basic", "PendulumWithBar2.jl"))
    include(joinpath("Basic", "PendulumWithDamper.jl"))
    include(joinpath("Basic", "PendulumWithFix.jl"))
    include(joinpath("Basic", "PendulumWithParameterizedDamper.jl"))
    include(joinpath("Basic", "PendulumWithSpring.jl"))
    include(joinpath("Basic", "DoublePendulumWithDampers.jl"))
    include(joinpath("Basic", "BoxPlanarMotion.jl"))
    include(joinpath("Basic", "ShaftFreeMotion.jl"))
    include(joinpath("Basic", "ShaftFreeMotionAdaptiveRotSequence.jl"))
    Test.@test_throws LoadError include(joinpath("Basic", "Object3DWithoutParentError.jl"))  # test for too many objects without parent
    if testsExtend >= normalTests
        include(joinpath("Basic", "Mobile.jl"))
    end
    if testsExtend == completeTests
        include(joinpath("Basic", "PendulumWithDamper_Measurements.jl"))
        include(joinpath("Basic", "PendulumWithDamper_MonteCarlo.jl"))
    end
end

Test.@testset "Force Elements" begin
    include(joinpath("ForceElements", "HarmonicOscillator.jl"))
    include(joinpath("ForceElements", "BoxBushing.jl"))
    include(joinpath("ForceElements", "BoxSpringDamperPtP.jl"))
    include(joinpath("ForceElements", "BoxNonLinearSpringDamperPtP.jl"))
    if testsExtend == completeTests
        include(joinpath("ForceElements", "BoxBushing_Measurements.jl"))
    end
end

Test.@testset "Robot" begin
    include(joinpath("Robot", "ServoWithRamp.jl"))
    include(joinpath("Robot", "ServoWithRampAndPrismatic.jl"))
    include(joinpath("Robot", "ServoWithRampAndRevolute.jl"))
    include(joinpath("Robot", "ServoWithPathAndRevolute.jl"))
    if testsExtend >= normalTests
        include(joinpath("Robot", "YouBotWithSphere.jl"))
        include(joinpath("Robot", "YouBotGripping.jl"))
    end
    if testsExtend == completeTests
        include(joinpath("Robot", "YouBotPingPong.jl"))  # long computation time
        Test.@test_skip include(joinpath("Robot", "YouBotsGripping.jl"))  # long computation time  error on Linux: [CVODES ERROR]  CVode At t = 3.24097 and h = 3.25115e-08, the error test failed repeatedly or with |h| = hmin.
    end
end

Test.@testset "Collision" begin
    include(joinpath("Collision", "BouncingSphere.jl"))
    include(joinpath("Collision", "BouncingSphereFreeMotion.jl"))
    include(joinpath("Collision", "BouncingEllipsoid.jl"))
    include(joinpath("Collision", "BouncingEllipsoidOnSphere.jl"))
    include(joinpath("Collision", "TwoCollidingBalls.jl"))
    include(joinpath("Collision", "TwoCollidingBoxes.jl"))
    Test.@test_skip include(joinpath("Collision", "CollidingCylinders.jl"))  # windows: cylinder in the middle has a different behaviour
    include(joinpath("Collision", "NewtonsCradle.jl"))
    Test.@test_throws LoadError include(joinpath("Collision", "InValidCollisionPairingError.jl"))  # test for undefined collision pair material
    if testsExtend >= normalTests
        include(joinpath("Collision", "BouncingSphere2.jl"))  # use solver QBDF, Tsit5 with stopTime=2.5s; requiredFinalStates=[0.0, 0.0]
        include(joinpath("Collision", "ZeroCrossingIssue.jl"))
        include(joinpath("Collision", "Rattleback.jl"))
        include(joinpath("Collision", "BouncingCones.jl"))
        include(joinpath("Collision", "BouncingFrustums.jl"))
        include(joinpath("Collision", "BouncingCapsules.jl"))
        include(joinpath("Collision", "BouncingBeams.jl"))
        include(joinpath("Collision", "CollidingSphereWithBunnies.jl"))
        include(joinpath("Collision", "Billard4Balls.jl"))
    end
    if testsExtend == completeTests
        include(joinpath("Collision", "Billard16Balls.jl"))  # long computation time
    end
end

Test.@testset "Tutorial" begin
    include(joinpath("Tutorial", "Pendulum1.jl"))
    include(joinpath("Tutorial", "Pendulum2.jl"))
    include(joinpath("Tutorial", "Pendulum3.jl"))
    include(joinpath("Tutorial", "BouncingSphere.jl"))
end

Test.@testset "old" begin
    include(joinpath("old", "Move_Pendulum.jl"))
    if currentPlotPackage() == "PyPlot"
        include(joinpath("old", "Plot_cor.jl"))  # direct PyPlot calls
        include(joinpath("old", "Plot_SlidingFriction.jl"))  # direct PyPlot calls
    end
    include(joinpath("old", "test_Shapes.jl"))
    include(joinpath("old", "Test_PathPlanning.jl"))
    include(joinpath("old", "test_Solids.jl"))
    include(joinpath("old", "Visualize_Beam.jl"))
end
