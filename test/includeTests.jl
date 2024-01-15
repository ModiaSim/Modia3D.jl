import Test

Modia3D.loadPalettes!(solidMaterialPalette       = "$(Modia3D.path)/palettes/solidMaterials.json",
                      contactPairMaterialPalette = "$(Modia3D.path)/palettes/contactPairMaterials.json",
                      visualMaterialPalette      = "$(Modia3D.path)/palettes/visualMaterials.json")

Test.@testset "Frames" begin
    include("TestFrames.jl")
end

Test.@testset "Basic" begin
    include(joinpath("Basic", "Pendulum.jl"))
    include(joinpath("Basic", "ModelsForPrecompilation.jl"))
    include(joinpath("Basic", "AllShapes.jl"))
    include(joinpath("Basic", "PendulumWithBar1.jl"))
    include(joinpath("Basic", "PendulumWithBar2.jl"))
    include(joinpath("Basic", "PendulumWithBar3.jl"))
    include(joinpath("Basic", "PendulumWithDamper.jl"))
    include(joinpath("Basic", "PendulumWithFix.jl"))
    include(joinpath("Basic", "PendulumWithParameterizedDamper.jl"))
    include(joinpath("Basic", "PendulumWithSpring.jl"))
    include(joinpath("Basic", "DoublePendulumWithDampers.jl"))
    include(joinpath("Basic", "BoxPlanarMotion.jl"))
    include(joinpath("Basic", "FreeShaft.jl"))
    include(joinpath("Basic", "FreeShaftAdaptiveRotSequence.jl"))
    include(joinpath("Basic", "FreeShaftAdaptiveRotSequenceWithFreeMotion.jl"))
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
    include(joinpath("ForceElements", "BoxWorldForce.jl"))
    include(joinpath("ForceElements", "BoxWorldTorque.jl"))
    include(joinpath("ForceElements", "HarmonicOscillator.jl"))
    include(joinpath("ForceElements", "BoxBushing.jl"))
    include(joinpath("ForceElements", "BoxSpringDamperPtP.jl"))
    include(joinpath("ForceElements", "BoxNonLinearSpringDamperPtP.jl"))
    if testsExtend >= normalTests
        include(joinpath("ForceElements", "PCMBubbleFunnel.jl"))
    end
    if testsExtend == completeTests
        include(joinpath("ForceElements", "PCMTorusSphere.jl"))
        include(joinpath("ForceElements", "PCMTorusSpheres.jl"))
        include(joinpath("ForceElements", "PCMBouncingBubbles.jl"))
        include(joinpath("ForceElements", "BoxBushing_Measurements.jl"))
    end
end

Test.@testset "Robot" begin
    include(joinpath("Robot", "ServoWithRamp.jl"))
    include(joinpath("Robot", "ServoWithRampAndPrismatic.jl"))
    include(joinpath("Robot", "ServoWithRampAndRevolute.jl"))
    include(joinpath("Robot", "ServoWithPathAndRevolute.jl"))
    if testsExtend >= normalTests
        Test.@test_skip include(joinpath("Robot", "YouBotWithSphere.jl"))  # LinearAlgebra.SingularException on some platforms
        include(joinpath("Robot", "YouBotGripping.jl"))
        include(joinpath("Robot", "YouBotSphereTransport.jl"))
    end
    if testsExtend == completeTests
        include(joinpath("Robot", "ScenarioCollisionOnly.jl"))  # long computation time
        include(joinpath("Robot", "YouBotPingPong.jl"))  # long computation time
        include(joinpath("Robot", "YouBotsGripping.jl"))  # long computation time
    end
end

Test.@testset "Collision" begin
    include(joinpath("Collision", "BouncingSphere.jl"))
    include(joinpath("Collision", "BouncingSphereFree.jl"))
    include(joinpath("Collision", "BouncingSphereContactResults.jl"))
    include(joinpath("Collision", "BouncingEllipsoid.jl"))
    include(joinpath("Collision", "BouncingEllipsoidOnSphere.jl"))
    include(joinpath("Collision", "TwoCollidingBalls.jl"))
    include(joinpath("Collision", "TwoCollidingBoxes.jl"))
    include(joinpath("Collision", "CollidingCylinders.jl"))
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
        Test.@test_throws LoadError include(joinpath("Collision", "OrthogonalLinesError.jl"))  # orthogonal lines moving until r_abs is NaN
        Test.@test_throws LoadError include(joinpath("Collision", "ParallelLinesError.jl"))  # MPR 2D not implemented
        Test.@test_throws LoadError include(joinpath("Collision", "PlaneVsPlaneEdgesError.jl"))  # MPR 2D not implemented
        Test.@test_throws LoadError include(joinpath("Collision", "PlaneVsPointError.jl"))  # MPR 2D not implemented
    end
    if testsExtend == completeTests
        include(joinpath("Collision", "Billard16Balls.jl"))  # long computation time
    end
end

Test.@testset "Segmented" begin
    include(joinpath("Segmented", "TwoStageRocket3D.jl"))
    if testsExtend >= normalTests
        include(joinpath("Segmented", "YouBotDynamicState.jl"))
        include(joinpath("Segmented", "YouBotFixBox.jl"))
        include(joinpath("Segmented", "YouBotFixSphere.jl"))
        include(joinpath("Segmented", "ScenarioSegmentedCollisionOff.jl"))
        include(joinpath("Segmented", "ScenarioSegmentedCollisionOn.jl"))
        include(joinpath("Segmented", "ScenarioSegmentedOnly.jl"))
    end
end

Test.@testset "Tutorial" begin
    include(joinpath("Tutorial", "Pendulum1.jl"))
    include(joinpath("Tutorial", "Pendulum2.jl"))
    include(joinpath("Tutorial", "Pendulum3.jl"))
    include(joinpath("Tutorial", "BouncingSphere.jl"))
    include(joinpath("Tutorial", "TwoStageRocket3D.jl"))
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
