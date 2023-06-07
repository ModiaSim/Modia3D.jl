module PCMBouncingBubbles

using Modia3D

massPropertiesBubble = MassProperties(; mass=2.0, Ixx=0.02, Iyy=0.01, Izz=0.02)
fileMeshFunnel = joinpath(Modia3D.path, "objects/pcm", "funnel.obj")
fileMeshBubble = joinpath(Modia3D.path, "objects/pcm", "bubble.obj")
visualMaterialFunnel = VisualMaterial(color="Blue")
visualMaterialBubble1 = VisualMaterial(color="Yellow")
visualMaterialBubble2 = VisualMaterial(color="Red")
visualMaterialBubble3 = VisualMaterial(color="Green")
pcmBubbleFunnel = Map(
    task=2,
    maxPenetration=0.05,
    bodyE_YoungsModulus=5500.0,
    bodyE_PoissonsRatio=0.4,
    bodyE_LayerDepth=0.01,
    compressionArealDampingFactor=500.0,
    expansionArealDampingFactor=500.0,
    dampingTransitionMode=1,
    dampingTransitionDepth=0.001,
    frictionCoefficient=0.25,
    frictionRegularizationVelocity=0.0005)
pcmBubbleBubble = Map(
    task=1,
    maxPenetration=0.04,
    bodyE_YoungsModulus=5500.0,
    bodyF_YoungsModulus=5500.0,
    bodyE_PoissonsRatio=0.4,
    bodyF_PoissonsRatio=0.4,
    bodyE_LayerDepth=0.01,
    bodyF_LayerDepth=0.01,
    compressionArealDampingFactor=1000.0,
    expansionArealDampingFactor=1000.0,
    dampingTransitionMode=1,
    dampingTransitionDepth=0.001,
    frictionCoefficient=0.25,
    frictionRegularizationVelocity=0.0005)

BouncingBubbles = Model3D(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
#                                  enableVisualization=false,
#                                  animationFile="PCMBouncingBubbles.json",
                                   nominalLength=1.0)),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=0.2))),
    funnel = Object3D(parent=:world,
                      feature=Solid(shape=FileMesh(filename=fileMeshFunnel, useMaterialColor=true, doubleSided=true),
                                    massProperties=MassProperties(; mass=1.0, Ixx=0.1, Iyy=0.1, Izz=0.1),
                                    visualMaterial=visualMaterialFunnel)),
    bubble1 = Object3D(parent=:world, fixedToParent=false,
                       translation=[0.0, 0.0, 1.0],
                       feature=Solid(shape=FileMesh(filename=fileMeshBubble, useMaterialColor=true),
                                     massProperties=massPropertiesBubble,
                                     visualMaterial=visualMaterialBubble1)),
    bubble2 = Object3D(parent=:world, fixedToParent=false,
                       translation=[-0.3, 0.2, 1.0],
                       feature=Solid(shape=FileMesh(filename=fileMeshBubble, useMaterialColor=true),
                                     massProperties=massPropertiesBubble,
                                     visualMaterial=visualMaterialBubble2)),
    bubble3 = Object3D(parent=:world, fixedToParent=false,
                       translation=[0.3,-0.2, 1.0],
                       feature=Solid(shape=FileMesh(filename=fileMeshBubble, useMaterialColor=true),
                                     massProperties=massPropertiesBubble,
                                     visualMaterial=visualMaterialBubble3)),
    pcmBubble1Funnel = PolygonalContactModel(obj1=:bubble1, obj2=:funnel) | pcmBubbleFunnel,
    pcmBubble2Funnel = PolygonalContactModel(obj1=:bubble2, obj2=:funnel) | pcmBubbleFunnel,
    pcmBubble3Funnel = PolygonalContactModel(obj1=:bubble3, obj2=:funnel) | pcmBubbleFunnel,
    pcmBubble1Bubble2 = PolygonalContactModel(obj1=:bubble1, obj2=:bubble2) | pcmBubbleBubble,
    pcmBubble2Bubble3 = PolygonalContactModel(obj1=:bubble2, obj2=:bubble3) | pcmBubbleBubble,
    pcmBubble3Bubble1 = PolygonalContactModel(obj1=:bubble3, obj2=:bubble1) | pcmBubbleBubble
)

pcmBouncingBubbles = @instantiateModel(BouncingBubbles, aliasReduction=false, unitless=true)

stopTime = 3.0
dtmax = 0.01
interval = 0.002
requiredFinalStates = missing
simulate!(pcmBouncingBubbles, stopTime=stopTime, dtmax=dtmax, interval=interval, log=true, logTiming=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(pcmBouncingBubbles, ["bubble1.translation"     "bubble2.translation"     "bubble3.translation"
                          "bubble1.velocity"        "bubble2.velocity"        "bubble3.velocity"
                          "bubble1.rotation"        "bubble2.rotation"        "bubble3.rotation"
                          "bubble1.angularVelocity" "bubble2.angularVelocity" "bubble3.angularVelocity"], figure=1)

end
