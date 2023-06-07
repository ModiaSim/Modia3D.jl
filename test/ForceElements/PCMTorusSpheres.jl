module PCMTorusSpheres

using Modia3D

fileMeshTorus = joinpath(Modia3D.path, "objects/pcm", "torus.obj")
fileMeshSphere = joinpath(Modia3D.path, "objects/pcm", "sphere.obj")
visualMaterialBlue = VisualMaterial(color="Blue")
visualMaterialGreen = VisualMaterial(color="Green")
visualMaterialRed = VisualMaterial(color="Red")
pcmTorusSphere = Map(
    task=2,
    bodyE_YoungsModulus=100000.0,
    bodyE_PoissonsRatio=0.4,
    bodyE_LayerDepth=0.01,
    maxPenetration=0.1,
    compressionArealDampingFactor=10000.0,
    expansionArealDampingFactor=10000.0,
    expansionDampingMode=1,
    dampingTransitionDepth=0.001,
    frictionCoefficient=0.25,
    frictionRegularizationVelocity=0.0005)

TorusSpheres = Model3D(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   nominalLength=2.0)),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=0.2))),
    torus = Object3D(parent=:world,
                     feature=Solid(shape=FileMesh(filename=fileMeshTorus, useMaterialColor=true),
                                   massProperties=MassProperties(; mass=10.0, Ixx=0.3, Iyy=0.3, Izz=0.5),
                                   visualMaterial=visualMaterialBlue)),
    sphere1 = Object3D(parent=:world, fixedToParent=false,
                       translation=[0.0, 0.2, 1.2],
                       feature=Solid(shape=FileMesh(filename=fileMeshSphere, scale=[0.7, 0.7, 0.7], useMaterialColor=true),
                                     massProperties=MassProperties(; mass=10.0, Ixx=0.5, Iyy=0.5, Izz=0.5),
                                     visualMaterial=visualMaterialGreen)),
    sphere2 = Object3D(parent=:world, fixedToParent=false,
                       translation=[0.0, 0.0, 3.2],
                       feature=Solid(shape=FileMesh(filename=fileMeshSphere, scale=[0.75, 0.75, 0.75], useMaterialColor=true),
                                     massProperties=MassProperties(; mass=10.0, Ixx=0.5, Iyy=0.5, Izz=0.5),
                                     visualMaterial=visualMaterialRed)),
    forceTorusSphere1 = PolygonalContactModel(obj1=:torus, obj2=:sphere1) | pcmTorusSphere,
    forceTorusSphere2 = PolygonalContactModel(obj1=:torus, obj2=:sphere2) | pcmTorusSphere,
    forceSphere1Sphere2 = PolygonalContactModel(obj1=:sphere1, obj2=:sphere2, task=3) | pcmTorusSphere
)

pcmTorusSpheres = @instantiateModel(TorusSpheres, aliasReduction=false, unitless=true)

stopTime = 7.0
dtmax = 0.01
interval = 0.01
requiredFinalStates = missing
simulate!(pcmTorusSpheres, stopTime=stopTime, dtmax=dtmax, interval=interval, log=true, logTiming=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(pcmTorusSpheres, ["sphere1.translation" "sphere2.translation"
                       "sphere1.rotation"    "sphere2.rotation"   ], figure=1)
plot(pcmTorusSpheres, [("forceTorusSphere1.numContactElements", "forceSphere1Sphere2.numContactElements", "forceTorusSphere2.numContactElements")
                       ("forceTorusSphere1.maxPenetration", "forceSphere1Sphere2.maxPenetration", "forceTorusSphere2.maxPenetration")], figure=2)

end
