module PCMTorusSphere

using Modia3D

fileMeshTorus = joinpath(Modia3D.path, "objects/pcm", "torus.obj")
fileMeshSphere = joinpath(Modia3D.path, "objects/pcm", "sphere.obj")
visualMaterialTorus = VisualMaterial(color="Blue")
visualMaterialSphere = VisualMaterial(color="Green")

TorusSphere = Model3D(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   enableContactDetection=false,
                                   nominalLength=2.0)),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=0.2))),
    torus = Object3D(parent=:world,
                     feature=Solid(shape=FileMesh(filename=fileMeshTorus, useMaterialColor=true),
                                   massProperties=MassProperties(; mass=10.0, Ixx=0.3, Iyy=0.3, Izz=0.5),
                                   visualMaterial=visualMaterialTorus)),
    sphere = Object3D(parent=:world, fixedToParent=false,
                      translation=[0.0, 0.1, 1.2],
                      feature=Solid(shape=FileMesh(filename=fileMeshSphere, useMaterialColor=true),
                                    massProperties=MassProperties(; mass=10.0, Ixx=0.5, Iyy=0.5, Izz=0.5),
                                    visualMaterial=visualMaterialSphere)),
    force = PolygonalContactModel(obj1=:torus, obj2=:sphere,
        task=2,
        bodyE_YoungsModulus=100000.0,
        bodyE_PoissonsRatio=0.4,
        bodyE_LayerDepth=0.01,
        maxPenetration=0.03,
        compressionArealDampingFactor=10000.0,
        expansionArealDampingFactor=10000.0,
        expansionDampingMode=1,
        dampingTransitionMode=1,
        dampingTransitionDepth=0.001,
        frictionCoefficient=0.3,
        frictionRegularizationVelocity=0.0005)
)

pcmTorusSphere = @instantiateModel(TorusSphere, aliasReduction=false, unitless=true)

stopTime = 6.0
dtmax = 0.01
interval = 0.01
requiredFinalStates = missing
simulate!(pcmTorusSphere, stopTime=stopTime, dtmax=dtmax, interval=interval, log=true, logTiming=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(pcmTorusSphere, ["sphere.translation", "sphere.rotation"], figure=1)

end
