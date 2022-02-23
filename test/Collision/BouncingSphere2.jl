module BouncingSphere2

# Simulate bouncing sphere with CVODE_BDF, QBDF, Tsit5

using Modia3D

BouncingSphere = Model3D(
    boxHeigth = 0.1,
    groundMaterial = VisualMaterial(color="DarkGreen", transparency=0.5),
    gravField = UniformGravityField(g=9.81, n=[0, -1, 0]),
    world = Object3D(feature=Scene(gravityField=:gravField,
                                   visualizeFrames=false,
                                   defaultFrameLength=0.2,
                                   visualizeBoundingBox = true,
                                   enableContactDetection=true,
                                   visualizeContactPoints=false)),
    worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.5))),
    ground = Object3D(parent=:world,
                      translation=:[0.0,-boxHeigth/2-0.1,0.0],
                      feature=Solid(shape=Box(lengthX=4.0, lengthY=:boxHeigth, lengthZ=0.7),
                                    visualMaterial=:groundMaterial,
                                    solidMaterial="Steel",
                                    collision=true)),
    sphere = Object3D(feature=Solid(shape=Sphere(diameter=0.2),
                                    visualMaterial=VisualMaterial(color="Blue"),
                                    solidMaterial="Steel",
                                    massProperties=MassPropertiesFromShapeAndMass(mass=0.001),
                                    collision=true)),
    prism = Prismatic(obj1=:world, obj2=:sphere, axis=2, s=Var(init=1.0))
)

bouncingSphere = @instantiateModel(BouncingSphere, unitless=true, log=false, logStateSelection=false, logCode=false, FloatType = Float64)

#@show bouncingSphere.parameterExpressions
#@show bouncingSphere.parameters

stopTime = 2.5
dtmax = 0.01
tolerance = 1e-8
requiredFinalStates = [0.0, 0.0]
simulate!(bouncingSphere, stopTime=stopTime, tolerance=tolerance, dtmax=dtmax, log=true, logStates=false, logEvents=false,
          requiredFinalStates_atol = 1e-7, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingSphere, ["prism.s", "prism.v"], figure=1)

simulate!(bouncingSphere, QBDF(autodiff=false), stopTime=stopTime, tolerance=tolerance, dtmax=dtmax, log=true,
          requiredFinalStates_atol = 1e-7, requiredFinalStates=requiredFinalStates)
plot(bouncingSphere, ["prism.s", "prism.v"], figure=2)

simulate!(bouncingSphere, Tsit5(), stopTime=stopTime, tolerance=tolerance, dtmax=dtmax, log=true,
          merge = Map(world = Map(feature = Map(maximumContactDamping=1000))),   # Demonstrate how to change Modia3D data
          requiredFinalStates_atol = 1e-7, requiredFinalStates=requiredFinalStates)
plot(bouncingSphere, ["prism.s", "prism.v"], figure=3)


end
