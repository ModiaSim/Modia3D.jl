module BouncingSphereContactResultsSimulation

using Modia3D

BouncingSphere = Model3D(
    boxHeigth = 0.1,
    groundMaterial = VisualMaterial(color="DarkGreen", transparency=0.5),
    gravField = UniformGravityField(g=9.81, n=[0, -1, 0]),
    world = Object3D(feature=Scene(gravityField=:gravField,
                                   visualizeFrames=false,
                                   defaultFrameLength=0.2,
                                   enableContactDetection=true,
                                   visualizeContactPoints=true)),
    worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.5))),
    ground = Object3D(parent=:world,
                      translation=:[0.0, -boxHeigth/2, 0.0],
                      feature=Solid(shape=Box(lengthX=2.0, lengthY=:boxHeigth, lengthZ=0.5),
                                    visualMaterial=:groundMaterial,
                                    solidMaterial="Steel",
                                    collision=true)),
    sphere = Object3D(parent=:world, fixedToParent=false,
                      translation=[-0.5, 0.3, 0.0],
                      velocity=[1.0, 0.0, 0.0],
                      feature=Solid(shape=Sphere(diameter=0.2),
                                    visualMaterial=VisualMaterial(color="Blue"),
                                    solidMaterial="Steel",
                                    massProperties=MassPropertiesFromShapeAndMass(mass=10000.0),
                                    collision=true)),
    result = ContactResult(object1=:sphere, object2=:ground, objectCoordinateRef=:ground)
)

bouncingSphere = @instantiateModel(BouncingSphere, unitless=true, log=false, logStateSelection=false, logCode=false, FloatType=Float64)

stopTime = 1.5
dtmax = 0.05
tolerance = 1e-8
requiredFinalStates = [0.6190964322303429, 0.09989803187140613, 0.0, 0.7018335184646917, -0.00011551585213889106, 0.0, 0.0, 0.0, -9.171436344350457, 0.0, 0.0, -7.0240720401621495]
simulate!(bouncingSphere, stopTime=stopTime, tolerance=tolerance, dtmax=dtmax, log=true, logStates=false, logEvents=true,
          requiredFinalStates_atol=1e-6, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingSphere, ["result.penetration", "result.penetrationVelocity", "result.tangentialVelocity", "result.angularVelocity", "result.normalForce", "result.tangentialForce", "result.torque"], figure=1)
plot(bouncingSphere, ["result.positionVector", "result.normalVector", "result.forceVector", "result.torqueVector"], figure=2)

end
