module BoxForceSensorResultSimulation

using Modia3D

BoxForceSensorResult = Model3D(
    Length = 0.1,
    Mass = 1.0,
    IMoment = 0.1,
    visualMaterial = VisualMaterial(color="IndianRed1", transparency=0.5),
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   nominalLength=:(2*Length))),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:Length))),
    coordRefFrame = Object3D(parent=:world,
                             rotation=[deg2rad(23), deg2rad(45), deg2rad(67)],
                             feature=Visual(shape=CoordinateSystem(length=:(Length/2)))),
    box1 = Object3D(parent=:world, fixedToParent=false,
                    translation=[0.2, 0.1, 0.05],
                    rotation=[0.12, 0.06, 0.03],
                    feature=Solid(shape=Box(lengthX=:Length, lengthY=:Length, lengthZ=:Length),
                                  massProperties=MassProperties(; mass=:Mass, Ixx=:IMoment, Iyy=:IMoment, Izz=:IMoment),
                                  visualMaterial=VisualMaterial(color="IndianRed1", transparency=0.5))),
    box2 = Object3D(parent=:world, fixedToParent=false,
                    feature=Solid(shape=Box(lengthX=:Length, lengthY=:Length, lengthZ=:Length),
                                  massProperties=MassProperties(; mass=:Mass, Ixx=:IMoment, Iyy=:IMoment, Izz=:IMoment),
                                  visualMaterial=VisualMaterial(color="Turquoise", transparency=0.5))),
    box2CornerFrame = Object3D(parent=:box2,
                               feature=Visual(shape=CoordinateSystem(length=:(Length/2))),
                               translation=:[Length/2, Length/2, Length/2]),
    force1 = Bushing(obj1=:world, obj2=:box1,
                     springForceLaw=[50.0, 100.0, 200.0], damperForceLaw=[1.0, 2.0, 4.0],
                     rotSpringForceLaw=[5.0, 10.0, 20.0], rotDamperForceLaw=[0.1, 0.2, 0.4], largeAngles=false),
    force2 = SpringDamperPtP(obj1=:world, obj2=:box2CornerFrame, springForceLaw=100.0, damperForceLaw=2.0),
    sensor = SensorResult(object=:box1, objectOrigin=:box2, objectCoordinateRef=:coordRefFrame, objectObserveRef=:box1)
)

boxForceSensorResult = @instantiateModel(BoxForceSensorResult, unitless=true)

stopTime = 6.0
dtmax = 0.1
requiredFinalStates = [-0.0016107113217414199, -0.0002480705395490782, -0.049050349752476706, 0.07029162030881492, 2.2312117805489287e-5, -6.372913904848077e-6, -0.0010449353993963431, -0.0001514824927969645, -2.4593890216411865e-7, 0.042388263052657665, 0.00010086233694909848, -5.551141011115388e-6, 0.05646126021462138, 0.0564613884676648, -0.12716607892200024, 0.2920400985011993, 0.2920398970084984, 0.04549365682408229, 2.1980798016988854, -0.6805794882560942, 1.213379900814701, 0.29439337230022455, -0.29439394783827416, 5.036863759629742e-7]
simulate!(boxForceSensorResult, stopTime=stopTime, dtmax=dtmax, log=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(boxForceSensorResult, ["sensor.translation", "sensor.velocity", "sensor.acceleration"], figure=1)
plot(boxForceSensorResult, ["sensor.rotation", "sensor.angularVelocity", "sensor.angularAcceleration"], figure=2)
plot(boxForceSensorResult, ["sensor.distance", "sensor.distanceVelocity", "sensor.distanceAcceleration"], figure=3)

end
