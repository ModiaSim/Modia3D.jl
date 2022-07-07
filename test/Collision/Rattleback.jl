module RattlebackSimulation

using Modia3D

mass = 0.014660765716752368
centerOfMass = [0.0, 0.0, 0.0]
inertiaTensor = [5.864306286700948e-7 0.0 0.0; 0.0 7.623598172711232e-6 0.0; 0.0  0.0 7.623598172711232e-6]
gam = -5  # inertia tensor orientation angle [deg] about vertical axis; sign defines preferred rotation direction
rotMatrix = [cosd(gam) -sind(gam) 0.0; sind(gam) cosd(gam) 0.0; 0.0 0.0 1.0]
inertiaTensor = rotMatrix * inertiaTensor * rotMatrix'
massProps = MassProperties(; mass=mass, centerOfMass=centerOfMass,
                             Ixx=inertiaTensor[1,1], Iyy=inertiaTensor[2,2], Izz=inertiaTensor[3,3],
                             Ixy=inertiaTensor[1,2], Iyz=inertiaTensor[2,3], Ixz=inertiaTensor[3,1])

Rattleback = Model3D(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   nominalLength=0.1,
                                   enableContactDetection=true)),
    worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.05))),
    ground = Object3D(parent=:world,
                      translation=:[0.0, 0.0, -0.005],
                      feature=Solid(shape=Box(lengthX=0.2, lengthY=0.2, lengthZ=0.01),
                                    visualMaterial=VisualMaterial(color="Grey90", transparency=0.5),
                                    solidMaterial="DryWood",
                                    collision=true)),
    ellipsoid = Object3D(parent=:world, fixedToParent=false,
                         translation=[0.0, 0.0, 0.01],
                         angularVelocity=[0.0, 0.1, 5.0],
                         feature=Solid(massProperties=massProps,
                                       shape=Ellipsoid(lengthX=0.1, lengthY=0.02, lengthZ=0.02),
                                       visualMaterial=VisualMaterial(color="Blue"),
                                       solidMaterial="DryWood",
                                       collision=true))
)

rattleback = @instantiateModel(Rattleback, unitless=true)

stopTime = 6.0
tolerance = 1e-8
requiredFinalStates = [0.0006642916745069836, -0.0003421601373475422, 0.009999957179380127, 0.00048100224838247405, 0.0001387189555267131, -3.844627131472889e-8, 0.08140087449927354, -0.2616224376222781, -1.264936920220031, -0.014944019262829457, 0.05292038717100961, -2.294245213745764]
simulate!(rattleback, stopTime=stopTime, tolerance=tolerance, log=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(rattleback, ["ellipsoid.translation" "ellipsoid.rotation"; "ellipsoid.velocity" "ellipsoid.angularVelocity"], figure=1)

end
