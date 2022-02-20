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
    ellipsoid = Object3D(feature=Solid(massProperties=massProps,
                                       shape=Ellipsoid(lengthX=0.1, lengthY=0.02, lengthZ=0.02),
                                       visualMaterial=VisualMaterial(color="Blue"),
                                       solidMaterial="DryWood",
                                       collision=true)),
    joint = FreeMotion(obj1=:world, obj2=:ellipsoid, r=Var(init=ModiaBase.SVector{3,Float64}(0.0, 0.0, 0.01)), w=Var(init=ModiaBase.SVector{3,Float64}(0.0, 0.1, 5.0)))
)

rattleback = @instantiateModel(Rattleback, unitless=true)

stopTime = 6.0
tolerance = 1e-8
requiredFinalStates = [0.0006643219607533129, -0.0003421523453737833, 0.00999995717824536, 0.0004810494172644089, 0.00013865075493256469, -3.842333721570369e-8, 0.08135690941922864, -0.26163303787420544, -1.2651039117459748, -0.05518153954821725, -0.6191105987556029, -2.2091732972104654]
simulate!(rattleback, stopTime=stopTime, tolerance=tolerance, log=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(rattleback, ["joint.r" "joint.rot"; "joint.v" "joint.w"], figure=1)

end
