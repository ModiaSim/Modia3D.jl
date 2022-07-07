module BoxPlanarMotionSimulation

using Modia3D

BoxPlanarMotion = Model3D(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=1.0, n=[0, -1, 0]))),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=0.2))),
    transx = Object3D(),
    transy = Object3D(),
    box = Object3D(feature=Solid(shape=Box(lengthX=0.4, lengthY=0.6, lengthZ=0.1),
                                 solidMaterial="Steel",
                                 visualMaterial=VisualMaterial(color="DarkGreen", transparency=0.2))),
    prismatic_x = Prismatic(obj1=:world , obj2=:transx, axis=1, s  =Var(init=-1.0), v=Var(init=1.0)),
    prismatic_y = Prismatic(obj1=:transx, obj2=:transy, axis=2, s  =Var(init=-1.0), v=Var(init=2.0)),
    revolute_z  = Revolute( obj1=:transy, obj2=:box   , axis=3, phi=Var(init= 0.0), w=Var(init=3.0))
)

model = @instantiateModel(BoxPlanarMotion, unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 4.0
requiredFinalStates = [1.0, 3.0, -2.0, -1.0, 3.0, 12.0]
simulate!(model, stopTime=stopTime, log=true, logStates=true, requiredFinalStates=requiredFinalStates)

@usingPlotPackage
plot(model, ["prismatic_x.s", "prismatic_y.s", "revolute_z.phi"], figure=1)

end
