module BouncingConesSimulation

using Modia3D

vmatRed   = VisualMaterial(color="Red")
vmatGreen = VisualMaterial(color="Green")
vmatBlue  = VisualMaterial(color="Blue")
vmatGrey  = VisualMaterial(color="Grey", transparency=0.5)

BouncingCones = Model3D(
    boxHeigth = 0.1,
    gravField = UniformGravityField(g=9.81, n=[0, 0, -1]),
    world = Object3D(feature=Scene(gravityField=:gravField,
                                   visualizeFrames=false,
                                   defaultFrameLength=0.2,
                                   visualizeBoundingBox = true,
                                   enableContactDetection=true,
                                   visualizeContactPoints=false,
                                   animationFile="BouncingCones.json")),
    worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.5))),
    ground = Object3D(parent=:world,
                      translation=:[0.0, 0.0, -boxHeigth/2],
                      feature=Solid(shape=Box(lengthX=5.0, lengthY=3.0, lengthZ=:boxHeigth),
                                    visualMaterial=vmatGrey,
                                    solidMaterial="DryWood",
                                    collision=true)),
    frameX = Object3D(parent=:world,
                      translation=:[-1.0, -0.5, 1.0],
                      rotation=:[-90*u"°", 0.0, -90*u"°"],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    coneX = Object3D(feature=Solid(shape=Cone(axis=1, diameter=0.4, length=1.0),
                                   visualMaterial=vmatRed,
                                   solidMaterial="DryWood",
                                   collision=true)),
    jointX = FreeMotion(obj1=:frameX, obj2=:coneX,
                        r=Var(init=ModiaBase.SVector{3,Float64}(0.0, 0.0, 0.0)),
                        rot=Var(init=ModiaBase.SVector{3,Float64}(0.0, -60*u"°", 0.0)),
                        v=Var(init=ModiaBase.SVector{3,Float64}(0.0, 1.0, 0.0))),
    frameY = Object3D(parent=:world,
                      translation=:[0.0, -0.5, 1.0],
                      rotation=:[90*u"°", 90*u"°", 0.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    coneY = Object3D(feature=Solid(shape=Cone(axis=2, diameter=0.4, length=1.0),
                                   visualMaterial=vmatGreen,
                                   solidMaterial="DryWood",
                                   collision=true)),
    jointY = FreeMotion(obj1=:frameY, obj2=:coneY,
                        r=Var(init=ModiaBase.SVector{3,Float64}(0.0, 0.0, 0.0)),
                        rot=Var(init=ModiaBase.SVector{3,Float64}(0.0, 0.0, -60*u"°")),
                        v=Var(init=ModiaBase.SVector{3,Float64}(0.0, 0.0, 1.0))),
    frameZ = Object3D(parent=:world,
                      translation=:[1.0, -0.5, 1.0],
                      feature=Visual(shape=CoordinateSystem(length=0.5))),
    coneZ = Object3D(feature=Solid(shape=Cone(axis=3, diameter=0.4, length=1.0),
                                   visualMaterial=vmatBlue,
                                   solidMaterial="DryWood",
                                   collision=true)),
    jointZ = FreeMotion(obj1=:frameZ, obj2=:coneZ,
                        r=Var(init=ModiaBase.SVector{3,Float64}(0.0, 0.0, 0.0)),
                        rot=Var(init=ModiaBase.SVector{3,Float64}(-60*u"°", 0.0, 0.0)),
                        v=Var(init=ModiaBase.SVector{3,Float64}(1.0, 0.0, 0.0)))
)

bouncingCones = @instantiateModel(BouncingCones, unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 1.2
tolerance = 1e-8
requiredFinalStates = [-0.7999312016374764, 1.0550588860480428, 0.8921254770510955, -0.1896960061863984, 0.8841404598824826, 0.43402607612744604, 1.3606906655246873, 1.1282714887339953, 2.052883317050297, 4.6497266384364115, 0.052126888408507835, 1.009893100393836, 0.8916594826499894, -0.8000400934802586, 1.0551915122440447, 0.43363393980540194, -0.19001673923281473, 0.8846394848571297, 2.735874415024229, -0.2906589834171084, -1.476919610003813, 1.010451091809992, 4.65340945904709, 0.051874592801066474, 1.0551437465113622, 0.8923792400352838, -0.8001536268911364, 0.8848687234167343, 0.4342006445471319, -0.19173561019070964, -1.7767415342672759, -0.2277632915908899, 2.6985438395240826, 0.05325010071392679, 1.012579595034664, 4.656827261542965]
simulate!(bouncingCones, stopTime=stopTime, tolerance=tolerance, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(bouncingCones, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
                     ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
