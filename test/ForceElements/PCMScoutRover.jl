module PCMScoutRover

using Modia3D

include("$(Modia3D.modelsPath)/Blocks.jl")
include("$(Modia3D.modelsPath)/Electric.jl")
include("$(Modia3D.modelsPath)/Rotational.jl")

wheelSpeed = 2.0u"rad/s"
payLoad = 3.0

hubDiameter = 0.06
hubLength = 0.069
hubMass = 0.2
hubLateralPosition = (0.35 + 0.069)/2
hubMassProperties = MassPropertiesFromShapeAndMass(mass=hubMass)
hubVisualMaterial = VisualMaterial(color="lightsteelblue3", reflectslight=true, shininess=1.0)
spokeLength = 0.15
spokeWidth = 0.044
spokeLateralOffset = (hubLength - spokeWidth)/2
spokeMass = 0.11
legThickness = 0.02
legVisualMaterial = VisualMaterial(color="grey15", reflectslight=false, shininess=0.5)
upperLegLength = 0.094
upperLegKneePositionZ = -spokeLength/2
upperLegKneePositionX = sqrt(upperLegLength^2 - upperLegKneePositionZ^2)
upperLegMassProperties = MassPropertiesFromShapeAndMass(mass=spokeMass/2)
lowerLegLength = 0.094
lowerLegFootPositionZ = -spokeLength/2
lowerLegFootPositionX = -sqrt(lowerLegLength^2 - lowerLegFootPositionZ^2)
lowerLegMassProperties = MassPropertiesFromShapeAndMass(mass=spokeMass/2)
kneeStiffness = 35.0
kneeDamping = 2*0.1*sqrt(0.265*kneeStiffness)
footMass = 0.02
footDiameter = 0.06
footInnerDiameter = 0.05
footPitchAngle = -0/180*pi
footStiffness = 10000
footDamping = 2*0.1*sqrt(footMass*footStiffness)
footMassProperties = MassPropertiesFromShapeAndMass(mass=footMass)
footVisualMaterial = VisualMaterial(color="ghostwhite", reflectslight=false, shininess=0.5)
vertebraePitchStiffness = 10.0
vertebraePitchDamping = 2*0.1*sqrt(6.0*0.2^2*vertebraePitchStiffness)
vertebraeRollStiffness = 30.0
vertebraeRollDamping = 2*0.1*sqrt(6.0*0.1^2*vertebraeRollStiffness)
vertebraeVisualMaterial = VisualMaterial(color="grey15", reflectslight=false, shininess=0.5)
segmentMass = (payLoad + 12.0 - 6*(hubMass + 3*(spokeMass + footMass)))/3
segmentLengthX = 0.2
middleSegmentLengthX = 0.22
segmentLengthY = 0.35
segmentLengthZ = 0.1
segmentMassProperties = MassPropertiesFromShapeAndMass(mass=segmentMass)
segmentVisualMaterial = VisualMaterial(color="lightsteelblue1", transparency=0.5, reflectslight=true, shininess=0.8)

terrainFileMesh = joinpath(Modia3D.path, "objects", "pcm", "fractal_terrain.obj")
terrainVisualMaterial = VisualMaterial(color="sandybrown", reflectslight=false, shininess=0.0)
footFileMesh = joinpath(Modia3D.path, "objects", "pcm", "scout_foot.obj")
footFileMeshScale = 0.001

pcmFootTerrain = Map(task=3,
                     maxPenetration=0.01,
                     bodyF_YoungsModulus=5.0e6,
                     bodyF_PoissonsRatio=0.4,
                     bodyF_LayerDepth=0.01,
                     compressionArealDampingFactor=2.0e4,
                     expansionArealDampingFactor=2.0e4,
                     dampingTransitionDepth=0.0005,
                     frictionCoefficient=0.7,
                     frictionRegularizationVelocity=0.001)

ConstantVelocity = Model(
    flange = Flange,
    w = 0.0u"rad/s",
    phi_0 = 0.0u"rad",
    equations = :[flange.phi = w*time + phi_0]
)

Spoke = Model(
    upperLeg = Object3D(),
    upperLegBody = Object3D(parent=:upperLeg,
                            translation=[upperLegKneePositionX/2, 0, upperLegKneePositionZ/2],
                            rotation=[0, atan(upperLegKneePositionX, upperLegKneePositionZ), 0],
                            feature=Solid(shape=Beam(axis=3, length=upperLegLength, width=legThickness, thickness=spokeWidth-0.001),
                                          massProperties=upperLegMassProperties,
                                          visualMaterial=legVisualMaterial)),
    upperLegKnee = Object3D(parent=:upperLeg,
                            translation=[upperLegKneePositionX, 0, upperLegKneePositionZ]),
    lowerLeg = Object3D(),
    lowerLegBody = Object3D(parent=:lowerLeg,
                            translation=[lowerLegFootPositionX/2, 0, lowerLegFootPositionZ/2],
                            rotation=[0, atan(lowerLegFootPositionX, lowerLegFootPositionZ), 0],
                            feature=Solid(shape=Beam(axis=3, length=lowerLegLength, width=legThickness, thickness=spokeWidth-0.001),
                                          massProperties=lowerLegMassProperties,
                                          visualMaterial=legVisualMaterial)),
    lowerLegFoot = Object3D(parent=:lowerLeg,
                            translation=[lowerLegFootPositionX, 0, lowerLegFootPositionZ]),
    foot = Object3D(),
    footBody = Object3D(parent=:foot,
                        rotation=[0, footPitchAngle, 0],
                        feature=Solid(shape=FileMesh(filename=footFileMesh, scale=fill(footFileMeshScale, 3), useMaterialColor=true),
                                      massProperties=footMassProperties,
                                      visualMaterial=footVisualMaterial)),
    kneeJoint = RevoluteWithFlange(obj1=:upperLegKnee, obj2=:lowerLeg, axis=2),
    footJoint = Prismatic(obj1=:lowerLegFoot, obj2=:foot, axis=3),
    kneeSpringDamper = SpringDamper | Map(c=kneeStiffness, d=kneeDamping),
    kneeFixed = Fixed,
    connect = :[(kneeSpringDamper.flange_b, kneeJoint.flange),
                (kneeSpringDamper.flange_a, kneeFixed.flange)],
    footSpringDamper = Bushing(obj1=:lowerLegFoot, obj2=:foot,
                               springForceLaw=[0, 0, footStiffness],
                               damperForceLaw=[0, 0, footDamping])
)

Wheel = Model(
    isLeft = true,
    phaseAngle = 0,
    hub = Object3D(feature=Solid(shape=Cylinder(axis=2, diameter=hubDiameter, length=hubLength),
                                 massProperties=hubMassProperties,
                                 visualMaterial=hubVisualMaterial)),
    hubSpoke1 = Object3D(parent=:hub,
                         translation=:([0, isLeft ? spokeLateralOffset : -spokeLateralOffset, 0]),
                         rotation=:([0, phaseAngle, 0])),
    hubSpoke2 = Object3D(parent=:hub,
                         translation=:([0, isLeft ? spokeLateralOffset : -spokeLateralOffset, 0]),
                         rotation=:([0, phaseAngle+2*pi/3, 0])),
    hubSpoke3 = Object3D(parent=:hub,
                         translation=:([0, isLeft ? spokeLateralOffset : -spokeLateralOffset, 0]),
                         rotation=:([0, phaseAngle-2*pi/3, 0])),
    spoke1 = Spoke,
    spoke2 = Spoke,
    spoke3 = Spoke,
    spoke1Joint = Fix(obj1=:hubSpoke1, obj2=:(spoke1.upperLeg)),
    spoke2Joint = Fix(obj1=:hubSpoke2, obj2=:(spoke2.upperLeg)),
    spoke3Joint = Fix(obj1=:hubSpoke3, obj2=:(spoke3.upperLeg))
)

Segment = Model(
    body = Object3D(feature=Solid(shape=Box(lengthX=segmentLengthX, lengthY=segmentLengthY, lengthZ=segmentLengthZ),
                                  massProperties=segmentMassProperties,
                                  visualMaterial=segmentVisualMaterial)),
    frontJoint = Object3D(parent=:body, translation=[0.2, 0, 0]),
    rearJoint = Object3D(parent=:body, translation=[-0.2, 0, 0]),
    leftWheel = Wheel | Map(isLeft=true, phaseAngle=0),
    rightWheel = Wheel | Map(isLeft=false, phaseAngle=0),
    leftHub = Object3D(parent=:body, translation=[0, hubLateralPosition, 0]),
    rightHub = Object3D(parent=:body, translation=[0, -hubLateralPosition, 0]),
    leftJoint = RevoluteWithFlange(obj1=:leftHub, obj2=:(leftWheel.hub), axis=2, w=Var(init=wheelSpeed)),
    rightJoint = RevoluteWithFlange(obj1=:rightHub, obj2=:(rightWheel.hub), axis=2, w=Var(init=wheelSpeed)),
    leftJointAngle = ConstantVelocity | Map(w=wheelSpeed),
    rightJointAngle = ConstantVelocity | Map(w=wheelSpeed),
    connect = :[(leftJointAngle.flange, leftJoint.flange),
                (rightJointAngle.flange, rightJoint.flange)]
)

ScoutRover = Model3D(
    # world
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
#                                  enableVisualization=false,
                                   enableContactDetection=false,
                                   animationFile="PCMScoutRover.json",
                                   nominalLength=1.0)),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=0.2))),
    terrain = Object3D(parent=:world,
                       feature=Solid(shape=FileMesh(filename=terrainFileMesh, scale=fill(10/128, 3), useMaterialColor=true),
                                     massProperties=MassProperties(mass=1.0),
                                     visualMaterial=terrainVisualMaterial),
                       translation=[-5, -5, -1.34]),
    # segments
    frontSegment = Segment,
    middleSegment = Segment,
    rearSegment = Segment,
    # joints
    middleJoint = FreeMotion2(obj1=:world, obj2=:(middleSegment.body),
                              r=Var(init=Modia.SVector{3,Float64}(-4, -2, -0.4)),
                              rot=Var(init=Modia.SVector{3,Float64}([0, 0, deg2rad(30)]))),
    frontJoint = Object3D(feature=Visual(shape=Box(lengthX=0.2, lengthY=0.1, lengthZ=0.01),
                          visualMaterial=vertebraeVisualMaterial)),
    frontPitchJoint = RevoluteWithFlange(obj1=:(middleSegment.frontJoint), obj2=:frontJoint, axis=2),
    frontRollJoint = RevoluteWithFlange(obj1=:frontJoint, obj2=:(frontSegment.rearJoint), axis=1),
    rearJoint = Object3D(feature=Visual(shape=Box(lengthX=0.2, lengthY=0.1, lengthZ=0.01),
                         visualMaterial=vertebraeVisualMaterial)),
    rearPitchJoint = RevoluteWithFlange(obj1=:(middleSegment.rearJoint), obj2=:rearJoint, axis=2),
    rearRollJoint = RevoluteWithFlange(obj1=:rearJoint, obj2=:(rearSegment.frontJoint), axis=1),
    # vertebrae
    frontPitchSpringDamper = SpringDamper | Map(c=vertebraePitchStiffness, d=vertebraePitchDamping),
    frontRollSpringDamper = SpringDamper | Map(c=vertebraeRollStiffness, d=vertebraeRollDamping),
    frontPitchFixed = Fixed,
    frontRollFixed = Fixed,
    rearPitchSpringDamper = SpringDamper | Map(c=vertebraePitchStiffness, d=vertebraePitchDamping),
    rearRollSpringDamper = SpringDamper | Map(c=vertebraeRollStiffness, d=vertebraeRollDamping),
    rearPitchFixed = Fixed,
    rearRollFixed = Fixed,
    connect = :[(rearPitchSpringDamper.flange_b, rearPitchJoint.flange),
                (rearPitchSpringDamper.flange_a, rearPitchFixed.flange),
                (rearRollSpringDamper.flange_b, rearRollJoint.flange),
                (rearRollSpringDamper.flange_a, rearRollFixed.flange),
                (frontPitchSpringDamper.flange_b, frontPitchJoint.flange),
                (frontPitchSpringDamper.flange_a, frontPitchFixed.flange),
                (frontRollSpringDamper.flange_b, frontRollJoint.flange),
                (frontRollSpringDamper.flange_a, frontRollFixed.flange)],
    # contacts
    pcmFrontLeftFoot1 = PolygonalContactModel(obj1=:terrain, obj2=:(frontSegment.leftWheel.spoke1.footBody)) | pcmFootTerrain,
    pcmFrontLeftFoot2 = PolygonalContactModel(obj1=:terrain, obj2=:(frontSegment.leftWheel.spoke2.footBody)) | pcmFootTerrain,
    pcmFrontLeftFoot3 = PolygonalContactModel(obj1=:terrain, obj2=:(frontSegment.leftWheel.spoke3.footBody)) | pcmFootTerrain,
    pcmFrontRightFoot1 = PolygonalContactModel(obj1=:terrain, obj2=:(frontSegment.rightWheel.spoke1.footBody)) | pcmFootTerrain,
    pcmFrontRightFoot2 = PolygonalContactModel(obj1=:terrain, obj2=:(frontSegment.rightWheel.spoke2.footBody)) | pcmFootTerrain,
    pcmFrontRightFoot3 = PolygonalContactModel(obj1=:terrain, obj2=:(frontSegment.rightWheel.spoke3.footBody)) | pcmFootTerrain,
    pcmMiddleLeftFoot1 = PolygonalContactModel(obj1=:terrain, obj2=:(middleSegment.leftWheel.spoke1.footBody)) | pcmFootTerrain,
    pcmMiddleLeftFoot2 = PolygonalContactModel(obj1=:terrain, obj2=:(middleSegment.leftWheel.spoke2.footBody)) | pcmFootTerrain,
    pcmMiddleLeftFoot3 = PolygonalContactModel(obj1=:terrain, obj2=:(middleSegment.leftWheel.spoke3.footBody)) | pcmFootTerrain,
    pcmMiddleRightFoot1 = PolygonalContactModel(obj1=:terrain, obj2=:(middleSegment.rightWheel.spoke1.footBody)) | pcmFootTerrain,
    pcmMiddleRightFoot2 = PolygonalContactModel(obj1=:terrain, obj2=:(middleSegment.rightWheel.spoke2.footBody)) | pcmFootTerrain,
    pcmMiddleRightFoot3 = PolygonalContactModel(obj1=:terrain, obj2=:(middleSegment.rightWheel.spoke3.footBody)) | pcmFootTerrain,
    pcmRearLeftFoot1 = PolygonalContactModel(obj1=:terrain, obj2=:(rearSegment.leftWheel.spoke1.footBody)) | pcmFootTerrain,
    pcmRearLeftFoot2 = PolygonalContactModel(obj1=:terrain, obj2=:(rearSegment.leftWheel.spoke2.footBody)) | pcmFootTerrain,
    pcmRearLeftFoot3 = PolygonalContactModel(obj1=:terrain, obj2=:(rearSegment.leftWheel.spoke3.footBody)) | pcmFootTerrain,
    pcmRearRightFoot1 = PolygonalContactModel(obj1=:terrain, obj2=:(rearSegment.rightWheel.spoke1.footBody)) | pcmFootTerrain,
    pcmRearRightFoot2 = PolygonalContactModel(obj1=:terrain, obj2=:(rearSegment.rightWheel.spoke2.footBody)) | pcmFootTerrain,
    pcmRearRightFoot3 = PolygonalContactModel(obj1=:terrain, obj2=:(rearSegment.rightWheel.spoke3.footBody)) | pcmFootTerrain
)

pcmScoutRover = @instantiateModel(ScoutRover, aliasReduction=false, unitless=true)

algorithm = IDA() # CVODE_BDF() # Vern7() # Tsit5() # IDA()
tolerance = 1.0e-6
stopTime = 1.0 # 30.0
dtmax = 0.01
interval = 0.01
requiredFinalStates = missing
simulate!(pcmScoutRover, algorithm; tolerance=tolerance, stopTime=stopTime, dtmax=dtmax, interval=interval, log=true, logTiming=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(pcmScoutRover, ["frontPitchJoint.phi" "frontRollJoint.phi"
                     "frontPitchJoint.w"   "frontRollJoint.w"  ], figure=1)
plot(pcmScoutRover, ["rearPitchJoint.phi" "rearRollJoint.phi"
                     "rearPitchJoint.w"   "rearRollJoint.w"  ], figure=2)

end
