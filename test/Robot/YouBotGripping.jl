module YouBotGripping

using Modia3D

include("$(Modia3D.modelsPath)/Blocks.jl")
include("$(Modia3D.modelsPath)/Electric.jl")
include("$(Modia3D.modelsPath)/Rotational.jl")
include("$(Modia3D.modelsPath)/Translational.jl")


# some constants
simplifiedContact = true  # use boxes instead of meshes for finger contact

# visual materials
vmatYellow    = VisualMaterial(color=[255,215,0])    # ground
vmatBlue      = VisualMaterial(color="DodgerBlue3")  # table
vmatGrey      = VisualMaterial(color="DarkGrey")     # work piece
vmatInvisible = VisualMaterial(transparency=1.0)     # contact boxes
tableX            = 0.3
tableY            = 0.3
tableZ            = 0.01
heigthLeg         = 0.355
widthLeg          = 0.02
diameter          = 0.05
rightFingerOffset = 0.03

# all needed paths to fileMeshes
base_frame_obj           = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/base_frame.obj")
plate_obj                = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/plate.obj")
front_right_wheel_obj    = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/front-right_wheel.obj")
front_left_wheel_obj     = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/front-left_wheel.obj")
back_right_wheel_obj     = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/back-right_wheel.obj")
back_left_wheel_obj      = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/back-left_wheel.obj")
arm_base_frame_obj       = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_base_frame.obj")
arm_joint_1_obj          = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_joint_1.obj")
arm_joint_2_obj          = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_joint_2.obj")
arm_joint_3_obj          = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_joint_3.obj")
arm_joint_4_obj          = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_joint_4.obj")
arm_joint_5_obj          = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/arm_joint_5.obj")
gripper_base_frame_obj   = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/gripper_base_frame.obj")
gripper_left_finger_obj  = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/gripper_left_finger.obj")
gripper_right_finger_obj = joinpath(Modia3D.path, "objects/robot_KUKA_YouBot/gripper_right_finger.obj")

# Drive train data: motor inertias and gear ratios
nullRot = nothing

motorInertia1 = 0.0000135 + 0.000000409
motorInertia2 = 0.0000135 + 0.000000409
motorInertia3 = 0.0000135 + 0.000000071
motorInertia4 = 0.00000925 + 0.000000071
motorInertia5 = 0.0000035 + 0.000000071
gearRatio1    = 156.0
gearRatio2    = 156.0
gearRatio3    = 100.0
gearRatio4    = 71.0
gearRatio5    = 71.0

m1=1.390
translation1 =[0.033,0,0]
rotation1 = [180u"°", 0, 0]

m2=1.318
translation2=[0.155,0,0]
rotation2 = [90u"°", 0.0, -90u"°"]

m3=0.821
translation3=[0,0.135,0]
rotation3 = [0, 0, -90u"°"]

m4=0.769
translation4=[0,0.11316,0]

m5=0.687
translation5=[0,0,0.05716]
rotation5 = [-90u"°", 0, 0]

### ----------------- Servo Model -----------------------
# parameters for Link
axisLink = 3
dLink  = 1.0
k1Link = 50.0
k2Link = 0.1
T2Link = 1.0
# parameters for Gripper
axisGripper = 2
k1Gripper = 50.0
k2Gripper = 0.1*100
T2Gripper = 1.0
motorInertiaGripper = 0.1
gearRatioGripper    = 1.0

#### ----------- Path Planning ------------------
referencePath1 = Modia3D.PathPlanning.ReferencePath(
    names =    ["angle1", "angle2", "angle3", "angle4", "angle5", "gripper"],
    position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    v_max =    [2.68512, 2.68512, 4.8879, 5.8997, 5.8997, 2.0],
    a_max =    [1.5, 1.5, 1.5, 1.5, 1.5, 0.5])

Modia3D.PathPlanning.ptpJointSpace(referencePath = referencePath1, positions =
    [0.0  0.0    0.0       0.0   0.0  0.0;
     pi   pi/4   pi/4      0.0   0.0  diameter+0.01;
     pi   pi/4   pi/4      1.04  0.0  diameter+0.01;
     pi   pi/4   pi/4      1.04  0.0  diameter-0.002;
     pi   pi/4   pi/4      0.0   0.0  diameter-0.002;
     0.0  0.0    pi/2-0.3  0.0   0.0  diameter-0.002;
     0.0  0.3    pi/2-0.3  0.0   0.0  diameter-0.002;
     0.0  0.3    pi/2-0.3  0.0   0.0  diameter+0.01;
     0.0  0.0    pi/2      0.0   0.0  diameter+0.01;
     0.0  0.0    0.0       0.0   0.0  0.03])

getReferencePath() = referencePath1

# Controller Model
Controller = Model(
    # Interface
    refLoadAngle     = input,
    actualMotorAngle = input,
    actualMotorSpeed = input,
    refTorque        = output,
    # Components
    gainOuter     = Gain,
    angleFeedback = Feedback,
    gainInner     = Gain,
    speedFeedback = Feedback,
    PI = PI,

    connect = :[
        (refLoadAngle, gainOuter.u)
        (gainOuter.y, angleFeedback.u1)
        (actualMotorAngle, angleFeedback.u2)
        (angleFeedback.y, gainInner.u)
        (gainInner.y, speedFeedback.u1)
        (actualMotorSpeed, speedFeedback.u2)
        (speedFeedback.y, PI.u)
        (PI.y, refTorque)]
)

Drive = Model(
    # Interface
    refTorque   = input,
    actualSpeed = output,
    actualAngle = output,
    flange      = Flange,
    # Components
    torque       = UnitlessTorque,
    speedSensor  = UnitlessSpeedSensor,
    angleSensor  = UnitlessAngleSensor,
    motorInertia = Inertia,
    idealGear    = IdealGear,

    connect = :[
        (refTorque, torque.tau)
        (torque.flange, speedSensor.flange, angleSensor.flange, motorInertia.flange_a)
        (motorInertia.flange_b, idealGear.flange_a)
        (idealGear.flange_b, flange)
        (speedSensor.w, actualSpeed)
        (angleSensor.phi, actualAngle)
        ]
)

Servo = Model(
    refLoadAngle = input,
    flange       = Flange,
    ppi          = Controller,
    drive        = Drive,

    connect = :[
        # inputs of ppi
        (refLoadAngle, ppi.refLoadAngle)
        (ppi.actualMotorSpeed, drive.actualSpeed)
        (ppi.actualMotorAngle, drive.actualAngle)
        # output of ppi --> input of motor
        (ppi.refTorque, drive.refTorque)
        # Flange of drive
        (drive.flange, flange)
        ]
)

# Controller Model
ControllerTrans = Model(
    # Interface
    refLoadPos     = input,
    actualMotorPos = input,
    actualMotorSpeed = input,
    refForce        = output,
    # Components
    gainOuter     = Gain,
    angleFeedback = Feedback,
    gainInner     = Gain,
    speedFeedback = Feedback,
    PI = PI,

    connect = :[
        (refLoadPos, gainOuter.u)
        (gainOuter.y, angleFeedback.u1)
        (actualMotorPos, angleFeedback.u2)
        (angleFeedback.y, gainInner.u)
        (gainInner.y, speedFeedback.u1)
        (actualMotorSpeed, speedFeedback.u2)
        (speedFeedback.y, PI.u)
        (PI.y, refForce)]
)

DriveTrans = Model(
    # Interface
    refForce    = input,
    actualSpeed = output,
    actualPos   = output,
    flange      = TranslationalFlange,
    # Components
    force       = UnitlessForce,
    speedSensor = UnitlessVelocitySensor,
    posSensor   = UnitlessPositionSensor,
    motorMass   = TranslationalMass,

    connect = :[
        (refForce, force.f)
        (force.flange, speedSensor.flange, posSensor.flange, motorMass.flange_a)
        (motorMass.flange_b, flange)
        (speedSensor.v, actualSpeed)
        (posSensor.s, actualPos)
        ]
)

ServoTrans = Model(
    refLoadPos = input,
    flange     = TranslationalFlange,
    ppi        = ControllerTrans,
    drive      = DriveTrans,

    connect = :[
        # inputs of ppi
        (refLoadPos, ppi.refLoadPos)
        (ppi.actualMotorSpeed, drive.actualSpeed)
        (ppi.actualMotorPos, drive.actualPos)
        # output of ppi --> input of motor
        (ppi.refForce, drive.refForce)
        # Flange of drive
        (drive.flange, flange)
        ]
)

Ground = Model(
    ground = Object3D(parent=:world,
        translation=[0.0, 0.0, -0.005], feature = Solid(shape =Box(lengthX=2.5, lengthY=2.5, lengthZ=0.01), solidMaterial="DryWood", visualMaterial=vmatYellow))
)

Table = Model(
    plate = Object3D(parent=:world, translation=[0.0, 0.0, heigthLeg],
        feature=Solid(shape=Box(lengthX=tableX, lengthY=tableY, lengthZ=tableZ),
        collisionSmoothingRadius=0.001, solidMaterial="DryWood", collision=true, visualMaterial=vmatBlue)),
    leg1  = Object3D(parent=:plate, translation=[ tableX/2-widthLeg/2,  tableY/2-widthLeg/2, -heigthLeg/2],
        feature=Solid(shape=Box(lengthX=widthLeg, lengthY=widthLeg, lengthZ=heigthLeg),
        solidMaterial="DryWood", visualMaterial=vmatBlue)),
    leg2  = Object3D(parent=:plate, translation=[-tableX/2+widthLeg/2,  tableY/2-widthLeg/2, -heigthLeg/2],
        feature=Solid(shape=Box(lengthX=widthLeg, lengthY=widthLeg, lengthZ=heigthLeg), solidMaterial="DryWood", visualMaterial=vmatBlue)),
    leg3  = Object3D(parent=:plate, translation=[ tableX/2-widthLeg/2, -tableY/2+widthLeg/2, -heigthLeg/2],
        feature=Solid(shape=Box(lengthX=widthLeg, lengthY=widthLeg, lengthZ=heigthLeg), solidMaterial="DryWood", visualMaterial=vmatBlue)),
    leg4  = Object3D(parent=:plate, translation=[-tableX/2+widthLeg/2, -tableY/2+widthLeg/2, -heigthLeg/2],
        feature=Solid(shape=Box(lengthX=widthLeg, lengthY=widthLeg, lengthZ=heigthLeg), solidMaterial="DryWood", visualMaterial=vmatBlue))
)

Base = Model(
    base_frame = Object3D(parent=:world, translation=[-0.585, 0, 0.084],
        feature=Solid(shape=FileMesh(filename = base_frame_obj), massProperties=MassPropertiesFromShapeAndMass(mass=19.803))),
    plate = Object3D(parent=:base_frame, translation=[-0.159, 0, 0.046],
        feature=Solid(shape=FileMesh(filename = plate_obj), massProperties=MassPropertiesFromShapeAndMass(mass=2.397), contactMaterial="DryWood", collision=!simplifiedContact)),
    plate_box = Object3D(parent=:base_frame, translation=[-0.16, 0, 0.0702],# 0.06514],
        feature=Solid(shape=Box(lengthX=0.22, lengthY=0.22, lengthZ=0.01), massProperties=MassPropertiesFromShapeAndMass(mass=1.0e-9), visualMaterial=vmatInvisible, contactMaterial="DryWood", collision=simplifiedContact)),
    front_right_wheel = Object3D(parent=:base_frame, translation=[0.228, -0.158, -0.034],
        feature=Solid(shape=FileMesh(filename = front_right_wheel_obj), massProperties=MassPropertiesFromShapeAndMass(mass=1.4))),
    front_left_wheel = Object3D(parent=:base_frame, translation=[0.228, 0.158, -0.034],
        feature=Solid(shape=FileMesh(filename = front_left_wheel_obj), massProperties=MassPropertiesFromShapeAndMass(mass=1.4))),
    back_right_wheel = Object3D(parent=:base_frame, translation=[-0.228, -0.158, -0.034],
        feature=Solid(shape=FileMesh(filename = back_right_wheel_obj), massProperties=MassPropertiesFromShapeAndMass(mass=1.4))),
    back_left_wheel   = Object3D(parent=:base_frame, translation=[-0.228, 0.158, -0.034],
        feature=Solid(shape=FileMesh(filename = back_left_wheel_obj), massProperties=MassPropertiesFromShapeAndMass(mass=1.4)))
)

servoParameters1 = Map(
                      ppi = Map(
                          gainOuter = Map(k=gearRatio1),
                          gainInner = Map(k=k1Link),
                          PI        = Map(k=k2Link, T=T2Link)
                      ),
                      drive = Map(
                          motorInertia = Map(J=motorInertia1),
                          idealGear = Map(ratio=gearRatio1)
                      )
                  )

servoParameters2 = Map(
                      ppi = Map(
                          gainOuter = Map(k=gearRatio2),
                          gainInner = Map(k=k1Link),
                          PI        = Map(k=k2Link, T=T2Link)
                      ),
                      drive = Map(
                          motorInertia = Map(J=motorInertia2),
                          idealGear = Map(ratio=gearRatio2)
                      )
                  )

servoParameters3 = Map(
                      ppi = Map(
                          gainOuter = Map(k=gearRatio3),
                          gainInner = Map(k=k1Link),
                          PI        = Map(k=k2Link, T=T2Link)
                      ),
                      drive = Map(
                          motorInertia = Map(J=motorInertia3),
                          idealGear = Map(ratio=gearRatio3)
                      )
                  )

servoParameters4 = Map(
                      ppi = Map(
                          gainOuter = Map(k=gearRatio4),
                          gainInner = Map(k=k1Link),
                          PI        = Map(k=k2Link, T=T2Link)
                      ),
                      drive = Map(
                          motorInertia = Map(J=motorInertia4),
                          idealGear = Map(ratio=gearRatio4)
                      )
                  )

servoParameters5 = Map(
                      ppi = Map(
                          gainOuter = Map(k=gearRatio5),
                          gainInner = Map(k=k1Link),
                          PI        = Map(k=k2Link, T=T2Link)
                      ),
                      drive = Map(
                          motorInertia = Map(J=motorInertia5),
                          idealGear = Map(ratio=gearRatio5)
                      )
                  )

servoParameters6 = Map(
                      ppi = Map(
                          gainOuter = Map(k=gearRatioGripper),
                          gainInner = Map(k=k1Gripper),
                          PI        = Map(k=k2Gripper, T=T2Gripper)
                      ),
                      drive = Map(
                          motorMass = Map(m=motorInertiaGripper)
                      )
                  )

featureBody1 = Solid(shape = FileMesh(filename = arm_joint_1_obj), massProperties = MassPropertiesFromShapeAndMass(mass=m1))
featureBody2 = Solid(shape = FileMesh(filename = arm_joint_2_obj), massProperties = MassPropertiesFromShapeAndMass(mass=m2))
featureBody3 = Solid(shape = FileMesh(filename = arm_joint_3_obj), massProperties = MassPropertiesFromShapeAndMass(mass=m3))
featureBody4 = Solid(shape = FileMesh(filename = arm_joint_4_obj), massProperties = MassPropertiesFromShapeAndMass(mass=m4))
featureBody5 = Solid(shape = FileMesh(filename = arm_joint_5_obj), massProperties = MassPropertiesFromShapeAndMass(mass=m5))

linkParameters1 = Map(
                    parent1 = Par(value = :(armBase_b)),
                    featureBody = featureBody1,
                    initRefPos = referencePath1.position[1],
                    trans = translation1,
                    rota = Par(value = :(rotation1))
)

linkParameters2 = Map(
                    parent1 = Par(value = :(link1.obj2)),
                    featureBody = featureBody2,
                    m = m2,
                    initRefPos = referencePath1.position[2],
                    trans = translation2,
                    rota = Par(value = :(rotation2))
)

linkParameters3 = Map(
                    parent1 = Par(value = :(link2.obj2)),
                    featureBody = featureBody3,
                    m = m3,
                    initRefPos = referencePath1.position[3],
                    trans = translation3,
                    rota = Par(value = :(rotation3))
)

linkParameters4 = Map(
                    parent1 = Par(value = :(link3.obj2)),
                    featureBody = featureBody4,
                    m = m4,
                    initRefPos = referencePath1.position[4],
                    trans = translation4,
                    rota = Par(value = :(nullRot))
)

linkParameters5 = Map(
                    parent1 = Par(value = :(link4.obj2)),
                    featureBody = featureBody5,
                    m = m5,
                    initRefPos = referencePath1.position[5],
                    trans = translation5,
                    rota = Par(value = :(rotation5))
)

Link = Model(
    parent1 = Par(value = :(nothing)),
    featureBody = featureBody1,
    trans = [0,0,0],
    rota = Par(value = :(nullRot)),

    obj1 = Object3D(parent=:parent1, rotation=:rota),
    body = Object3D(feature = :featureBody),
    obj2 = Object3D(parent =:body, translation = :trans),
)

Gripper = Model(
    obj1 = Object3D(parent=:(link5.obj2), rotation = [0, 0, -180u"°"]),
    gripper_base_frame = Object3D(parent=:obj1, feature=Solid(shape=
        FileMesh(filename = gripper_base_frame_obj), massProperties=MassPropertiesFromShapeAndMass(mass=0.199))),
    gripper_left_finger_a = Object3D(),
    gripper_left_finger = Object3D(parent=:gripper_left_finger_a, translation=[0, 0.0082, 0],
        feature=Solid(shape= FileMesh(filename = gripper_left_finger_obj), massProperties=MassPropertiesFromShapeAndMass(mass=0.010), contactMaterial="DryWood", collision=!simplifiedContact)),
    gripper_left_finger_box = Object3D(parent=:gripper_left_finger_a, translation=[0.0, 0.0051, 0.0135],
        feature = Solid(shape=Box(lengthX=0.012, lengthY=0.01, lengthZ=0.045), massProperties=MassPropertiesFromShapeAndMass(mass=1.0e-9), visualMaterial=vmatInvisible, contactMaterial="DryWood", collision=simplifiedContact)),
    gripper_right_finger_a = Object3D(parent=:gripper_base_frame,
        translation=[0,-rightFingerOffset,0]),
    gripper_right_finger = Object3D(parent=:gripper_right_finger_a, translation=[0, -0.0082, 0],
        feature = Solid(shape = FileMesh(filename = gripper_right_finger_obj), massProperties=MassPropertiesFromShapeAndMass(mass=0.010), contactMaterial="DryWood", collision=!simplifiedContact)),
    gripper_right_finger_box = Object3D(parent=:gripper_right_finger_a, translation=[0.0, -0.0051, 0.0135],
        feature = Solid(shape=Box(lengthX=0.012, lengthY=0.01, lengthZ=0.045), massProperties=MassPropertiesFromShapeAndMass(mass=1.0e-9), visualMaterial=vmatInvisible, contactMaterial="DryWood", collision=simplifiedContact))
)

YouBot = Model(
    base = Base,
    arm_base_frame = Object3D(parent=:(base.base_frame),
        translation=[0.143, 0.0, 0.046],
        feature = Solid(shape = FileMesh(filename = arm_base_frame_obj), massProperties=MassPropertiesFromShapeAndMass(mass=0.961))),
    armBase_b = Object3D(parent=:arm_base_frame, translation=[0.024, 0, 0.115]),

    link1 = Link | linkParameters1,
    link2 = Link | linkParameters2,
    link3 = Link | linkParameters3,
    link4 = Link | linkParameters4,
    link5 = Link | linkParameters5,
    gripper = Gripper,

    rev1 = RevoluteWithFlange(obj1 = :(link1.obj1), obj2 = :(link1.body),
        axis=axisLink, phi = Var(init = getRefPathInitPosition(referencePath1, 1)), w=Var(init=0.0)),
    rev2 = RevoluteWithFlange(obj1 = :(link2.obj1), obj2 = :(link2.body),
        axis=axisLink, phi = Var(init = getRefPathInitPosition(referencePath1, 2)), w=Var(init=0.0)),
    rev3 = RevoluteWithFlange(obj1 = :(link3.obj1), obj2 = :(link3.body),
        axis=axisLink, phi = Var(init = getRefPathInitPosition(referencePath1, 3)), w=Var(init=0.0)),
    rev4 = RevoluteWithFlange(obj1 = :(link4.obj1), obj2 = :(link4.body),
        axis=axisLink, phi = Var(init = getRefPathInitPosition(referencePath1, 4)), w=Var(init=0.0)),
    rev5 = RevoluteWithFlange(obj1 = :(link5.obj1), obj2 = :(link5.body),
        axis=axisLink, phi = Var(init = getRefPathInitPosition(referencePath1, 5)), w=Var(init=0.0)),

    prism = PrismaticWithFlange(obj1 = :(gripper.gripper_right_finger_a), obj2 = :(gripper.gripper_left_finger_a),
        axis=axisGripper, s = Var(init = getRefPathInitPosition(referencePath1, 6)) ),

    servo1 = Servo,
    servo2 = Servo,
    servo3 = Servo,
    servo4 = Servo,
    servo5 = Servo,
    servo6 = ServoTrans,

    equations=:[
        refPath = calculateRobotMovement(getReferencePath(), instantiatedModel),
        servo1.refLoadAngle = getRefPathPosition(refPath, 1),
        servo2.refLoadAngle = getRefPathPosition(refPath, 2),
        servo3.refLoadAngle = getRefPathPosition(refPath, 3),
        servo4.refLoadAngle = getRefPathPosition(refPath, 4),
        servo5.refLoadAngle = getRefPathPosition(refPath, 5),
        servo6.refLoadPos   = getRefPathPosition(refPath, 6)
    ],

    connect = :[
        (servo1.flange, rev1.flange)
        (servo2.flange, rev2.flange)
        (servo3.flange, rev3.flange)
        (servo4.flange, rev4.flange)
        (servo5.flange, rev5.flange)
        (servo6.flange, prism.flange)
        ]
)

Scenario = Model3D(
    gravField = UniformGravityField(g=9.81, n=[0,0,-1]),
    world = Object3D(feature=Scene(gravityField=:gravField,mprTolerance = 1.0e-13, visualizeFrames=false, nominalLength=tableX,
    animationFile="YouBotGripping.json",
    enableContactDetection=true, maximumContactDamping=1000, elasticContactReductionFactor=1e-3)),
#   worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.2))),

    table = Table,
    ground = Ground,

    sphere = Object3D(
        feature=Solid(shape=Sphere(diameter=diameter), visualMaterial=vmatGrey, solidMaterial="DryWood", collision=true)),

    sphereJoint = FreeMotion(obj1=:world, obj2=:sphere, r=Var(init=Modia.SVector{3,Float64}(-0.799, 0.005, 0.1842))),  # 0.1792

    youbot1 = YouBot,
)

modelParameters = Map(
    youbot1 = Map(
        servo1 = servoParameters1,
        servo2 = servoParameters2,
        servo3 = servoParameters3,
        servo4 = servoParameters4,
        servo5 = servoParameters5,
        servo6 = servoParameters6
    )
)

youbotModel = Scenario | modelParameters

youbot = @instantiateModel(youbotModel, unitless=true, logCode=false, log=false)

stopTime = 14.0
tolerance = 1e-5
if simplifiedContact
    requiredFinalStates = [-0.0034351234016326967, -0.005096438607421972, 0.3849919166786696, 3.7229476165027234e-9, 1.3643680751630356e-12, -8.773513748073357e-12, 0.0007801023786739114, -1.0295269969841694, 3.1423881978880415, 8.226252288034842e-12, -1.3249736451599036e-7, -1.1589254631531364e-11, -8.486904800510932e-7, 8.498125796982337e-7, -1.0481970538614587e-5, 1.0487147314270069e-5, -2.3853976413064467e-5, 2.3390184409166785e-5, -1.4665554655810717e-5, 1.4674378011837178e-5, 1.494217786661545e-7, -1.494826954306272e-7, -0.006484081235183565, 0.1615223363461487, -0.11771015565151635, -0.051952238911059184, 0.0005197823054442761, -0.014728137528941643, 0.02969949577720149, 0.0003004151601044065]
else
    requiredFinalStates = [0.0009285649671864051, -0.00510121706010873, 0.38499183731994097, 1.3815799549216646e-8, -3.013883724849587e-8, -1.3548815880513574e-12, 0.3728261933992865, -0.996577976084585, 3.458983136911289, -5.493765692434776e-7, -2.9787026664428377e-7, -9.903133313245147e-7, -8.815312501299099e-7, 8.819099920244317e-7, -1.048429764456881e-5, 1.0485482464956499e-5, -2.38693150412509e-5, 2.3877893868562615e-5, -1.4676089351824731e-5, 1.467815727005773e-5, 1.5116246063921994e-7, -1.5117524141251032e-7, -0.00673525336167914, 0.1615038110919147, -0.1177385926739786, -0.05198945600575353, 0.0005258421886067691, -0.01468148168734971, 0.029700446414509994, 0.000299495164936174]
end
simulate!(youbot, stopTime=stopTime, tolerance=tolerance, useRecursiveFactorizationUptoSize=500, requiredFinalStates_atol=0.001, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(youbot, ["sphereJoint.r"], figure=1)

end
