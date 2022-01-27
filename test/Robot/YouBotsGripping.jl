module YouBotsGripping

using  ModiaLang
import Modia3D
using  Unitful

# ModiaLang models
include("$(ModiaLang.path)/models/Blocks.jl")
include("$(ModiaLang.path)/models/Electric.jl")
include("$(ModiaLang.path)/models/Rotational.jl")
include("$(ModiaLang.path)/models/Translational.jl")

using Modia

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
heigthLeg         = 0.365
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
translation4 = [0,0.11316,0]

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
    names =    ["bot1angle1", "bot1angle2", "bot1angle3", "bot1angle4", "bot1angle5", "bot1gripper", "bot2angle1", "bot2angle2", "bot2angle3", "bot2angle4", "bot2angle5", "bot2gripper"],
    position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    v_max =    [2.68512, 2.68512, 4.8879, 5.8997, 5.8997, 2.0, 2.68512, 2.68512, 4.8879, 5.8997, 5.8997, 2.0],
    a_max =    [1.5, 1.5, 1.5, 1.5, 1.5, 0.5, 1.5, 1.5, 1.5, 1.5, 1.5, 0.5])

Modia3D.PathPlanning.ptpJointSpace(referencePath = referencePath1, positions =[
        0.0  0.0    0.0       0.0    0.0  0.0             0.0  0.0    0.0       0.0   0.0  0.0;
        pi   pi/4   pi/4      0.0    0.0  diameter+0.01   0.0  0.0    0.0       0.0   0.0  0.0;
        pi   pi/4   pi/4      1.057  0.0  diameter+0.01   0.0  0.0    0.0       0.0   0.0  0.0;
        pi   pi/4   pi/4      1.057  0.0  diameter-0.002  0.0  0.0    0.0       0.0   0.0  0.0;
        pi   pi/4   pi/4      0.0    0.0  diameter-0.002  0.0  0.0    0.0       0.0   0.0  0.0;
        0.0  0.0    pi/2-0.2  0.0    0.0  diameter-0.002  0.0  0.0    0.0       0.0   0.0  0.0;
        0.0  0.2    pi/2-0.2  0.0    0.0  diameter-0.002  0.0  0.0    0.0       0.0   0.0  0.0;
        0.0  0.2    pi/2-0.2  0.0    0.0  diameter+0.01   0.0  0.0    0.0       0.0   0.0  0.0;
        0.0  0.0    pi/2      0.0    0.0  diameter+0.01   0.0  0.0    0.0       0.0   0.0  0.0;
        0.0  0.0    0.0       0.0    0.0  0.001           0.0  0.0    0.0       0.0   0.0  0.0
        0.0  0.0    0.0       0.0    0.0  0.001           0.0  0.0    pi/2      0.0   0.0  diameter+0.01
        0.0  0.0    0.0       0.0    0.0  0.001           0.0  0.3    pi/2-0.3  0.0   0.0  diameter+0.01
        0.0  0.0    0.0       0.0    0.0  0.001           0.0  0.3    pi/2-0.3  0.0   0.0  diameter-0.002
        0.0  0.0    0.0       0.0    0.0  0.001           0.0  0.0    pi/2-0.3  0.0   0.0  diameter-0.002
        0.0  0.0    0.0       0.0    0.0  0.001           pi   pi/4   pi/4      0.0   0.0  diameter-0.002
        0.0  0.0    0.0       0.0    0.0  0.001           pi   pi/4   pi/4      1.0   0.0  diameter-0.002
        0.0  0.0    0.0       0.0    0.0  0.001           pi   pi/4   pi/4      1.0   0.0  diameter+0.01
        0.0  0.0    0.0       0.0    0.0  0.001           pi   pi/4   pi/4      0.0   0.0  diameter+0.01
        0.0  0.0    0.0       0.0    0.0  0.001           0.0  0.0    0.0       0.0   0.0  0.001
    ])

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
    position = [0.0, 0.0, 0.0],
    orientation = [0.0, 0.0, 0.0],
    base_frame = Object3D(parent=:world, translation=:position, rotation=:orientation,
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
    obj1 = Object3D(parent=:(link5.obj2), rotation=[0, 0, -180u"°"]),
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

YouBot(;pathIndexOffset) = Model(
    pathIndexOffset2 = pathIndexOffset,
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
        axis=axisLink, phi = Var(init = getRefPathInitPosition(referencePath1, 1+pathIndexOffset))),
    rev2 = RevoluteWithFlange(obj1 = :(link2.obj1), obj2 = :(link2.body),
        axis=axisLink, phi = Var(init = getRefPathInitPosition(referencePath1, 2+pathIndexOffset))),
    rev3 = RevoluteWithFlange(obj1 = :(link3.obj1), obj2 = :(link3.body),
        axis=axisLink, phi = Var(init = getRefPathInitPosition(referencePath1, 3+pathIndexOffset))),
    rev4 = RevoluteWithFlange(obj1 = :(link4.obj1), obj2 = :(link4.body),
        axis=axisLink, phi = Var(init = getRefPathInitPosition(referencePath1, 4+pathIndexOffset))),
    rev5 = RevoluteWithFlange(obj1 = :(link5.obj1), obj2 = :(link5.body),
        axis=axisLink, phi = Var(init = getRefPathInitPosition(referencePath1, 5+pathIndexOffset))),

    prism = PrismaticWithFlange(obj1 = :(gripper.gripper_right_finger_a), obj2 = :(gripper.gripper_left_finger_a),
        axis=axisGripper, s = Var(init = getRefPathInitPosition(referencePath1, 6+pathIndexOffset)) ),

    servo1 = Servo,
    servo2 = Servo,
    servo3 = Servo,
    servo4 = Servo,
    servo5 = Servo,
    servo6 = ServoTrans,

    equations=:[
        refPath = calculateRobotMovement(getReferencePath(), instantiatedModel),
        servo1.refLoadAngle = getRefPathPosition(refPath, 1+pathIndexOffset2),
        servo2.refLoadAngle = getRefPathPosition(refPath, 2+pathIndexOffset2),
        servo3.refLoadAngle = getRefPathPosition(refPath, 3+pathIndexOffset2),
        servo4.refLoadAngle = getRefPathPosition(refPath, 4+pathIndexOffset2),
        servo5.refLoadAngle = getRefPathPosition(refPath, 5+pathIndexOffset2),
        servo6.refLoadPos   = getRefPathPosition(refPath, 6+pathIndexOffset2)
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

YouBot1 = YouBot(pathIndexOffset=0)
YouBot2 = YouBot(pathIndexOffset=6)

Scenario = Model(
    gravField = UniformGravityField(g=9.81, n=[0,0,-1]),
    world = Object3D(feature=Scene(gravityField=:gravField, visualizeFrames=false, nominalLength=tableX,
    #mprTolerance = 1.0e-14,
        animationFile="YouBotsGripping.json",
        enableContactDetection=true, maximumContactDamping=1000, elasticContactReductionFactor=1e-3)),
#   worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.2))),

    table = Table,
    ground = Ground,

    sphere = Object3D(
        feature=Solid(shape=Sphere(diameter=diameter), visualMaterial=vmatGrey, solidMaterial="DryWood", collision=true)),

    sphereJoint = FreeMotion(obj1=:world, obj2=:sphere, r=Var(init=[-0.78, 0.0, 0.1792])),

    youbot1 = YouBot1,
    youbot2 = YouBot2
)

modelParameters = Map(
    youbot1 = Map(
        base = Map(
            orientation = [0.0, 0.0, 0.0],
            position = [-0.569, 0.0, 0.084],
        ),
        servo1 = servoParameters1,
        servo2 = servoParameters2,
        servo3 = servoParameters3,
        servo4 = servoParameters4,
        servo5 = servoParameters5,
        servo6 = servoParameters6
    ),

    youbot2 = Map(
        base = Map(
            orientation = [0.0, 0.0, -pi/2],
            position = [0.0, 0.575, 0.084],
        ),
        servo1 = servoParameters1,
        servo2 = servoParameters2,
        servo3 = servoParameters3,
        servo4 = servoParameters4,
        servo5 = servoParameters5,
        servo6 = servoParameters6
    )
)


youbotModel = buildModia3D(Scenario) | modelParameters

youbot = @instantiateModel(youbotModel, unitless=true, logCode=false, log=false)

stopTime = 28.0
tolerance = 1e-8
if simplifiedContact
    if Sys.iswindows()
        requiredFinalStates = [0.005097050469777106, 0.790418385000708, 0.18419404938733336, 8.124310723700832e-17, 1.3027240998714e-16, 1.7725808315480977e-18, -3.9283414978925184, -0.39961584103637043, 3.225628121473559, 5.334838250706396e-15, 1.6157090817508103e-15, -5.194712147977695e-17, -5.035511551762052e-13, 5.038013291879718e-13, -6.423598600637583e-12, 6.424419857142242e-12, -1.466658222060818e-11, 1.466846815375979e-11, -9.012363354641088e-12, 9.013792637572867e-12, 9.030826935614824e-14, -9.031396553281621e-14, -3.847193934232432e-9, 0.24163532322181047, -0.0007949230986782002, -0.0009396161976613041, 3.1416225034160644e-10, -1.0373745229148803e-8, 0.0009999997883393767, 2.1161314082305064e-10, 4.760744012058533e-7, -4.758374211789597e-7, -4.541752290620761e-6, 4.542279900016498e-6, -8.52918695583147e-6, 8.530366077103486e-6, -5.053572781455762e-6, 5.054434116771785e-6, 3.5345552644064696e-8, -3.5348994333510725e-8, 0.003640316875415476, 0.20692216784275164, -0.04258208192239307, -0.018517910195505076, 0.000122954970096428, -0.0014253599087014034, 0.0009709176808302505, 2.9075922535018736e-5]
    else
        requiredFinalStates = [0.005097020576287002, 0.7904180034030823, 0.18419404938635842, 5.390417346742597e-17, -8.673499922502523e-17, -4.513753382934421e-16, -1.4079324544735619, 1.052918908203659, 0.3595206321093979, 5.4714221571263096e-17, 3.1619944255194954e-16, 3.7699404920898335e-15, -5.035300902448364e-13, 5.037802702654452e-13, -6.4237539325379105e-12, 6.4245753715109354e-12, -1.4666997371985055e-11, 1.4668883352584913e-11, -9.01250411002059e-12, 9.013933428106989e-12, 9.03271424946133e-14, -9.033283879380805e-14, -3.847032869719696e-9, 0.2416353232206233, -0.0007949231007121495, -0.0009396161981508509, 3.142279096709202e-10, -1.037445285596054e-8, 0.0009999997883249386, 2.1162757445055148e-10, 4.7610222924797003e-7, -4.758650606672954e-7, -4.541709673952366e-6, 4.542237324673783e-6, -8.529098662828723e-6, 8.530277819203257e-6, -5.053533224614732e-6, 5.054394550825199e-6, 3.5345150117792754e-8, -3.534859176964157e-8, 0.003640529611895304, 0.20692249357311396, -0.04258164933868417, -0.01851777260611393, 0.0001229535698218701, -0.0014252363360592198, 0.0009709202021459315, 2.90734017873674e-5]
    end
else
    requiredFinalStates = [0.005098596727514149, 0.7904159015385256, 0.1791342195707778, -1.1808457144007462e-10, -1.88763357133662e-9, 8.608177052460935e-14, -1.5627420768515174, 1.1910702824997361, 0.5304431666324095, 2.4640818942197007e-8, -1.4491090645065959e-8, 6.037552152197727e-8, -5.269643935234176e-13, 5.272139874305713e-13, -6.420034932436889e-12, 6.420854829588564e-12, -1.4631919047149923e-11, 1.463380141274083e-11, -8.989367532219111e-12, 8.990793910036479e-12, 7.01557712829279e-14, -7.016147036815077e-14, -4.026169255817747e-9, 0.24163532324903933, -0.0007949229288464374, -0.000939616117669137, 2.4405169933839243e-10, -1.0329058212887232e-8, 0.000999999789251148, 2.107015735905466e-10, 4.7377460108628524e-7, -4.7353745252266476e-7, -4.541835528743539e-6, 4.542363196555878e-6, -8.529395514275828e-6, 8.530574712207511e-6, -5.054073275334597e-6, 5.054934647304243e-6, 3.5472140130848604e-8, -3.547558188477468e-8, 0.0036227372301417533, 0.20692153169511313, -0.04258310367908495, -0.018519651281104485, 0.0001233953677379415, -0.0014214723892361516, 0.0009709969998011199, 2.8996621358862662e-5]
end
simulate!(youbot, stopTime=stopTime, tolerance=tolerance, log=true, logStates=false, requiredFinalStates=requiredFinalStates)

#=
simulate!(youbot, stopTime=0.1, tolerance=tolerance, log=false)
using Profile
Profile.clear()
@profile simulate!(youbot, stopTime=14.0, tolerance=tolerance, log=true, logStates=true)
open("YouBotsGripping_profile.txt", "w") do io
    Profile.print(io, format=:flat, sortedby=:overhead)
end
=#

@usingModiaPlot
plot(youbot, ["sphereJoint.r"], figure=1)

end
