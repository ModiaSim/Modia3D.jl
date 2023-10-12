module YouBotsGripping

using  Modia3D

include("$(Modia3D.modelsPath)/Blocks.jl")
include("$(Modia3D.modelsPath)/Electric.jl")
include("$(Modia3D.modelsPath)/Rotational.jl")
include("$(Modia3D.modelsPath)/Translational.jl")


# some constants
simplifiedContact = false  # use boxes instead of meshes for finger contact

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
nullRot = [0, 0, 0]

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

#### ----------- Robot Program ------------------
initPosition = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

function robotProgram(robotActions)
    addReferencePath(robotActions,
    names =    ["bot1angle1", "bot1angle2", "bot1angle3", "bot1angle4", "bot1angle5", "bot1gripper", "bot2angle1", "bot2angle2", "bot2angle3", "bot2angle4", "bot2angle5", "bot2gripper"],
    position = initPosition,
    v_max =    [2.68512, 2.68512, 4.8879, 5.8997, 5.8997, 2.0, 2.68512, 2.68512, 4.8879, 5.8997, 5.8997, 2.0],
    a_max =    [1.5, 1.5, 1.5, 1.5, 1.5, 0.5, 1.5, 1.5, 1.5, 1.5, 1.5, 0.5])

    ptpJointSpace(robotActions,
    [
        0.0  0.0    0.0       0.0    0.0  diameter        0.0  0.0    0.0       0.0   0.0  diameter;
        pi   pi/4   pi/4      0.0    0.0  diameter+0.01   0.0  0.0    0.0       0.0   0.0  diameter;
        pi   pi/4   pi/4      1.057  0.0  diameter+0.01   0.0  0.0    0.0       0.0   0.0  diameter;
        pi   pi/4   pi/4      1.057  0.0  diameter-0.002  0.0  0.0    0.0       0.0   0.0  diameter;
        pi   pi/4   pi/4      0.0    0.0  diameter-0.002  0.0  0.0    0.0       0.0   0.0  diameter;
        0.0  0.0    pi/2-0.2  0.0    0.0  diameter-0.002  0.0  0.0    0.0       0.0   0.0  diameter;
        0.0  0.2    pi/2-0.2  0.0    0.0  diameter-0.002  0.0  0.0    0.0       0.0   0.0  diameter;
        0.0  0.2    pi/2-0.2  0.0    0.0  diameter+0.01   0.0  0.0    0.0       0.0   0.0  diameter;
        0.0  0.0    pi/2      0.0    0.0  diameter+0.01   0.0  0.0    0.0       0.0   0.0  diameter;
        0.0  0.0    0.0       0.0    0.0  0.001           0.0  0.0    0.0       0.0   0.0  diameter;
        0.0  0.0    0.0       0.0    0.0  0.001           0.0  0.0    pi/2      0.0   0.0  diameter+0.01;
        0.0  0.0    0.0       0.0    0.0  0.001           0.0  0.3    pi/2-0.3  0.0   0.0  diameter+0.01;
        0.0  0.0    0.0       0.0    0.0  0.001           0.0  0.3    pi/2-0.3  0.0   0.0  diameter-0.002;
        0.0  0.0    0.0       0.0    0.0  0.001           0.0  0.0    pi/2-0.3  0.0   0.0  diameter-0.002;
        0.0  0.0    0.0       0.0    0.0  0.001           pi   pi/4   pi/4      0.0   0.0  diameter-0.002;
        0.0  0.0    0.0       0.0    0.0  0.001           pi   pi/4   pi/4      1.0   0.0  diameter-0.002;
        0.0  0.0    0.0       0.0    0.0  0.001           pi   pi/4   pi/4      1.0   0.0  diameter+0.01;
        0.0  0.0    0.0       0.0    0.0  0.001           pi   pi/4   pi/4      0.0   0.0  diameter+0.01;
        0.0  0.0    0.0       0.0    0.0  0.001           0.0  0.0    0.0       0.0   0.0  0.01
    ])

    return nothing
end

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
                    initRefPos = initPosition[1],
                    trans = translation1,
                    rota = Par(value = :(rotation1))
)

linkParameters2 = Map(
                    parent1 = Par(value = :(link1.obj2)),
                    featureBody = featureBody2,
                    m = m2,
                    initRefPos = initPosition[2],
                    trans = translation2,
                    rota = Par(value = :(rotation2))
)

linkParameters3 = Map(
                    parent1 = Par(value = :(link2.obj2)),
                    featureBody = featureBody3,
                    m = m3,
                    initRefPos = initPosition[3],
                    trans = translation3,
                    rota = Par(value = :(rotation3))
)

linkParameters4 = Map(
                    parent1 = Par(value = :(link3.obj2)),
                    featureBody = featureBody4,
                    m = m4,
                    initRefPos = initPosition[4],
                    trans = translation4,
                    rota = Par(value = :(nullRot))
)

linkParameters5 = Map(
                    parent1 = Par(value = :(link4.obj2)),
                    featureBody = featureBody5,
                    m = m5,
                    initRefPos = initPosition[5],
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

YouBot(worldName;pathIndexOffset) = Model(
    pathIndexOffset2 = pathIndexOffset,
    worldName = Par(worldName),
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
        axis=axisLink, phi = Var(init = initPosition[1+pathIndexOffset])),
    rev2 = RevoluteWithFlange(obj1 = :(link2.obj1), obj2 = :(link2.body),
        axis=axisLink, phi = Var(init = initPosition[2+pathIndexOffset])),
    rev3 = RevoluteWithFlange(obj1 = :(link3.obj1), obj2 = :(link3.body),
        axis=axisLink, phi = Var(init = initPosition[3+pathIndexOffset])),
    rev4 = RevoluteWithFlange(obj1 = :(link4.obj1), obj2 = :(link4.body),
        axis=axisLink, phi = Var(init = initPosition[4+pathIndexOffset])),
    rev5 = RevoluteWithFlange(obj1 = :(link5.obj1), obj2 = :(link5.body),
        axis=axisLink, phi = Var(init = initPosition[5+pathIndexOffset])),

    prism = PrismaticWithFlange(obj1 = :(gripper.gripper_right_finger_a), obj2 = :(gripper.gripper_left_finger_a),
        axis=axisGripper, s = Var(init = initPosition[6+pathIndexOffset]) ),

    servo1 = Servo,
    servo2 = Servo,
    servo3 = Servo,
    servo4 = Servo,
    servo5 = Servo,
    servo6 = ServoTrans,

    modelActions = ModelActions(world=:world, actions=robotProgram),
    currentAction = Var(hideResult=true),

    equations=:[
        currentAction = executeActions(modelActions),
        servo1.refLoadAngle = getRefPathPosition(currentAction, 1+pathIndexOffset2),
        servo2.refLoadAngle = getRefPathPosition(currentAction, 2+pathIndexOffset2),
        servo3.refLoadAngle = getRefPathPosition(currentAction, 3+pathIndexOffset2),
        servo4.refLoadAngle = getRefPathPosition(currentAction, 4+pathIndexOffset2),
        servo5.refLoadAngle = getRefPathPosition(currentAction, 5+pathIndexOffset2),
        servo6.refLoadPos   = getRefPathPosition(currentAction, 6+pathIndexOffset2)
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

YouBot1 = YouBot("world",pathIndexOffset=0)
YouBot2 = YouBot("world",pathIndexOffset=6)

Scenario = Model3D(
    gravField = UniformGravityField(g=9.81, n=[0,0,-1]),
    world = Object3D(feature=Scene(gravityField=:gravField, visualizeFrames=false, nominalLength=2*tableX,
    #mprTolerance = 1.0e-14,
        animationFile="YouBotsGripping.json",
        enableContactDetection=true, maximumContactDamping=1000, elasticContactReductionFactor=1e-3)),
#   worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.2))),

    table = Table,
    ground = Ground,

    sphere = Object3D(parent=:world, fixedToParent=false,
                      translation=[-0.78, 0.0, 0.1792],
                      feature=Solid(shape=Sphere(diameter=diameter),
                      visualMaterial=vmatGrey,
                      solidMaterial="DryWood",
                      collision=true)),

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


youbotModel = Scenario | modelParameters

youbot = @instantiateModel(youbotModel, unitless=true, logCode=false, log=false)

stopTime = 28.0
tolerance = 1e-7
if simplifiedContact
    requiredFinalStates = [6.626036032444092e-14, -6.625937921803225e-14, -1.2190637002942868e-11, 1.219255639304887e-11, -2.7736194512872756e-11, 2.7739781453199694e-11, -1.721911423238536e-11, 1.722185259213342e-11, 3.7964989396855374e-14, -3.79626957385797e-14, 5.065018594629937e-10, -0.0016527210478103193, -0.0007949871312746726, -0.0009396447443010905, 1.3208833922710134e-10, -1.9522808771237002e-8, 0.0009999996016665366, 3.9824410340217393e-10, 1.7593355332875133e-6, -1.7597152222807876e-6, -1.121428727254605e-5, 1.1215595105886919e-5, -1.6104289373934256e-5, 1.610661215757139e-5, -9.610673174743719e-6, 9.612450911470863e-6, 1.779037161133729e-8, -1.779016601717162e-8, 0.01344834037824163, -0.08736378440155342, -0.0796938518093969, -0.0343688377368504, 6.189342032026106e-5, -0.0027012995965528266, 0.009944884090430062, 5.510242274722822e-5, 0.0050971594906382, 0.7904184475839454, 0.1841940692121035, 1.2620043370105517e-14, -2.303657866562073e-14, -4.007573135451641e-16, -1.393228415991675, 1.0553987893717447, 0.34669947766717063, 8.356834840408132e-13, 4.62039252248169e-13, 3.732380627962388e-25]
else
    requiredFinalStates = [6.615347137075564e-14, -6.615250228105601e-14, -1.2141774825203918e-11, 1.214368601093644e-11, -2.762732806724143e-11, 2.763089935806184e-11, -1.7090602502993377e-11, 1.7093327189090367e-11, 4.1323070097982075e-14, -4.132078537334784e-14, 5.056847063757215e-10, -0.00165272067437182, -0.0007949865979073207, -0.0009396442972573212, 1.4377107027789479e-10, -1.9295497475123062e-8, 0.0009999996063044807, 3.936072042374753e-10, 1.759346467131401e-6, -1.7601182325695458e-6, -1.1214294975053435e-5, 1.121549794252837e-5, -1.610430978865418e-5, 1.610652565494e-5, -9.610682642543182e-6, 9.61245449683454e-6, 1.7795653875697086e-8, -1.7795445108591726e-8, 0.013448363228336462, -0.08736385936052293, -0.07969396231903642, -0.034368871026250296, 6.191179728632753e-5, -0.002676698247122822, 0.009945386028546352, 5.4601183528503756e-5, 0.0050911069427693615, 0.790390841026788, 0.179131874644137, 1.646332464054688e-11, -1.015633500498424e-11, -1.1037818715887114e-13, -1.343513801214781, 0.8913349750469441, 0.24005686371447058, 3.8806501442000056e-10, 6.288778376919387e-10, -2.1369966988566976e-18]
end
simulate!(youbot, stopTime=stopTime, tolerance=tolerance, requiredFinalStates_atol=0.002, log=true, logStates=false, requiredFinalStates=requiredFinalStates)

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
plot(youbot, ["sphere.translation"], figure=1)

end
