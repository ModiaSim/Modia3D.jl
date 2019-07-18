module Simulate_YouBot

using  Modia3D
import Modia3D.ModiaMath
using  Modia3D.ModiaMath.Unitful

vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.0)    # material of SolidFileMesh
vmat2 = Modia3D.Material(color="Grey"      , transparency=0.3)    # material of work piece
vmat3 = Modia3D.Material(color="LightBlue" , transparency=0.3)    # material of table

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

# Drive train data
effectiveInertia(J, gearRatio) = J*gearRatio^2

const motorInertia1 = 0.0000135 + 0.000000409
const gearRatio1    = 156.0
const J1            = effectiveInertia(motorInertia1, gearRatio1)

const motorInertia2 = 0.0000135 + 0.000000409
const gearRatio2    = 156.0
const J2            = effectiveInertia(motorInertia2, gearRatio2)

const motorInertia3 = 0.0000135 + 0.000000071
const gearRatio3    = 100.0
const J3            = effectiveInertia(motorInertia3, gearRatio3)

const motorInertia4 = 0.00000925 + 0.000000071
const gearRatio4    = 71.0
const J4            = effectiveInertia(motorInertia4, gearRatio4)

const motorInertia5 = 0.0000035 + 0.000000071
const gearRatio5    = 71.0
const J5            = effectiveInertia(motorInertia5, gearRatio5)


println("")
println("J1 = $J1, gearRatio1 = $gearRatio1")
println("J2 = $J2, gearRatio2 = $gearRatio2")
println("J3 = $J3, gearRatio3 = $gearRatio3")
println("J4 = $J4, gearRatio4 = $gearRatio4")
println("J5 = $J5, gearRatio5 = $gearRatio5\n")


# Damper
@forceElement Damper(; d=1.0) begin
    w   = ModiaMath.RealScalar("w",   causality=ModiaMath.Input,  numericType=ModiaMath.WR)
    tau = ModiaMath.RealScalar("tau", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeTorque(damper::Damper, sim::ModiaMath.SimulationState)
    damper.tau.value = -damper.d*damper.w.value
end


# Controller
const ptp_path1 = PTP_path(["angle1", "angle2", "angle3", "angle4", "angle5", "gripper"],
                           positions = [0.0  0.0  pi/2 0.0 0.0 0.0;
                                        pi   pi/2 0.0  1.3 1.4 0.0;
                                        pi/2 0.0  0.5  0.5 0.5 0.0;
                                        0.0  0.0  pi/2 0.0 0.0 0.0;
                                        0.0  0.0  pi/2 0.0 0.0 0.04;
                                        0.0  0.0  pi/2 0.0 0.0 0.041;],
                           startTime = 0.0,
                           v_max = [2.68512, 2.68512, 4.8879, 5.8997, 5.8997, 1.0],
                           a_max = [1.5, 1.5, 1.5, 1.5, 1.5, 0.5])

const ptp_path2 = PTP_path(["angle1", "angle2", "angle3", "angle4", "angle5", "gripper"],
                           positions = [0.0  0.0 0.0  0.0 0.0 0.0;
                                        0.0  0.0 0.0  0.0 0.0 0.04;
                                        0.0  0.0 pi/2 0.0 0.0 0.04;
                                        0.0  0.0 pi/2 0.0 0.0 0.0380703;    # no contact 0.3808; 0.3807 fails; 0.380703: a lot of events, but box stays on table
                                        0.0 -pi/4 2.5*pi/4 0.0 0.0 0.0380703;
                                        0.0  0.0 0.0  0.0 0.0 0.03807],
                           startTime = 0.0,
                           v_max = [2.68512, 2.68512, 4.8879, 5.8997, 5.8997, 1.0],
                           a_max = [1.5, 1.5, 1.5, 1.5, 1.5, 0.5])

const ptp_path = ptp_path2


plotPath(ptp_path, figure=1, ntime=501)
plotPath(ptp_path, names=["gripper"], figure=2, ntime=501)

@forceElement P_PI_Controller(; k1=1.0, k2=10.0, T2=0.01, gearRatio=1.0, phi_ref_name="UnknownName") begin
    phi_ref_index = getIndex(ptp_path, phi_ref_name)
    PI_x    = ModiaMath.RealScalar("PI_x"   , start=0.0, info="State of PI controller", numericType=ModiaMath.XD_EXP)
    PI_derx = ModiaMath.RealScalar("PI_derx", start=0.0, info="= der(PI_x)"           , numericType=ModiaMath.DER_XD_EXP, integral=PI_x)
    phi_ref = ModiaMath.RealScalar(phi_ref_name                ,  numericType=ModiaMath.WR)
    phi = ModiaMath.RealScalar("phi", causality=ModiaMath.Input,  numericType=ModiaMath.WR)
    w   = ModiaMath.RealScalar("w",   causality=ModiaMath.Input,  numericType=ModiaMath.WR)
    tau = ModiaMath.RealScalar("tau", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeTorque(c::P_PI_Controller, sim::ModiaMath.SimulationState)
    gearRatio       = c.gearRatio
    c.phi_ref.value = getPosition(ptp_path, c.phi_ref_index, sim.time)
    gain_y          = c.k1*(c.phi_ref.value - c.phi.value)*gearRatio
    PI_u            = gain_y - c.w.value*gearRatio
    c.PI_derx.value = PI_u/c.T2
    c.tau.value = gearRatio*c.k2*(c.PI_x.value + PI_u)
end


@assembly ControlledRevolute(frame_a, frame_b; axis=3, J=1.0,
                             d=1.0, k1=1.0, k2=10.0, T2=0.01, gearRatio=1.0, phi_ref_name="UnknownName") begin

    rev = Revolute(frame_a, frame_b, phi_start=getPosition(ptp_path,getIndex(ptp_path, phi_ref_name), 0.0), J=J, axis=axis)

    # Damping in the joint
    damper         = Damper(d=d)
    damper_adaptor = Modia3D.AdaptorForceElementToFlange(w=damper.w, tau=damper.tau)
    Modia3D.connect(damper_adaptor, rev)

    # P-PI controller in the joint
    controller         = P_PI_Controller(k1=k1, k2=k2, T2=T2, gearRatio=gearRatio, phi_ref_name=phi_ref_name)
    controller_adaptor = Modia3D.AdaptorForceElementToFlange(phi=controller.phi, w=controller.w, tau=controller.tau)
    Modia3D.connect(controller_adaptor, rev)
end


@assembly ControlledPrismatic(frame_a, frame_b; axis=3,
                              d=0.0, k1=1.0, k2=10.0, T2=0.01, gearRatio=1.0, s_ref_name="UnknownName") begin

    prism = Prismatic(frame_a, frame_b, s_start=getPosition(ptp_path,getIndex(ptp_path, s_ref_name), 0.0), axis=axis)

    # Damping in the joint
    damper         = Damper(d=d)
    damper_adaptor = Modia3D.AdaptorForceElementToPFlange(v=damper.w, f=damper.tau)
    Modia3D.connect(damper_adaptor, prism)

    # P-PI controller in the joint
    controller         = P_PI_Controller(k1=k1, k2=k2, T2=T2, gearRatio=gearRatio, phi_ref_name=s_ref_name)
    controller_adaptor = Modia3D.AdaptorForceElementToPFlange(s=controller.phi, v=controller.w, f=controller.tau)
    Modia3D.connect(controller_adaptor, prism)
end


@assembly Link(frame_a; axis=3, J=1.0,
               d=1.0, k1=50.0, k2=50.0, T2=0.002, gearRatio=1.0, phi_ref_name="UnknownName",
               fileMesh="", m=0.0, r_rev_b=[0.0, 0.0, 0.0], R_a_rev=ModiaMath.NullRotation) begin
    rev_a   = Object3D(frame_a, R=R_a_rev, visualizeFrame=false)
    rev_b   = Object3D(Solid(SolidFileMesh(fileMesh),m,vmat1))
    rev     = ControlledRevolute(rev_a, rev_b, axis=axis, J=J,
                                 d=d, k1=k1, k2=k2, T2=T2, gearRatio=gearRatio, phi_ref_name=phi_ref_name)
    frame_b = Object3D(rev_b, r=r_rev_b, visualizeFrame=false)
end

@assembly Gripper(frame_a, right_finger_offset=0.019,
                  d=0.0, k1=50.0, k2=50.0, T2=0.002, gearRatio=1.0) begin
    frame1                   = Object3D(frame_a, R=ModiaMath.rot3(-180u"°"))
    gripper_base_frame       = Object3D(frame1,Solid(SolidFileMesh(gripper_base_frame_obj),0.199,vmat1), visualizeFrame=false)
    gripper_left_finger_a    = Object3D(visualizeFrame=true)
    gripper_left_finger      = Object3D(gripper_left_finger_a, Solid(SolidFileMesh(gripper_left_finger_obj),0.010,vmat1,contactMaterial="DryWood"), r=[0, 0.0082,0])
    gripper_right_finger_a   = Object3D(gripper_base_frame, r=[0,-right_finger_offset,0])
    gripper_right_finger     = Object3D(gripper_right_finger_a, Solid(SolidFileMesh(gripper_right_finger_obj),0.010,vmat1,contactMaterial="DryWood"), r=[0,-0.0082,0])
    prism = ControlledPrismatic(gripper_right_finger_a, gripper_left_finger_a, axis=2,
                                 d=d, k1=k1, k2=k2, T2=T2, gearRatio=gearRatio, s_ref_name="gripper")
end

@assembly Base(world) begin
    base_frame        = Object3D(world, Solid(SolidFileMesh(base_frame_obj),19.803,vmat1), r=[0,0,0.084])
    plate             = Object3D(base_frame, Solid(SolidFileMesh(plate_obj),MassProperties(m=2.397),vmat1), r=[-0.159,0,0.046])
    front_right_wheel = Object3D(base_frame, Solid(SolidFileMesh(front_right_wheel_obj),1.4,vmat1), r=[0.228, -0.158, -0.034])
    front_left_wheel  = Object3D(base_frame, Solid(SolidFileMesh(front_left_wheel_obj),1.4,vmat1) , r=[0.228, 0.158, -0.034])
    back_right_wheel  = Object3D(base_frame, Solid(SolidFileMesh(back_right_wheel_obj),1.4,vmat1) , r=[-0.228, -0.158, -0.034])
    back_left_wheel   = Object3D(base_frame, Solid(SolidFileMesh(back_left_wheel_obj),1.4,vmat1)  , r=[-0.228, 0.158, -0.034])
end

workpiece_Lx = 0.04
workpiece_Ly = 0.04
workpiece_Lz = 0.08

xPosTable = 0.71
tableX    = 0.4
tableY    = 0.3
tableZ    = 0.01
heigthLeg = 0.35
widthLeg  = 0.02
@assembly Table(world) begin
    solidPlate = Solid(SolidBox(tableX, tableY, tableZ, rsmall=0.0) , "DryWood", vmat1, contactMaterial="DryWood")
    plate = Object3D(world, solidPlate, fixed=true, r=[xPosTable , 0.0, heigthLeg], visualizeFrame=false)
    leg   = Solid(SolidBox(widthLeg, widthLeg, heigthLeg) , "DryWood", vmat3)
    leg1  = Object3D(plate, leg, fixed=true, r=[tableX/2 - widthLeg/2 , tableY/2 - widthLeg/2, -heigthLeg/2])
    leg2  = Object3D(plate, leg, fixed=true, r=[-tableX/2 + widthLeg/2 , tableY/2 - widthLeg/2, -heigthLeg/2])
    leg3  = Object3D(plate, leg, fixed=true, r=[tableX/2 - widthLeg/2 , -tableY/2 + widthLeg/2, -heigthLeg/2])
    leg4  = Object3D(plate, leg, fixed=true, r=[-tableX/2 + widthLeg/2 , -tableY/2 + widthLeg/2, -heigthLeg/2])
end


@assembly YouBot begin
    world           = Object3D(visualizeFrame=true)
    base            = Base(world)
    arm_base_frame  = Object3D(base.base_frame, Solid(SolidFileMesh(arm_base_frame_obj), 0.961, vmat1), r=[0.143, 0.0, 0.046], visualizeFrame=false)
    armBase_b       = Object3D(arm_base_frame, r=[0.024, 0, 0.115])
    link1   = Link(armBase_b    , J=J1, phi_ref_name="angle1", gearRatio=gearRatio1, fileMesh=arm_joint_1_obj, m=1.390, r_rev_b=[0.033,0,0], R_a_rev = ModiaMath.rot1(180u"°"))
    link2   = Link(link1.frame_b, J=J2, phi_ref_name="angle2", gearRatio=gearRatio2, fileMesh=arm_joint_2_obj, m=1.318, r_rev_b=[0.155,0,0], R_a_rev = ModiaMath.rot123(90u"°", 0.0,-90u"°"))
    link3   = Link(link2.frame_b, J=J3, phi_ref_name="angle3", gearRatio=gearRatio3, fileMesh=arm_joint_3_obj, m=0.821, r_rev_b=[0,0.135,0], R_a_rev = ModiaMath.rot3(-90u"°"))
    link4   = Link(link3.frame_b, J=J4, phi_ref_name="angle4", gearRatio=gearRatio4, fileMesh=arm_joint_4_obj, m=0.769, r_rev_b=[0,0.11316,0])
    link5   = Link(link4.frame_b, J=J5, phi_ref_name="angle5", gearRatio=gearRatio5, fileMesh=arm_joint_5_obj, m=0.687, r_rev_b=[0,0,0.05716], R_a_rev = ModiaMath.rot1(-90u"°"))
    gripper = Gripper(link5.frame_b)

    # Table + workpiece
    table     = Table(world)
    workpiece = Modia3D.ContactBox2(table.plate, scale=[workpiece_Lx, workpiece_Ly, workpiece_Lz], massProperties="DryWood",
                                   material=vmat2, contactMaterial="DryWood",
                                   r=[-tableX/2+1.2*workpiece_Lx/2, 0.0, workpiece_Lz/2+tableZ/2], visualizeFrame=false, fixed=false)
end

gravField = UniformGravityField(g=9.81, n=[0,0,-1])
youBot    = YouBot(sceneOptions=SceneOptions(gravityField=gravField, visualizeFrames=false,
                                             defaultFrameLength=0.1, enableContactDetection=true, elasticContactReductionFactor=1e-4))

#Modia3D.visualizeAssembly!(youBot)
model  = Modia3D.SimulationModel( youBot, maxNumberOfSteps=500 )
#result = ModiaMath.simulate!(model, stopTime=2.764, log=true, interval=0.001, tolerance=1e-5)
result = ModiaMath.simulate!(model, stopTime=6.0, log=true, interval=0.001, tolerance=1e-5)

ModiaMath.plot(result,[("link1.rev.controller.phi_ref", "link1.rev.rev.phi") ("link4.rev.controller.phi_ref", "link4.rev.rev.phi");
                       ("link2.rev.controller.phi_ref", "link2.rev.rev.phi") ("link5.rev.controller.phi_ref", "link5.rev.rev.phi");
                       ("link3.rev.controller.phi_ref", "link3.rev.rev.phi") ("gripper.prism.controller.phi_ref", "gripper.prism.prism.s")], figure=3 )

ModiaMath.plot(result, "workpiece.box.r[3]", figure=4)
end