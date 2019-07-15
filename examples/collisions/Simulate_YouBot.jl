module Simulate_YouBot

using  Modia3D
import Modia3D.ModiaMath
using  Modia3D.ModiaMath.Unitful

vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.4)    # material of SolidFileMesh

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


# Damper
@forceElement Damper(; d=1.0) begin
    w   = ModiaMath.RealScalar("w",   causality=ModiaMath.Input,  numericType=ModiaMath.WR)
    tau = ModiaMath.RealScalar("tau", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeTorque(damper::Damper, sim::ModiaMath.SimulationState)
    damper.tau.value = -damper.d*damper.w.value
end


# Controller
@forceElement P_PI_Controller(; k1=1.0, k2=1.0, T2=0.01, freqHz=0.5, A=1.0) begin
    PI_x    = ModiaMath.RealScalar("PI_x"   , start=0.0, info="State of PI controller", numericType=ModiaMath.XD_EXP)
    PI_derx = ModiaMath.RealScalar("PI_derx", start=0.0, info="= der(PI_x)"           , numericType=ModiaMath.DER_XD_EXP, integral=PI_x)
    sine_y  = ModiaMath.RealScalar("sine_y" , start=0.0, info="= sin(2*pi*f*time)"    , numericType=ModiaMath.WR)
    phi = ModiaMath.RealScalar("phi", causality=ModiaMath.Input,  numericType=ModiaMath.WR)
    w   = ModiaMath.RealScalar("w",   causality=ModiaMath.Input,  numericType=ModiaMath.WR)
    tau = ModiaMath.RealScalar("tau", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeTorque(c::P_PI_Controller, sim::ModiaMath.SimulationState)
    c.sine_y.value  = c.A*sin(2*pi*c.freqHz*sim.time)
    gain_y          = c.k1*(c.sine_y.value - c.phi.value)
    PI_u            = gain_y - c.w.value
    c.PI_derx.value = PI_u/c.T2
    c.tau.value = c.k2*(c.PI_x.value + PI_u)
end


@signal Sine(;A=1.5, freqHz = 0.5) begin
   y = ModiaMath.RealScalar(causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::Sine, sim::ModiaMath.SimulationState)
    signal.y.value  = signal.A*sin(2*pi*signal.freqHz*sim.time)
end


@assembly Link(frame_a; axis=3, R_a_rev=ModiaMath.NullRotation, r_rev_b=[0.0, 0.0, 0.0],
                r_rev_CM=[0.0, 0.0, 0.0], m=0.0, Ixx=0.0, Iyy=0.0, Izz=0.0, fileMesh="",
                d=0.0, k1=10.0, k2=10.0, T2=0.01, freqHz=0.1, A=1.0) begin
    rev_a   = Object3D(frame_a, R=R_a_rev)
    rev_b   = Object3D(Solid(nothing,MassProperties(m=m, rCM=r_rev_CM, Ixx=Ixx, Iyy=Iyy, Izz=Izz),vmat1), visualizeFrame=true)
    rev     = Revolute(rev_a, rev_b)
    frame_vis = Object3D(rev_b, FileMesh(fileMesh), R=ModiaMath.rot1(90u"°"))
    frame_b = Object3D(rev_b, r=r_rev_b)

    # Kinematic movement of joint
    sig         = Sine(A=A,freqHz=freqHz)
    sig_adaptor = Modia3D.SignalToFlangeAngle(sig.y)
    Modia3D.connect(sig_adaptor, rev)

#=
    # Damping in the joint
    damper         = Damper(d=d)
    damper_adaptor = Modia3D.AdaptorForceElementToFlange(w=damper.w, tau=damper.tau)
    Modia3D.connect(damper_adaptor, rev)

    # P-PI controller in the joint
    controller         = P_PI_Controller(k1=k1, k2=k2, T2=T2, freqHz=freqHz, A=A)
    controller_adaptor = Modia3D.AdaptorForceElementToFlange(phi=controller.phi, w=controller.w, tau=controller.tau)
    Modia3D.connect(controller_adaptor, rev)
=#
end

@assembly Gripper(frame_a) begin
    frame1                   = Object3D(frame_a, R=ModiaMath.rot3(-180u"°"))
    gripper_base_frame       = Object3D(frame1,Solid(nothing,MassProperties(m=0.199, Ixx=0.0002324, Iyy=0.0003629, Izz=0.0002067),nothing), visualizeFrame=true)
    gripper_base_frame_vis   = Object3D(frame1, FileMesh(gripper_base_frame_obj), R=ModiaMath.rot1(90u"°"))

    gripper_left_finger      = Object3D(gripper_base_frame, Solid(nothing,MassProperties(m=0.010),nothing), r=[0, 0.0082,0])
    gripper_left_finger_vis  = Object3D(gripper_left_finger, FileMesh(gripper_left_finger_obj), R=ModiaMath.rot1(90u"°"))

    gripper_right_finger     = Object3D(gripper_base_frame, Solid(nothing,MassProperties(m=0.010),nothing), r=[0,-0.0082,0])
    gripper_right_finger_vis = Object3D(gripper_right_finger, FileMesh(gripper_right_finger_obj), R=ModiaMath.rot1(90u"°"))
end

@assembly Base(world) begin
    base_frame            = Object3D(world, Solid(nothing,MassProperties(m=19.803),nothing), r=[0,0,0.084])
    base_frame_vis        = Object3D(base_frame, FileMesh(base_frame_obj), R=ModiaMath.rot1(90u"°"))

    plate                 = Object3D(base_frame, Solid(nothing,MassProperties(m=2.397),nothing), r=[-0.159,0,0.046])
    plate_vis             = Object3D(plate, FileMesh(plate_obj), R=ModiaMath.rot1(90u"°"))

    front_right_wheel     = Object3D(base_frame, Solid(nothing,MassProperties(m=1.4),nothing), r=[0.228, -0.158, -0.034])
    front_right_wheel_vis = Object3D(front_right_wheel, FileMesh(front_right_wheel_obj), R=ModiaMath.rot1(90u"°"))

    front_left_wheel     = Object3D(base_frame, Solid(nothing,MassProperties(m=1.4),nothing), r=[0.228, 0.158, -0.034])
    front_left_wheel_vis = Object3D(front_left_wheel, FileMesh(front_left_wheel_obj), R=ModiaMath.rot1(90u"°"))

    back_right_wheel     = Object3D(base_frame, Solid(nothing,MassProperties(m=1.4),nothing), r=[-0.228, -0.158, -0.034])
    back_right_wheel_vis = Object3D(back_right_wheel, FileMesh(back_right_wheel_obj), R=ModiaMath.rot1(90u"°"))

    back_left_wheel      = Object3D(base_frame, Solid(nothing,MassProperties(m=1.4),nothing), r=[-0.228, 0.158, -0.034])
    back_left_wheel_vis  = Object3D(back_left_wheel, FileMesh(back_left_wheel_obj), R=ModiaMath.rot1(90u"°"))
end


@assembly YouBot begin
    world              = Object3D(visualizeFrame=true)
    base               = Base(world)
    arm_base_frame     = Object3D(base.base_frame, Solid(nothing, MassProperties(m=0.961), vmat1), r=[0.143, 0.0, 0.046], visualizeFrame=true)
    arm_base_frame_vis = Object3D(arm_base_frame, FileMesh(arm_base_frame_obj), R=ModiaMath.rot1(90u"°"))
    armBase_b          = Object3D(arm_base_frame, r=[0.024, 0, 0.115])
    link1   = Link(armBase_b, R_a_rev = ModiaMath.rot1(180u"°"), r_rev_b=[0.033,0,0],
                   r_rev_CM=[0.0, 0.0, 0.0], m=1.390, Ixx=0.0029525, Iyy=0.0060091, Izz=0.0058821, fileMesh=arm_joint_1_obj)
    link2   = Link(link1.frame_b, R_a_rev = ModiaMath.rot123(90u"°", 0.0,-90u"°"), r_rev_b=[0.155,0,0],
                   r_rev_CM=[0.0, 0.0, 0.0], m=1.318, Ixx=0.0031145, Iyy=0.0005843, Izz=0.0031631, fileMesh=arm_joint_2_obj)
    link3   = Link(link2.frame_b, R_a_rev = ModiaMath.rot3(-90u"°"), r_rev_b=[0,0.135,0],
                   r_rev_CM=[0.0, 0.0, 0.0], m=0.821, Ixx=0.00172767, Iyy=0.0005843, Izz=0.0018468, fileMesh=arm_joint_3_obj)
    link4   = Link(link3.frame_b, r_rev_b=[0,0.11316,0],
                   r_rev_CM=[0.0, 0.0, 0.0], m=0.769, Ixx=0.0006764, Iyy=0.0010573, Izz=0.000661, fileMesh=arm_joint_4_obj)
    link5   = Link(link4.frame_b, R_a_rev = ModiaMath.rot1(-90u"°"), r_rev_b=[0,0,0.05716],
                   r_rev_CM=[0.0, 0.0, 0.0], m=0.687, Ixx=0.0001934, Iyy=0.0001602, Izz=0.0000689, fileMesh=arm_joint_5_obj)
    gripper = Gripper(link5.frame_b)
end

gravField = UniformGravityField(g=9.81, n=[0,0,-1])
youBot    = YouBot(sceneOptions=SceneOptions(gravityField=gravField, visualizeFrames=false,
                                             defaultFrameLength=0.1, enableContactDetection=false))

# Modia3D.visualizeAssembly!(youBot)
model  = Modia3D.SimulationModel( youBot, analysis=ModiaMath.KinematicAnalysis )
result = ModiaMath.simulate!(model, stopTime=5.0, log=true, tolerance=1e-4)
end