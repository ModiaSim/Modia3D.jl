module Test_2Rev_ZylZ_BarX

using  Modia3D
import Modia3D.ModiaMath



# Solids
groundMaterial = Modia3D.Material(color="DarkGreen", transparency=0.5)
# massProperties = Modia3D.MassProperties()
vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)
vmat2 = Modia3D.Material(color="Red")

@forceElement Force(; d=1.0) begin
    w   = ModiaMath.RealScalar("w",   causality=ModiaMath.Input,  numericType=ModiaMath.WR)
    tau = ModiaMath.RealScalar("tau", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeTorque(damper::Force, sim::ModiaMath.SimulationState)
    damper.tau.value = damper.d
end



@assembly Box(; Lx = 1.0, Ly=Lx, Lz=3*Lx) begin
   frame0 = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBox(Lx,Ly,Lz), "Aluminium", vmat1))
   frame1 = Modia3D.Object3D(frame0; r=[0.0, 0.0, -Lz/2])
   frame2 = Modia3D.Object3D(frame0; r=[0.0, 0.0,  Lz/2])
   cyl    = Modia3D.Cylinder(Lx/5,Lz/7; material=vmat2)
   cyl1   = Modia3D.Object3D(frame1, cyl; visualizeFrame=false)
end


@assembly Bar(;Lx = 1.0, Ly=Lx, Lz=Ly) begin
   frame0 = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), "Aluminium", vmat1))
   frame1 = Modia3D.Object3D(frame0; r=[-Lx/2, 0.0, 0.0])
   frame2 = Modia3D.Object3D(frame0; r=[ Lx/2, 0.0, 0.0])
   cyl    = Modia3D.Cylinder(Ly/2,1.2*Ly; material=vmat2)
   cyl1   = Modia3D.Object3D(frame1, cyl; visualizeFrame=false)
   cyl2   = Modia3D.Object3D(frame2, cyl; visualizeFrame=false)
end

@assembly Cyl(;Dx = 0.1, Dy=Dx, Lz=4*Dx, axis=1) begin
   # Dx = , Lz =
   frame0 = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidCylinder(Dx,Lz), "Aluminium", vmat1))
   frame1 = Modia3D.Object3D(frame0; r=[0.0, 0.0, -Lz/2])
   frame2 = Modia3D.Object3D(frame0; r=[0.0, 0.0,  Lz/2])
   cyl    = Modia3D.Cylinder(Dx/5,Dx; material=vmat2)
   if axis == 1
      cyl1   = Modia3D.Object3D(frame1, cyl; R=ModiaMath.rot2(-pi/2), visualizeFrame=false)
   elseif axis == 2
      cyl1   = Modia3D.Object3D(frame1, cyl; R=ModiaMath.rot1(-pi/2), visualizeFrame=false)
   end
end

@assembly DoublePendulum(;Lx = 1.0) begin
   world = Modia3D.Object3D(Modia3D.CoordinateSystem(0.5*Lx))
   boxZ  = Box(Lx=Lx)
   cylinderX  = Cyl(Dx=Lx, axis = 1)
   cylinderY  = Cyl(Dx=Lx, axis = 2)
   rev1  = Modia3D.Revolute(world, boxZ.frame1; axis = 3)
   rev2  = Modia3D.Revolute(boxZ.frame2, cylinderX.frame1; axis = 1, phi_start = pi/2)
   rev3  = Modia3D.Revolute(cylinderX.frame2, cylinderY.frame1; axis = 2, phi_start = -pi/2)

#=
   d     = Force(d=10.0)
   damper = Modia3D.AdaptorForceElementToFlange(w=d.w, tau=d.tau)
   Modia3D.connect(damper, rev1)
   =#
end

gravField = Modia3D.UniformGravityField(n=[0,0,-1])
doublePendulum = DoublePendulum(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=true, defaultFrameLength=0.3))


#Modia3D.visualizeAssembly!( doublePendulum )




model = Modia3D.SimulationModel( doublePendulum; useOptimizedStructure = false )
result = ModiaMath.simulate!(model; stopTime=5.0, tolerance=1e-6,interval=0.001, log=false)

ModiaMath.plot(result, [("rev1.phi", "rev2.phi"),
                        ("rev1.w"  , "rev2.w"),
                        ("rev1.a"  , "rev2.a")])

println("... success of Test_2Rev_ZylZ_BarX.jl!")
end
