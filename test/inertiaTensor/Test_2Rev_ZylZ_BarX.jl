module Test_2Rev_ZylZ_BarX

using  Modia3D
import Modia3D.ModiaMath


# Solids
groundMaterial = Modia3D.Material(color="DarkGreen", transparency=0.5)
vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)
vmat2 = Modia3D.Material(color="Red")

@forceElement Force(; d=10.0) begin
   w   = ModiaMath.RealScalar("w",   causality=ModiaMath.Input,  numericType=ModiaMath.WR)
   tau = ModiaMath.RealScalar("tau",  causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeTorque(f::Force, sim::ModiaMath.SimulationState)
   f.tau.value = f.d #*sin(sim.time)
end


@assembly Box(; Lx = 1.0, Ly=Lx, Lz=3*Lx) begin
   frame0 = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBox(Lx,Ly,Lz), "Aluminium", vmat1))
   frame1 = Modia3D.Object3D(frame0; r=[0.0, 0.0, -Lz/2])
   frame2 = Modia3D.Object3D(frame0; r=[0.0, Ly/2,  Lz/2])
   cyl    = Modia3D.Cylinder(Lx/5,Lz/7; material=vmat2)
   cyl1   = Modia3D.Object3D(frame1, cyl; visualizeFrame=false)
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
   rev1  = Modia3D.Revolute(world, boxZ.frame1; axis = 3)
   rev2  = Modia3D.Revolute(boxZ.frame2, cylinderX.frame1; axis = 1, phi_start = pi) # -2*pi/3)

   f    = Force(d=100000.0)
   force = Modia3D.AdaptorForceElementToFlange(tau=f.tau)
   Modia3D.connect(force, rev1)
end


gravField = Modia3D.UniformGravityField( n=[0,0,-1])
doublePendulum = DoublePendulum(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=true, defaultFrameLength=0.3))

#Modia3D.visualizeAssembly!( doublePendulum )


model = Modia3D.SimulationModel( doublePendulum )
result = ModiaMath.simulate!(model; stopTime=5.0, tolerance=1e-6,interval=0.001, log=false)

ModiaMath.plot(result, [("rev1.phi", "rev2.phi"),
                        ("rev1.w"  , "rev2.w"),
                        ("rev1.a"  , "rev2.a"),
                        ("rev1.tau", "rev2.tau")])

println("... success of Test_2Rev_ZylZ_BarX.jl!")
end
