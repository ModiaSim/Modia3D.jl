module Simulate_DoublePendulum

using  Modia3D
import Modia3D.ModiaMath



# Solids
groundMaterial = Modia3D.Material(color="DarkGreen", transparency=0.5)
# massProperties = Modia3D.MassProperties()
vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)
vmat2 = Modia3D.Material(color="Red")

@assembly Bar(;Lx = 0.1, Ly=Lx/5, Lz=Ly, m=1.0) begin
   frame0 = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), Modia3D.MassProperties(m=m), vmat1))
   frame1 = Modia3D.Object3D(frame0; r=[-Lx/2, 0.0, 0.0])
   frame2 = Modia3D.Object3D(frame0; r=[ Lx/2, 0.0, 0.0])
   cyl    = Modia3D.Cylinder(Ly/2,1.2*Ly; material=vmat2)
   cyl1   = Modia3D.Object3D(frame1, cyl; visualizeFrame=false)
   cyl2   = Modia3D.Object3D(frame2, cyl; visualizeFrame=false)
end

@assembly DoublePendulum(;Lx = 1.0, m=1.0) begin
   world = Modia3D.Object3D(Modia3D.CoordinateSystem(0.5*Lx))
   bar1  = Bar(Lx=Lx, m=m)
   bar2  = Bar(Lx=Lx, m=m)
   rev1  = Modia3D.Revolute(world, bar1.frame1; axis = 3)
   rev2  = Modia3D.Revolute(bar1.frame2, bar2.frame1; axis = 1)
end


# Modia3D.visualizeAssembly!( DoublePendulum(), Modia3D.SceneOptions(visualizeFrames=true, defaultFrameLength=0.3) )


doublePendulum = DoublePendulum(sceneOptions=Modia3D.SceneOptions(visualizeFrames=true, defaultFrameLength=0.3))
model = Modia3D.SimulationModel( doublePendulum; useOptimizedStructure = true )
result = ModiaMath.simulate!(model; stopTime=5.0, tolerance=1e-6,interval=0.001, log=false)

ModiaMath.plot(result, [("rev1.phi", "rev2.phi"),
                        ("rev1.w"  , "rev2.w"),
                        ("rev1.a"  , "rev2.a")])

println("... success of Simulate_DoublePendulum.jl!")
end
