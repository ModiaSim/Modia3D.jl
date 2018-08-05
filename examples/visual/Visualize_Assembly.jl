module Visualize_Assembly

using Modia3D

# Basic definitions
vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)
vmat2 = Modia3D.Material(color="Red")


@assembly Bar(;Lx = 0.1, Ly=Lx/5, Lz=Ly) begin
   frame0 = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), nothing, vmat1))
   frame1 = Modia3D.Object3D(frame0; r=[-Lx/2, 0.0, 0.0])
   frame2 = Modia3D.Object3D(frame0; r=[ Lx/2, 0.0, 0.0])

   cyl    = Modia3D.Cylinder(Ly/2,1.2*Ly; material=vmat2)
   cyl1   = Modia3D.Object3D(frame1, cyl)
   cyl2   = Modia3D.Object3D(frame2, cyl)
end

Modia3D.visualizeAssembly!( Bar(Lx = 1.0, sceneOptions=Modia3D.SceneOptions(visualizeFrames=true,
                                                                            defaultFrameLength=0.3,
                                                                            enableContactDetection=false) ) )

println("... success of Visualize_Assembly.jl!")
end
