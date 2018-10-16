module Visualize_Beam

using Modia3D


mat1  = Modia3D.Material(color="LightBlue", transparency=0.5)

world = Modia3D.Object3D(Modia3D.CoordinateSystem(1.0))
beam  = Modia3D.Object3D(world, Modia3D.Beam(0.4,0.5,0.3, material=mat1); r=[1.0,1.0,1.0])

Modia3D.visualizeWorld!(world, sceneOptions = Modia3D.SceneOptions(visualizeFrames=true,
                                                                   defaultFrameLength=0.7,
                                                                   enableContactDetection=false))

println("... success of Visualize_Beam.jl!")

end
