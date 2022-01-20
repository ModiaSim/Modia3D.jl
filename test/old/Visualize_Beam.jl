module Visualize_Beam

import Modia3D

mat1  = Modia3D.VisualMaterial(color="LightBlue", transparency=0.5)

world = Modia3D.Object3D(
    feature = Modia3D.Visual(shape=Modia3D.CoordinateSystem(length=1.0)))
beam  = Modia3D.Object3D(parent=world, translation=[1.0, 1.0, 1.0],
    feature = Modia3D.Visual(shape=Modia3D.Shapes.Beam(axis=1, length=0.4, width=0.5, thickness=0.3), visualMaterial=mat1))

Modia3D.visualizeWorld!(world,
    scene=Modia3D.Scene(visualizeFrames=true, defaultFrameLength=0.7, enableContactDetection=false))

println("... test/old/Visualize_Beam.jl completed.")

end
