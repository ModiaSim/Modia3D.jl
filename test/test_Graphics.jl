module TestGraphics

import Modia3D

@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Modia3D.Test
end

using Modia3D.StaticArrays


material1 = Modia3D.Material(;color="DarkRed", wireframe=true)
material2 = Modia3D.Material(;color=[0,0,255], shininess=0.2)

sphere    = Modia3D.Sphere(2.0, material=material1)
ellipsoid = Modia3D.Ellipsoid(1.0,2.0,3.0, material=material1)
box       = Modia3D.Box(1.0,2.0,3.0, material=material1)
cylinder  = Modia3D.Cylinder(3.0,1.0;Dy=0., material=material1)
capsule   = Modia3D.Capsule(3.0,1.0;Dy=0.5, material=material1)
beam      = Modia3D.Beam(1.0,2.0,3.0, material=material1)
cone      = Modia3D.Cone(3.0,1.0;Dy=0.5, relativeTipDiameter=0.2,
                         relativeInnerDiameter=0.5, material=material1)
pipe      = Modia3D.Pipe(3.0,1.0;Dy=0.5, relativeTipDiameter=0.2,
                         relativeInnerDiameter=0.5, material=material1)
spring    = Modia3D.Spring(3.0,1.0;Dy=0.5, windings=10,springRadius=0.1, material=material2)
gearWheel = Modia3D.GearWheel(3.0,1.0;Dy=0.5,relativeInnerDiameter=0.8,teeth=30,
                              angle=0.2, material=material2)
coordsys  = Modia3D.CoordinateSystem(2.0)
grid      = Modia3D.Grid(3.0,1.0, distance=0.1, lineWidth=2.0)

filename = joinpath(Modia3D.path, "objects", "engine", "crank", "crank.obj")
mesh1 = Modia3D.FileMesh(filename)
mesh2 = Modia3D.FileMesh(filename, 2.0; useMaterialColor=true, smoothNormals=true)
mesh3 = Modia3D.FileMesh(filename; scaleFactor=[2.0,3.0,4.0], useMaterialColor=true, smoothNormals=true)

text1 = Modia3D.TextShape("This is a text line")
font1 = Modia3D.Font()
font2 = Modia3D.Font(fontFamily="FreeSans", bold=false, italic=false, charSize=0.1,
                     color = "LightBlue", transparency=0.0)
text2 = Modia3D.TextShape("Another text line", font=font2, offset=[0.1,0.2,0.3],
                          axisAlignment=Modia3D.XZ_Plane,alignment=Modia3D.Left)


@testset "Modia3D.Graphics: Test geometry, color, fonts" begin 
   @test material1.color     == MVector{3,Cint}(155,0,0)
   @test material1.wireframe == true
end

end
