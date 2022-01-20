module TestShapes

import Modia3D
using  Modia3D.Test
using  Modia3D.StaticArrays


material1 = Modia3D.VisualMaterial(;color="DarkRed", wireframe=true)
material2 = Modia3D.VisualMaterial(;color=[0,0,255], shininess=0.2)

sphere = Modia3D.Visual(shape=Modia3D.Shapes.Sphere(diameter=2.0),
    visualMaterial=material1)
ellipsoid = Modia3D.Visual(shape=Modia3D.Shapes.Ellipsoid(lengthX=1.0, lengthY=2.0,
    lengthZ=3.0))
box = Modia3D.Visual(shape=Modia3D.Shapes.Box(lengthX=1.0, lengthY=2.0,
    lengthZ=3.0))
cylinder = Modia3D.Visual(shape=Modia3D.Shapes.Cylinder(axis=3, diameter=3.0,
    length=1.0, innerDiameter=1.5))
cone = Modia3D.Visual(shape=Modia3D.Shapes.Cone(diameter=3.0, length=1.0,
    topDiameter=0.6), visualMaterial=material1)
capsule = Modia3D.Visual(shape=Modia3D.Shapes.Capsule(axis=3, diameter=3.0,
    length=1.0))
beam = Modia3D.Visual(shape=Modia3D.Shapes.Beam(axis=1, length=1.0, width=2.0,
    thickness=3.0), visualMaterial=material1)
spring = Modia3D.Visual(shape=Modia3D.Spring(axis=3, length=1.0, diameter=3.0,
    windings=10, wireDiameter=0.2), visualMaterial=material2)
gearWheel = Modia3D.Visual(shape=Modia3D.GearWheel(axis=3, diameter=3.0,
    length=1.0, innerDiameter=2.4, teeth=30, angle=0.2), visualMaterial=material2)
coordsys = Modia3D.Visual(shape=Modia3D.CoordinateSystem(length=2.0))
grid= Modia3D.Visual(shape=Modia3D.Grid(axis=3, length=3.0, width=1.0, distance=0.1, lineWidth=2.0))

filename = joinpath(Modia3D.path, "objects", "engine", "crank", "crank.obj")
mesh1 = Modia3D.Visual(shape=Modia3D.Shapes.FileMesh(filename=filename))
mesh2 = Modia3D.Visual(shape=Modia3D.Shapes.FileMesh(filename=filename, scale=[2.0,2.0,2.0],
    useMaterialColor=true, smoothNormals=true))
mesh3 = Modia3D.Visual(shape=Modia3D.Shapes.FileMesh(filename=filename, scale=[2.0,3.0,4.0],
    useMaterialColor=true, smoothNormals=true))

text1 = Modia3D.TextShape(text="This is a text line")
font1 = Modia3D.Font()
font2 = Modia3D.Font(fontFamily="FreeSans", bold=false, italic=false, charSize=0.1,
    color = "LightBlue", transparency=0.0)
text2 = Modia3D.TextShape(text="Another text line", font=font2, offset=[0.1,0.2,0.3],
    axisAlignment=Modia3D.XZ_Plane,alignment=Modia3D.Left)


@testset "Modia3D.Shapes: Test shape, color, fonts" begin
   @test material1.color     == MVector{3,Cint}(139,0,0)
   @test material1.wireframe == true
end

end
