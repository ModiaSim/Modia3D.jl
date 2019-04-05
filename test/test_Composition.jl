module Test_Composition

using Modia3D
using Modia3D.Test
using Modia3D.LinearAlgebra
using Modia3D.ModiaMath
using Modia3D.Unitful


r2 = [2.0,3.0,4.0]
@assembly TestObject3D begin
   world = Modia3D.Object3D()

   # Object3D fixed in world
   frame1 = Modia3D.Object3D(world, r=[1.0, 2.0, 3.0])

   # Object3D moving relatively to frame1

   frame2 = Modia3D.Object3D(frame1, r=r2, fixed=false)

   # Object3D fixed in frame2
   frame3 = Modia3D.Object3D(frame2, r=[-1,-2,-3])
end

@assembly TestGraphics begin
   world = Modia3D.Object3D()

   font  = Modia3D.Font(fontFamily="Arial", bold=true, charSize=0.2, color="LightBlue", transparency=0.5)
   text1 = Modia3D.TextShape("This is a box")
   text2 = Modia3D.TextShape("This is the xy plane"; font=font, 
                              axisAlignment=Modia3D.XY_Plane, alignment=Modia3D.Left)

   textShape1 = Modia3D.Object3D(world, text1)
   textShape2 = Modia3D.Object3D(world, text2; r=[1.0, 1.0, 0.0])
end


@assembly TestVisualElements begin
   world = Modia3D.Object3D()

   vmat   = Modia3D.Material(color="Blue", transparency=0.5)
   box    = Modia3D.Box(1.0,2.0,3.0; material=vmat)

   shape1 = Modia3D.Object3D(world , box)     
   shape2 = Modia3D.Object3D(world , Modia3D.Sphere(0.1))
   shape3 = Modia3D.Object3D(shape1, box; r=[1.0,2.0,3.0])
   shape4 = Modia3D.Object3D(shape3, box; R=ModiaMath.rot2(45u"°")) 
end


@assembly TestSolids begin
   world = Modia3D.Object3D()

   sbox  = Modia3D.SolidBox(1.0,2.0,3.0)
   smat  = Modia3D.SolidMaterial(density = 2700) 
   vmat  = Modia3D.Material(color="Blue", transparency=0.5)
   cmat  = Modia3D.ContactMaterialElastic(c=1e5, d=100)
   mass  = Modia3D.MassProperties(m=0.1, Ixx=1.0, Iyy=2.0, Izz=3.0)

   solid1 = Modia3D.Object3D(world, Modia3D.Solid(sbox, "Aluminium", vmat))
   solid2 = Modia3D.Object3D(world, Modia3D.Solid(sbox, 2700       , vmat))
   solid3 = Modia3D.Object3D(world, Modia3D.Solid(sbox, smat       , vmat)) 
   solid4 = Modia3D.Object3D(solid1, Modia3D.Solid(sbox, nothing, contactMaterial=cmat); r=[10.0,10.0,20.0])
   solid5 = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(0.1), mass, vmat, contactMaterial=cmat); r=[1.0,2.0,3.0])
   solid6 = Modia3D.Object3D(world, Modia3D.Solid(nothing                 , mass); r=[1.0,2.0,3.0])
end



# Test global properties
grav1 = Modia3D.NoGravityField()
grav2 = Modia3D.UniformGravityField()
grav3 = Modia3D.PointGravityField()
r     = [Modia3D.EarthRadius, 0.0, 0.0]
g1    = norm( Modia3D.gravityAcceleration(grav1,r) )
g2    = norm( Modia3D.gravityAcceleration(grav2,r) )
g3    = norm( Modia3D.gravityAcceleration(grav3,r) )

prop = Modia3D.SceneOptions()

@testset "Modia3D.Composition: test global properties" begin 
   @test g1 == 0.0
   @test isapprox(g2,9.81)
   @test isapprox(g3,9.8;atol=0.01)
end


# Test other objects
testObject3D = TestObject3D()
Modia3D.initAnalysis!(testObject3D)

for time = range(0.0, stop=2.0, length=101)
   r2 .+= 0.01*sin.([time, 2*time, 3*time])
   Modia3D.set_r!(testObject3D.frame2,r2)
   Modia3D.updatePosition!(testObject3D)  # absolute position of frame2 and frame3 updated
end
Modia3D.closeAnalysis!(testObject3D)

TestGraphics()
TestVisualElements()  
TestSolids()

end