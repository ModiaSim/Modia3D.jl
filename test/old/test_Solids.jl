module TestSolid

import Modia3D
using  Modia3D.Test
using  Modia3D.LinearAlgebra
using StaticArrays
EYE3() = @SMatrix[1.0  0.0  0.0;
                0.0   1.0    0.0 ;
                0.0   0.0    1.0]



# Test Sphere
r = 2.0
m = 3.0
shape   = Modia3D.Sphere(diameter=2r)
mass  = Modia3D.MassProperties(shape,m)
solid = Modia3D.Solid(shape=shape, massProperties=mass)
obj   = Modia3D.Object3D(feature = solid)

rref  = SVector(1.0,2.0,3.0)
Tref  = EYE3()
obj.r_abs = rref
obj.R_abs = Tref
eref  = rref/norm(rref)
sp    = Modia3D.supportPoint(obj,eref)
AABB1 = Modia3D.BoundingBox{Float64}()
AABB2 = Modia3D.BoundingBox{Float64}(rref[1]-r, rref[1]+r, rref[2]-r, rref[2]+r, rref[3]-r, rref[3]+r)
Modia3D.boundingBox!(obj, AABB1)

@testset "Modia3D.Solid: Test Sphere" begin
   @test mass.rCM == [0.0, 0.0, 0.0]
   @test isapprox(Modia3D.volume(shape), 4/3*pi*r^3)
   @test isapprox(mass.I             , 2/5*m*r^2*EYE3())
   @test isapprox(sp                 , rref + r*eref)
   @test isapprox(AABB1.x_min        , AABB2.x_min)
   @test isapprox(AABB1.x_max        , AABB2.x_max)
   @test isapprox(AABB1.y_min        , AABB2.y_min)
   @test isapprox(AABB1.y_max        , AABB2.y_max)
   @test isapprox(AABB1.z_min        , AABB2.z_min)
   @test isapprox(AABB1.z_max        , AABB2.z_max)
end


# Test Ellipsoid
a = 1.0
b = 2.0
c = 3.0
m = 4.0
shape  = Modia3D.Ellipsoid(lengthX=2a, lengthY=2b, lengthZ=2c)
mass = Modia3D.MassProperties(shape,m)
solid = Modia3D.Solid(shape=shape, massProperties=mass)
obj = Modia3D.Object3D(feature = solid)

rref  = SVector(1.0,2.0,3.0)
Tref  = EYE3()
eref  = SVector(0.0, 0.0, 1.0)
obj.r_abs = rref
obj.R_abs = Tref
sp    = Modia3D.supportPoint(obj,eref)
AABB1 = Modia3D.BoundingBox{Float64}()
AABB2 = Modia3D.BoundingBox{Float64}(rref[1]-a, rref[1]+a, rref[2]-b, rref[2]+b, rref[3]-c, rref[3]+c)
Modia3D.boundingBox!(obj, AABB1)

@testset "Modia3D.Solid: Test Ellipsoid" begin
   @test mass.rCM == [0.0, 0.0, 0.0]
   @test isapprox(Modia3D.volume(shape), 4/3*pi*a*b*c)
   @test isapprox(mass.I             , m/5*Diagonal([b^2+c^2, c^2+a^2, a^2+b^2]))
   @test isapprox(sp                 , rref + c*eref)
   @test isapprox(AABB1.x_min        , AABB2.x_min)
   @test isapprox(AABB1.x_max        , AABB2.x_max)
   @test isapprox(AABB1.y_min        , AABB2.y_min)
   @test isapprox(AABB1.y_max        , AABB2.y_max)
   @test isapprox(AABB1.z_min        , AABB2.z_min)
   @test isapprox(AABB1.z_max        , AABB2.z_max)
end


# Test Box and MassProperties
a = 1.0
b = 2.0
c = 3.0
m = 4.0
d = 2700.0
solidMaterial = Modia3D.SolidMaterial(density=d)
shape   = Modia3D.Box(lengthX=a, lengthY=b, lengthZ=c)
mass1 = Modia3D.MassProperties(shape,m)
mass2 = Modia3D.MassProperties(shape,solidMaterial)
mass3 = Modia3D.MassProperties(shape,"Aluminium")
solid = Modia3D.Solid(shape=shape, massProperties=mass1)
obj = Modia3D.Object3D(feature = solid)

V     = Modia3D.volume(shape)
m2    = mass2.m

rref  = SVector(1.0,2.0,3.0)
Tref  = EYE3()
eref  = SVector(0.0, 0.0, 1.0)
obj.r_abs = rref
obj.R_abs = Tref
sp    = Modia3D.supportPoint(obj,eref)
AABB1 = Modia3D.BoundingBox{Float64}()
AABB2 = Modia3D.BoundingBox{Float64}(rref[1]-a/2-solid.collisionSmoothingRadius, rref[1]+a/2+solid.collisionSmoothingRadius, rref[2]-b/2-solid.collisionSmoothingRadius, rref[2]+b/2+solid.collisionSmoothingRadius, rref[3]-c/2-solid.collisionSmoothingRadius, rref[3]+c/2+solid.collisionSmoothingRadius)
Modia3D.boundingBox!(obj, AABB1)

@testset "Modia3D.Solid: Test Box and MassProperties" begin
   @test mass1.rCM == [0.0, 0.0, 0.0]
   @test isapprox(V          , a*b*c)
   @test isapprox(mass1.I    , m/12*Diagonal([b^2+c^2, c^2+a^2, a^2+b^2]))
   @test isapprox(mass2.m    , d*V)
   @test isapprox(mass2.I    , m2/12*Diagonal([b^2+c^2, c^2+a^2, a^2+b^2]))
   @test isapprox(mass3.m    , mass2.m)
   @test isapprox(mass3.I    , mass2.I)
  # @test isapprox(AABB1.x_min, AABB2.x_min)
  # @test isapprox(AABB1.x_max, AABB2.x_max)
  # @test isapprox(AABB1.y_min, AABB2.y_min)
  # @test isapprox(AABB1.y_max, AABB2.y_max)
  # @test isapprox(AABB1.z_min, AABB2.z_min)
  # @test isapprox(AABB1.z_max, AABB2.z_max)
end


# Test MassProperties
m   = 2.0
rCM = [1,2,3]
II  = [5 1 2;
       1 6 3;
       2 3 7]

mass1 = Modia3D.MassProperties(m,rCM,II)
mass2 = Modia3D.MassProperties(mass=m,centerOfMass=rCM,Ixx=5,Iyy=6,Izz=7,Ixy=1,Ixz=2,Iyz=3)

@testset "Modia3D.Solid: Test MassProperties" begin
   @test isapprox(mass1.m, m)
   @test isapprox(mass2.m, m)

   @test mass1.rCM == rCM
   @test mass2.rCM == rCM

   @test mass1.I == II
   @test mass2.I == II
end

end
