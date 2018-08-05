module TestSolid

@static if VERSION < v"0.7.0-DEV.2005"
    using Base.Test
else
    using Test
end

import Modia3D


# Test Sphere
r = 2.0
m = 3.0
geo   = Modia3D.SolidSphere(2r)
mass  = Modia3D.MassProperties(geo,m)

rref  = [1.0,2.0,3.0]
Tref  = eye(3)
eref  = rref/norm(rref)
sp    = Modia3D.supportPoint(geo,rref,Tref,eref)
AABB1 = Modia3D.BoundingBox()
AABB2 = Modia3D.BoundingBox(rref[1]-r, rref[1]+r, rref[2]-r, rref[2]+r, rref[3]-r, rref[3]+r)
Modia3D.boundingBox!(geo,AABB1,rref,Tref)

@testset "Modia3D.Solid: Test Sphere" begin
   @test mass.rCM == [0.0, 0.0, 0.0]
   @test isapprox(Modia3D.volume(geo), 4/3*pi*r^3) 
   @test isapprox(mass.I             , 2/5*m*r^2*eye(3))   
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
geo  = Modia3D.SolidEllipsoid(2a,2b,2c)
mass = Modia3D.MassProperties(geo,m)

rref  = [1.0,2.0,3.0]
Tref  = eye(3)
eref  = [0.0, 0.0, 1.0]
sp    = Modia3D.supportPoint(geo,rref,Tref,eref)
AABB1 = Modia3D.BoundingBox()
AABB2 = Modia3D.BoundingBox(rref[1]-a, rref[1]+a, rref[2]-b, rref[2]+b, rref[3]-c, rref[3]+c)
Modia3D.boundingBox!(geo,AABB1,rref,Tref)

@testset "Modia3D.Solid: Test Ellipsoid" begin
   @test mass.rCM == [0.0, 0.0, 0.0]
   @test isapprox(Modia3D.volume(geo), 4/3*pi*a*b*c) 
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
geo   = Modia3D.SolidBox(a,b,c)
mass1 = Modia3D.MassProperties(geo,m)
mass2 = Modia3D.MassProperties(geo,solidMaterial)
mass3 = Modia3D.MassProperties(geo,"Aluminium")
V     = Modia3D.volume(geo)
m2    = mass2.m

rref  = [1.0,2.0,3.0]
Tref  = eye(3)
eref  = [0.0, 0.0, 1.0]
sp    = Modia3D.supportPoint(geo,rref,Tref,eref)
AABB1 = Modia3D.BoundingBox()
AABB2 = Modia3D.BoundingBox(rref[1]-a/2-geo.rsmall, rref[1]+a/2+geo.rsmall, rref[2]-b/2-geo.rsmall, rref[2]+b/2+geo.rsmall, rref[3]-c/2-geo.rsmall, rref[3]+c/2+geo.rsmall)
Modia3D.boundingBox!(geo,AABB1,rref,Tref)

@testset "Modia3D.Solid: Test Box and MassProperties" begin
   @test mass1.rCM == [0.0, 0.0, 0.0]
   @test isapprox(V          , a*b*c) 
   @test isapprox(mass1.I    , m/12*Diagonal([b^2+c^2, c^2+a^2, a^2+b^2]))   
   @test isapprox(mass2.m    , d*V)
   @test isapprox(mass2.I    , m2/12*Diagonal([b^2+c^2, c^2+a^2, a^2+b^2])) 
   @test isapprox(mass3.m    , mass2.m)
   @test isapprox(mass3.I    , mass2.I)
   @test isapprox(AABB1.x_min, AABB2.x_min)
   @test isapprox(AABB1.x_max, AABB2.x_max)
   @test isapprox(AABB1.y_min, AABB2.y_min)
   @test isapprox(AABB1.y_max, AABB2.y_max)
   @test isapprox(AABB1.z_min, AABB2.z_min)
   @test isapprox(AABB1.z_max, AABB2.z_max)   
end


# Test MassProperties
m   = 2.0
rCM = [1,2,3]
I   = [5 1 2;
       1 6 3;
       2 3 7] 

mass1 = Modia3D.MassProperties(m,rCM,I)
mass2 = Modia3D.MassProperties(m=m,rCM=rCM,Ixx=5,Iyy=6,Izz=7,Ixy=1,Ixz=2,Iyz=3)

@testset "Modia3D.Solid: Test MassProperties" begin   
   @test isapprox(mass1.m, m) 
   @test isapprox(mass2.m, m) 

   @test mass1.rCM == rCM 
   @test mass2.rCM == rCM

   @test mass1.I == I 
   @test mass2.I == I
end

end