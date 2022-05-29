module TestFrames

using Modia3D
using Modia3D.Test

@test isapprox(Modia3D.rot1(pi/4), Modia3D.rot1(45u"°"))
@test isapprox(Modia3D.rot2(pi/4), Modia3D.rot2(45u"°"))
@test isapprox(Modia3D.rot3(pi/4), Modia3D.rot3(45u"°"))

@test isapprox(Modia3D.rot123(pi/2, pi/2, pi/2), Modia3D.rot123([90.0, 90.0, 90.0]u"°"))
@test isapprox(Modia3D.rot123(pi/2, pi/2, pi/2), [0.0  0.0 1.0;
                                                  0.0 -1.0 0.0;
                                                  1.0  0.0 0.0])

angles = [30.0, 40.0, 50.0]u"°"
Ra  = Modia3D.rot123(angles)
va1 = [1.0, -2.0, 3.0]
va2 = Modia3D.resolve2(Ra,va1)
@test isapprox(va1, Modia3D.resolve1(Ra, va2))
@test isapprox(va2, Modia3D.resolve2(angles, va1))
@test isapprox(va1, Modia3D.resolve1(angles, va2))

angle1 = 30.0u"°"
angle3 = 40.0u"°"
angle2 = 50.0u"°"
anglesb = [angle1, angle3, angle2]
Rb = Modia3D.rot2(angle2)*Modia3D.rot3(angle3)*Modia3D.rot1(angle1)  # rotation sequence 132
rotation123b = false
vb1 = va1
vb2 = Modia3D.resolve2(Rb,vb1)
@test isapprox(vb1, Modia3D.resolve1(Rb, vb2))
@test isapprox(vb2, Modia3D.resolve2(anglesb, vb1, rotation123=rotation123b))
@test isapprox(vb1, Modia3D.resolve1(anglesb, vb2, rotation123=rotation123b))


end