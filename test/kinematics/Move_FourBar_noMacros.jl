module Move_FourBar_noMacros

import Modia3D
using  Modia3D.ModiaMath
using  Modia3D.Unitful

# Solids
groundMaterial = Modia3D.Material(color="DarkGreen", transparency=0.5)
massProperties = "Aluminium"
vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)
vmat2 = Modia3D.Material(color="Red")

# Base dimensions
length1  = 1.0
width    = 0.2
height   = 0.2
diameter = 0.1
groundWidth  = 1.0
groundHeight = 0.01
cyl  = Modia3D.Cylinder(width/2, 1.2*width; material=vmat2)
Tcyl = ModiaMath.rot1(90u"°")
TGround = ModiaMath.rot1(-90u"°")


# -----------------------------------------------------------------------
# @assembly Bar(;Lx = 0.1, Ly=Lx/5, Lz=Ly) begin
#   frame0 = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), nothing, vmat1))
#   frame1 = Modia3D.Object3D(frame0; r=[-Lx/2, 0.0, 0.0])
#   frame2 = Modia3D.Object3D(frame0; r=[ Lx/2, 0.0, 0.0])
#   cyl1   = Modia3D.Object3D(frame1, cyl)
#   cyl2   = Modia3D.Object3D(frame2, cyl)
# end
mutable struct Bar <: Modia3D.AbstractAssembly
   _internal::Modia3D.AssemblyInternal
   Lx
   Ly
   Lz
   frame0
   frame1
   frame2
   cyl1
   cyl2

   function Bar(;sceneOptions=nothing,Lx = 0.1, Ly=Lx/5, Lz=Ly)
      this = new( Modia3D.AssemblyInternal(:Bar,sceneOptions) )
      this.Lx = Lx
      this.Ly = Ly
      this.Lz = Lz

      frame0 = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), nothing, vmat1))
      Modia3D.initAssemblyComponent!(this, frame0, "frame0")

      frame1 = Modia3D.Object3D(frame0; r=[-Lx/2, 0.0, 0.0])
      Modia3D.initAssemblyComponent!(this, frame1, "frame1")

      frame2 = Modia3D.Object3D(frame0; r=[ Lx/2, 0.0, 0.0])
      Modia3D.initAssemblyComponent!(this, frame2, "frame2")

      cyl1   = Modia3D.Object3D(frame1, cyl)
      Modia3D.initAssemblyComponent!(this, cyl1, "cyl1")

      cyl2   = Modia3D.Object3D(frame2, cyl)
      Modia3D.initAssemblyComponent!(this, cyl2, "cyl2")

      return this
   end
end
# -----------------------------------------------------------------------



# -----------------------------------------------------------------------
# @assembly Fourbar begin
#    world  = Modia3D.Object3D(Modia3D.CoordinateSystem(0.6))
#    frame1 = Modia3D.Object3D(world ; r=[0.5*length1, 0.0, groundWidth/2])
#    frame2 = Modia3D.Object3D(frame1; r=[length1    , 0.0, 0.0])
#    ground = Modia3D.Object3D(world , Modia3D.Box(3*length1, groundWidth, groundHeight; material=groundMaterial);
#                                               r = [1.5*length1, -groundHeight/2, groundWidth/2],
#                                               R = Modia3D.rot(;n1 = [-1,0,0], n2 = [0,0,1]))
#    bar1   = Bar(Lx=length1)
#    bar2   = Bar(Lx=length1)
#    bar3   = Bar(Lx=length1)
#
#    rev1   = Modia3D.Revolute(frame1     , bar1.frame1; phi =  pi/2)
#    rev2   = Modia3D.Revolute(bar1.frame2, bar2.frame1; phi = -pi/2)
#    rev3   = Modia3D.Revolute(frame2     , bar3.frame1; phi =  pi/2)
#    rev4   = Modia3D.Revolute(bar3.frame2, bar2.frame2; phi = -pi/2)
# end
mutable struct Fourbar <: Modia3D.AbstractAssembly
   _internal::Modia3D.AssemblyInternal
   world
   frame1
   frame2
   ground
   bar1
   bar2
   bar3
   rev1
   rev2
   rev3
   rev4

   function Fourbar(;sceneOptions=nothing)
      this = new( Modia3D.AssemblyInternal(:Fourbar, sceneOptions) )

      world  = Modia3D.Object3D(Modia3D.CoordinateSystem(0.6)) 
      Modia3D.initAssemblyComponent!(this, world , "world")

      frame1 = Modia3D.Object3D(world ; r=[0.5*length1, 0.0, groundWidth/2])
      Modia3D.initAssemblyComponent!(this, frame1, "frame1")

      frame2 = Modia3D.Object3D(frame1; r=[length1, 0.0, 0.0])
      Modia3D.initAssemblyComponent!(this, frame2, "frame2")

      ground = Modia3D.Object3D(world , Modia3D.Box(3*length1, groundWidth, groundHeight; material=groundMaterial);
                                                 r = [1.5*length1, -groundHeight/2, groundWidth/2],
                                                 R = ModiaMath.rot_nxy([-1,0,0], [0,0,1]))
      Modia3D.initAssemblyComponent!(this, ground, "ground")

      bar1 = Bar(Lx=length1)
      Modia3D.initAssemblyComponent!(this, bar1, "bar1")

      bar2 = Bar(Lx=length1)
      Modia3D.initAssemblyComponent!(this, bar2, "bar2")

      bar3 = Bar(Lx=length1)
      Modia3D.initAssemblyComponent!(this, bar3, "bar3")

      rev1 = Modia3D.Revolute(frame1, bar1.frame1; phi_start =  pi/2)
      Modia3D.initAssemblyComponent!(this, rev1, "rev1")

      rev2 = Modia3D.Revolute(bar1.frame2, bar2.frame1; phi_start = -pi/2)
      Modia3D.initAssemblyComponent!(this, rev2, "rev2")

      rev3 = Modia3D.Revolute(frame2, bar3.frame1; phi_start =  pi/2)
      Modia3D.initAssemblyComponent!(this, rev3, "rev3")

      rev4 = Modia3D.Revolute(bar3.frame2, bar2.frame2; phi_start = -pi/2)      
      Modia3D.initAssemblyComponent!(this, rev4, "rev4")

      return this
   end
end
# -----------------------------------------------------------------------


# Kinematic simulation
fourbar = Fourbar(sceneOptions=Modia3D.SceneOptions(visualizeFrames=true,
                                                    defaultFrameLength=0.3,
                                                    enableContactDetection=false))
Modia3D.initAnalysis!(fourbar)
tStart=0.0
tEnd  =1.0
Modia3D.driveJoint!(fourbar.rev1)  # Define that rev1 is driven (setAngle! can be called on it)
Modia3D.driveJoint!(fourbar.rev2)  # Define that rev2 is driven (setAngle! can be called on it)

@static if VERSION >= v"0.7.0-DEV.2005"
    LINSPACE(start,stop,length) = range(0.0, stop=stop, length=length)
else
    LINSPACE(start,stop,length) = linspace(start,stop,length)
end

for time = LINSPACE(tStart, tEnd, 101)
  # update positional degrees of freedom
  delta_phi = Modia3D.linearMovement(pi/3, tStart, tEnd, time)
  Modia3D.setAngle!(fourbar.rev1,  pi/2 - delta_phi)
  Modia3D.setAngle!(fourbar.rev2, -pi/2 + delta_phi)

  Modia3D.updatePosition!(fourbar)

  # Visualize
  Modia3D.visualize!(fourbar,time)
end

Modia3D.closeAnalysis!(fourbar)

println("... success of Move_FourBar_noMacros.jl!")
end
