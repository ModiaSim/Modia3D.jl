# Examples

Modia3D has examples in directory

```
$(Modia3D.path)/examples
```

to demonstrate various features of the package. Every example is in a separate module.
Most important examples (by pasting the corresponding `include(..)` command in the REPL,
the example is executed):

```julia 
  import Modia3D

  # Examples to demonstrate the visualization capabilities of Modia3D
  include("$(Modia3D.path)/examples/visual/Visualize_Solids.jl")
  include("$(Modia3D.path)/examples/visual/Visualize_GeometriesWithMaterial.jl")
  include("$(Modia3D.path)/examples/visual/Visualize_GeometriesWithoutMaterial.jl")
  include("$(Modia3D.path)/examples/visual/Visualize_Text.jl")
  include("$(Modia3D.path)/examples/visual/Visualize_TextFonts.jl")
  include("$(Modia3D.path)/examples/visual/Move_AllVisualObjects.jl")

  # Examples to demonstrate kinematic movement of assemblies
  include("$(Modia3D.path)/examples/kinematics/Move_DoublePendulum.jl")
  include("$(Modia3D.path)/examples/kinematics/Move_FourBar.jl")
  include("$(Modia3D.path)/examples/kinematics/Move_FourBar2.jl")

  # Examples to demonstrate dynamic simulation without force elements
  include("$(Modia3D.path)/examples/dynamics/Simulate_Pendulum.jl")
  include("$(Modia3D.path)/examples/dynamics/Simulate_DoublePendulum.jl")

  # Examples to demonstrate dynamic simulation with force elements connected to joints
  include("$(Modia3D.path)/examples/dynamics/Simulate_PendulumWithDamper.jl")
  include("$(Modia3D.path)/examples/dynamics/Simulate_PendulumWithController.jl")
  include("$(Modia3D.path)/examples/dynamics/Simulate_DoublePendulumWithDampers.jl")

  # Examples to demonstrate dynamic simulation with collision handling
  include("$(Modia3D.path)/examples/dynamics/Simulate_FallingBall2.jl")
  include("$(Modia3D.path)/examples/dynamics/Simulate_FallingBall4.jl")
```
