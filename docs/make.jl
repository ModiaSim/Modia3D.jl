using Documenter, Modia3D

makedocs(
  #modules  = [Modia3D],
  sitename = "Modia3D",
  authors  = "Andrea Neumayr, Martin Otter, Gerhard Hippmann (DLR-SR)",
  format   = Documenter.HTML(prettyurls = false),
  pages    = [
     "Home"   => "index.md",
     "Tutorial" => [
        "tutorial/Tutorial.md",
        "tutorial/GettingStarted.md",
        "tutorial/HierarchicalModels.md",
        "tutorial/RecursiveModels.md",
        "tutorial/CollisionHandling.md"
     ],
     "Components" => [
        "Components/Object3D.md"
        "Components/Shapes.md"
        "Components/Joints.md"
        "Components/Materials.md"
        "Components/GravityField.md"
        "Components/ForceElements.md"
     ],
     "Internal" => [
        "internal/Profiling.md"
        "internal/DynamicDispatch.md"
        "internal/ContactDetection.md"
        "internal/ContactForceLaw.md"
     ],
  ]
)
