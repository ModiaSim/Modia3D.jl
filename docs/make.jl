using Documenter, Modia3D

makedocs(
  modules  = [Modia3D],
  sitename = "Modia3D",
  authors  = "Andrea Neumayr, Martin Otter (DLR-SR)",
  format   = Documenter.HTML(prettyurls = false),
  pages    = [
     "Home"   => "index.md",
     "Manual" => [
        "man/GettingStarted.md"
        "man/Examples.md"
        "man/Materials.md"
        "man/CollisionHandling.md"
        "man/Plans.md"
        ],
     "Library" => [
        "lib/Composition.md",
        "lib/Graphics.md",
        "lib/Solids.md",
        "lib/ForceElements.md",
        "lib/Basics.md"
     ],
     "Internal" => [
        "internal/ContactDetection.md"
     ]
  ]
)
