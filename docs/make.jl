using Documenter, Modia3D

makedocs(
  modules  = [Modia3D],
  format   = :html,
  sitename = "Modia3D",
  authors  = "Andrea Neumayr, Martin Otter (DLR-SR)",
  pages    = [
     "Home"   => "index.md",
     "Manual" => [
        "man/GettingStarted.md"
        "man/Examples.md"
        "man/Plans.md"
        ],
     "Library" => [
        "lib/Composition.md",
        "lib/Graphics.md",
        "lib/Solids.md",
        "lib/ForceElements.md",
        "lib/Basics.md"
     ]
  ]
)

