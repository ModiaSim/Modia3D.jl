module RuntestsWithGraphics

using  ModiaLang
import Test

const title = "Modia3D (with " * currentPlotPackage() * ")"

@time Test.@testset verbose=true "$title" begin
    include("includeTests.jl")
end

end
