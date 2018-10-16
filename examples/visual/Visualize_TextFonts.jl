module Visualize_TextFonts

using Modia3D
 

@assembly DisplayFontFamily(fontFamily; cs=0.1) begin
   frame0         = Modia3D.Object3D()
   fontNormal     = Modia3D.Object3D(frame0, Modia3D.TextShape(fontFamily; alignment = Modia3D.Left, offset=[0.0,0.0,10*cs],
                                                            font = Modia3D.Font(fontFamily=fontFamily, bold=false, italic=false, charSize=cs)))
   fontItalic     = Modia3D.Object3D(frame0, Modia3D.TextShape(fontFamily * " italic"; alignment = Modia3D.Left, offset=[0.0,0.0,8.5*cs],
                                                            font = Modia3D.Font(fontFamily=fontFamily, bold=false, italic=true, charSize=cs)))
   fontBold       = Modia3D.Object3D(frame0, Modia3D.TextShape(fontFamily * " bold"; alignment = Modia3D.Left, offset=[0.0,0.0,7*cs],
                                                            font = Modia3D.Font(fontFamily=fontFamily, bold=true, italic=false, charSize=cs)))
   fontBoldItalic = Modia3D.Object3D(frame0, Modia3D.TextShape(fontFamily * " bold + italic"; alignment = Modia3D.Left, offset=[0.0,0.0,5.5*cs],
                                                            font = Modia3D.Font(fontFamily=fontFamily, bold=true, italic=true, charSize=cs)))
   fontNormalRed  = Modia3D.Object3D(frame0, Modia3D.TextShape(fontFamily * " red color"; alignment = Modia3D.Left, offset=[0.0,0.0,4*cs],
                                                            font = Modia3D.Font(fontFamily=fontFamily, color="Red", charSize=cs)))
end

@assembly DisplayFonts begin
   Arial         = DisplayFontFamily("Arial")
   ArialNarrow   = DisplayFontFamily("ArialNarrow")
   TimesNewRoman = DisplayFontFamily("TimesNewRoman")
   Verdana       = DisplayFontFamily("Verdana")
   CourierNew    = DisplayFontFamily("CourierNew")
   FreeSans      = DisplayFontFamily("FreeSans")
   FreeSerif     = DisplayFontFamily("FreeSerif")

   world         = Modia3D.Object3D()

   Modia3D.connect(world, Arial.frame0        ; r=[0.0,-4.5,0.0])
   Modia3D.connect(world, ArialNarrow.frame0  ; r=[0.0,-3.5,0.0])
   Modia3D.connect(world, TimesNewRoman.frame0; r=[0.0,-2.0,0.0])
   Modia3D.connect(world, Verdana.frame0      ; r=[0.0,-0.5,0.0])
   Modia3D.connect(world, CourierNew.frame0   ; r=[0.0, 1.0,0.0])
   Modia3D.connect(world, FreeSans.frame0     ; r=[0.0, 2.5,0.0])
   Modia3D.connect(world, FreeSerif.frame0    ; r=[0.0, 4.0,0.0])
end
Modia3D.visualizeAssembly!( DisplayFonts() )

println("... success of Visualize_TextFonts.jl!")

end
