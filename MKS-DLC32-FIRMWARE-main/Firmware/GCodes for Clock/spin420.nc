( spin420.nc  –  double whirl, then land on 4:20 )
( assumes: 1 mm = 1 degree, X=minute, Y=hour )

G21            ; units mm (deg)
G90            ; absolute coords from home (12 o’clock)

; — two fast clockwise revolutions —
G0 X0   Y0    F20000
G0 X360 Y360  F20000
G0 X720 Y720  F20000

G4 P0.5        ; 0.5 s pause for flair

; — quick reverse half-spin —
G0 X360 Y360  F15000
G0 X0   Y0    F15000

; — snap to 4:20 —
; minute: 20 min × 6 deg = 120
; hour:   (4 + 20/60) × 30 deg ≈ 130
G0 X120 Y130  F12000

G4 P2          ; hold on the time for 2 s