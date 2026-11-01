# Flushing volume calibration — High Level Design

## Purpose and scope

When a single nozzle switches filament, the old filament has to be pushed out
before the new colour prints clean. `flush_volumes_matrix` holds that volume
for every ordered pair of filaments. OrcaSlicer fills the matrix automatically
from the colour distance between the two filaments, and `flush_multiplier`
scales the whole matrix. The estimate does not know the hotend, the nozzle or
how strongly a pigment tints, so users either flush too much and waste
filament, or flush too little and get contaminated walls.

**Calibration → Flushing volume** measures the volume instead. For every
ordered pair of the selected filaments it prints a small object: a base in the
"from" filament and a tower in the "to" filament, with only a small priming
flush (the *minimum flush*) between them. The tower starts printing while the
old filament is still in the nozzle, so its lower part shows a gradient from
the old colour to the new one. The tower has a constant cross-section, so the
height `h` of the gradient converts directly into a volume:

```
flushing volume = minimum flush + h × (tower volume per mm of height)
```

This gives a continuous reading from one print, where the usual manual method
(blocks at several fixed flush volumes) only gives pass or fail at a few
values. Each direction is measured separately, because dark → light needs far
more flushing than light → dark. And it measures what matters for print
quality: when the plastic laid into a wall stops showing the old colour.

The tool only generates the test. The user reads the towers and enters the
results in the flushing volumes table, with the multiplier at 1. The user
guide is the "Flushing Volume Calibration" page of the OrcaSlicer wiki.

## Supported printers

The test needs a single nozzle that switches filament: an AMS or another
multi-material unit feeding one hotend. When a project prints by object with
more than one object, the prime tower is always off, so every bit of flushing
must come from the printer's change-filament G-code. That G-code receives the
matrix value as `flush_length` / `flush_length_N` (`GCode.cpp`), which is why
the minimum flush is written into the matrix.

| Printer | Supported | Notes |
|---|---|---|
| Single nozzle, change-filament G-code purges `flush_length` (Bambu Lab, and Qidi, Anycubic, Creality and Elegoo profiles that use these variables) | Yes | The minimum flush is purged before each tower. |
| Single nozzle, change-filament G-code ignores `flush_length` (for example profiles that purge only on the prime tower) | Partly | The test is generated, but the minimum flush is never purged, so the towers start unprimed and may fail to stick. |
| More than one nozzle (tool changers, IDEX, dual-nozzle printers) | No | Flushing is per nozzle there. The tool refuses to generate the test. |
| Belt printers | No | The tool refuses to generate the test. |

## Generated project

`Plater::calib_flush_volume()` builds the test in a new project named
"Flushing volume". `FlushVolume_Test_Dlg` (`calib_dlg.cpp`) collects the
filaments, the flushing volume each tower holds (default 500 mm³) and the
minimum flush (default 100 mm³).

### Objects and order

Each object has two parts loaded from `resources/calib/flush_volume/`:
`base.stl` (18.8 × 11 × 1.6 mm, printed in the "from" filament, which is the
object's filament) and `tower.stl` (17.25 × 9.4 mm, printed in the "to"
filament). The tower mesh is scaled along Z so that it holds the requested
flushing volume. Its volume per mm of height is read from the mesh, not
hard-coded, and does not change with the scaling.

Objects are added in print order, and by-object printing follows the object
list order. `flush_volume_change_sequence()` (`calib.hpp`) orders the changes
as an Eulerian circuit of the complete directed graph on the selected
filaments, built with Hierholzer's algorithm. Every ordered pair appears once,
and each object's base is in the filament the previous tower ended with. The
only filament changes are the ones under test, at the foot of each tower.

### Layout

`flush_volume_grid_cells()` (`calib.hpp`) places the objects on a grid centred
on the bed, row by row from the front left. The pitch is the object footprint
plus the extruder clearance radius, so by-object printing does not report
collisions. Cells that leave the printable area or touch a bed exclusion area
are skipped. Objects that do not fit on one plate go to new plates, up to the
project's plate limit.

The total height (base plus tower) must stay under both the extruder clearance
height to rod and the printable height. Otherwise a later object could not be
printed next to a taller one. The tool refuses a flushing volume that would
exceed it.

### Settings

The test changes these settings in the edited print preset:

| Setting | Value | Why |
|---|---|---|
| `print_sequence` | By object | Each tower prints in full right after its base. |
| `enable_prime_tower` | off | Flushing must not happen on a prime tower. |
| `flush_into_infill`, `flush_into_objects`, `flush_into_support` | off | Flushing must not go into other objects. |
| `enable_wrapping_detection` | off | Its reserved bed area would collide with the grid. |
| `skirt_loops` | 0 | A skirt would need extra room between objects. |
| `timelapse_type` | Traditional | Smooth timelapse is not allowed when printing by object. |

In the project config, every value of `flush_volumes_matrix` is set to the
minimum flush, and `flush_multiplier` and `flush_multiplier_fast` are set to
1, so each change flushes exactly the minimum flush.

## Limitations

- **Purge flow differs from print flow.** The flush is purged at the
  filament's flush speed, while the tower prints at normal print speed. How
  cleanly a colour clears depends somewhat on flow, so the test measures
  mixing during printing, not during purging.
- **The minimum flush and the gradient are added.** This assumes nothing is
  lost between the purge and the tower, which holds unless the printer cuts or
  retracts filament in between. On printers with a `grab_length`, part of the
  flush is extruded during filament detection instead of the purge. The total
  extruded volume is the same.
- **Reading the gradient is a judgement.** Light and translucent colours fade
  gradually, so two readings of one tower can differ by a millimetre or two.
- **The test grows with the square of the filament count.** N filaments give
  N × (N − 1) objects.
- **The results are entered by hand.** The tool does not write the measured
  volumes into `flush_volumes_matrix`.
