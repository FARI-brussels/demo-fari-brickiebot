# CAD

Parametric CAD models for the Brickiebot crane, carriage, clamp, and press-fit brick box.

The CAD files are Python scripts built with `build123d`. They are meant to be opened and run from VS Code with the OCP CAD Viewer extension so that calls like `show(...)` display the model while developing.

## Folder Layout

- `crane.py` - assembles the full crane and exports meshes used by the simulator.
- `parts.py` - shared mechanical parts: pulleys, feet, carriage plates, robot carriage, and endstop holder.
- `utils.py` - helper classes for steppers, extrusions, rollers, spacers, and mounting geometry.
- `input_models/` - source STEP/STL/IGS models imported by the Python scripts.
- `parts/` - generated STL outputs for crane and mechanical subparts.
- `clamp/` - clamp and linkage CAD scripts plus generated STLs.
- `brick/` - press-fit brick/box generator, logo SVGs, and generated STLs.
- `stl/` - external or reference STL files.

## Install VS Code CAD Tooling

Install the VS Code extensions:

2. Install `OCP CAD Viewer` from the VS Code Marketplace.

Recommended Python environment from the repository root:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install build123d ocp-vscode bd-warehouse
```

On Windows PowerShell, use:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install build123d ocp-vscode bd-warehouse
```

In VS Code, select the same interpreter:

1. Open the command palette.
2. Run `Python: Select Interpreter`.
3. Pick the `.venv` interpreter created above.

Alternative: after installing the OCP CAD Viewer extension, open its sidebar and use `Quickstart build123d`. That installs the viewer dependencies for the selected Python environment. This project still needs `bd-warehouse`, so install it afterwards if the scripts cannot import `bd_warehouse`:

```bash
python -m pip install bd-warehouse
```

## Running The CAD Scripts In VS Code

Open the repository folder in VS Code:

```bash
code .
```

Start the OCP CAD Viewer from the VS Code sidebar. Then open a CAD script and run it with `Run > Run Without Debugging` or `Ctrl+F5`.

Most files contain `show(...)` calls. When the script runs successfully, the part or assembly appears in the OCP CAD Viewer panel.

For this project, run scripts from the `CAD/` folder when generating local outputs:

```bash
cd CAD
python parts.py
python crane.py
python brick/box.py
```

`crane.py` exports the simulator meshes to `../simulation/assets/`.


The scripts resolve imported CAD models from `CAD/input_models/` Required source models include:

- `Nema17.step`
- `gt2-80tooth-8mm-shaft-6mm-belt.stl`
- `Motedis Profile 20x20 B-Type slot 6.stp`
- `Motedis Profile 20x40 B-Type slot 6.stp`
- `Motedis_roller_29mm_slot6.STEP`
- `myCobotMK1.IGS`

Keep imported source models in `input_models/`. Keep generated printable parts in `parts/`, `clamp/`, `brick/`, or `stl/` depending on their subsystem.

## Output Map

| Script | Main outputs |
| --- | --- |
| `parts.py` | `parts/pulley.stl`, `parts/crane_plate*.stl`, `parts/crane_carriage_v2.stl`, `leg_plate*.stl` |
| `crane.py` | `../simulation/assets/rails.stl`, `crane.stl`, `end_effector.stl`, `crane_body.stl`, `pulley.stl`, `robot_carriage.stl` |
| `brick/box.py` | `brick/a.stl`, `b.stl`, `c.stl`, `d.stl`, `e_fari.stl`, `fari.stl` |
| `clamp/clamp.py` | `latte*.stl`, `jaw*.stl`, `bent_stripe.stl`, `stripe_jaw.stl`, `clamp_fixture.stl` |

## Notes

- The files are notebook-style scripts with `# %%` cells. They can be run as normal Python files or cell-by-cell in VS Code.
- Some scripts export files immediately when imported or run.
- `clamp/clamp.py` currently references `jaw` in the assembly section, but the defined jaw variables are `jaw1` and `jaw2`. Run the earlier export cells first or clean up that assembly block before expecting the whole script to run end-to-end.
- The generated STL files can be large and may change when scripts are rerun.

## References

- build123d installation: https://build123d.readthedocs.io/en/stable/installation.html
- build123d external viewers: https://build123d.readthedocs.io/en/stable/external.html
- OCP CAD Viewer for VS Code: https://github.com/bernhard-42/vscode-ocp-cad-viewer
