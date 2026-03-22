# Tucil2_13524011 - Mesh voxelizer (ttov)

## Brief description

This repository is **Tugas Kecil 2** for **IF2211 Strategi Algoritma**. The program **ttov** reads a triangle mesh in [Wavefront OBJ](https://en.wikipedia.org/wiki/Wavefront_.obj_file_format) format, fits an axis-aligned bounding cube around the mesh, and recursively subdivides that cube into an **octree** (8 children per level). At each leaf depth, voxels that intersect the mesh are kept; empty branches are not subdivided further. The result is exported as a **quad-dominant** OBJ mesh (`output4.obj`): each voxel becomes a box of six quadrilateral faces. Shared vertices are deduplicated before writing.

## Requirements and installation

- **Go** toolchain matching the module (see `src/ttov/go.mod`; currently **Go 1.26.1** or compatible).
- No extra libraries beyond the Go standard library.
- Install Go from [https://go.dev/dl/](https://go.dev/dl/) and ensure `go` is on your `PATH`.

Optional:

- **GNU Make** (Linux / WSL) if you use the provided `Makefile`.
- **Windows**: use `build.bat` (no Make required).

The repo uses a **Go workspace** (`go.work`) so builds should be run from the **repository root** (where `go.work` lives).

## How to compile

From the **root** of this repository (folder containing `go.work`):

**Linux / WSL (Makefile)**

```bash
make          # or: make build
```

Binary output: `bin/ttov`

**Windows (batch)**

```cmd
build.bat build
```

Binary output: `bin\ttov.exe`

**Manual (any OS, from root)**

```bash
mkdir -p bin
go build -o bin/ttov ./src/ttov
```

On Windows, use `bin\ttov.exe` if you prefer an explicit `.exe` suffix.

**Clean build artifacts**

```bash
make clean          # Linux / WSL
build.bat clean     # Windows
```

## How to run and use

1. Build the program (see above).
2. Run the executable **from the directory where you want `output4.obj` to be created** (the program writes `output4.obj` in the current working directory).

**Linux / WSL**

```bash
./bin/ttov
```

**Windows**

```cmd
bin\ttov.exe
```

3. When prompted:

   - **Enter file path:** path to an input `.obj` file. The reader expects lines starting with `v` (three coordinates) and `f` (three 1-based vertex indices per triangle). Other line types are rejected.
   - **Enter subdivision level:** non-negative integer. The octree subdivides up to that depth (each level multiplies the finest cell count by up to 8 along each branch that still intersects the mesh). Higher values produce finer voxels and much larger output files.

4. The program prints timings and statistics, then writes **`output4.obj`** next to your current working directory.

## Input / output summary

|  Item  |                                 Detail                                 |
|--------|------------------------------------------------------------------------|
| Input  | OBJ with `v` and `f` lines only (triangles)                            |
| Output | `output4.obj` - vertices (`v`) then quad faces (`f` with four indices) |

## Author / identity

**Muhammad Iqbal Raihan** -- 13524011

Module path: `github.com/Gixgine-budi/Tucil2_13524011/src/ttov`
