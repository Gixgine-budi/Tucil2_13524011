package main

import "math"

// vertex is a point or direction in 3D space. The same type is used for positions
// and for vector arithmetic (difference, cross product, etc.).
type vertex struct {
	x float64
	y float64
	z float64
}

// face is a triangle referencing three vertices by 0-based indices into an
// objFile.vertices slice. Wavefront OBJ uses 1-based indices when serializing.
type face struct {
	vertices [3]int
}

// face4 is a quadrilateral referencing four vertices by
// 0-based indices into an objFile4.vertices slice. Wavefront OBJ uses 1-based
// indices when serializing.
type face4 struct {
	vertices [4]int
}

// objFile holds triangle mesh data as read from a minimal OBJ input.
type objFile struct {
	vertices []vertex
	faces    []face
}

// objFile4 holds a quad-dominant mesh produced by voxelization (six quads per
// voxel box).
type objFile4 struct {
	vertices []vertex
	faces    []face4
}

// voxel is an axis-aligned cube: corner base and positive edge length size
// along each axis.
type voxel struct {
	base vertex
	size float64
}

// octree is one node of an octree over space. root points to this node's voxel;
// depth is the distance from the top-level subdivided children; fill indicates
// whether geometry may intersect this node; children are the eight octants when
// subdivided.
type octree struct {
	size     float64
	root     *voxel
	depth    int
	fill     bool
	children [8]*octree
}

// sub returns the vector from v2 to v (component-wise difference).
func (v vertex) sub(v2 vertex) vertex {
	return vertex{v.x - v2.x, v.y - v2.y, v.z - v2.z}
}

// add returns the sum of v and v2.
func (v vertex) add(v2 vertex) vertex {
	return vertex{v.x + v2.x, v.y + v2.y, v.z + v2.z}
}

// mul returns v scaled by f.
func (v vertex) mul(f float64) vertex {
	return vertex{v.x * f, v.y * f, v.z * f}
}

// dot returns the dot product of v and v2.
func (v vertex) dot(v2 vertex) float64 {
	return v.x*v2.x + v.y*v2.y + v.z*v2.z
}

// cross returns the cross product of v and v2.
func (v vertex) cross(v2 vertex) vertex {
	return vertex{v.y*v2.z - v.z*v2.y, v.z*v2.x - v.x*v2.z, v.x*v2.y - v.y*v2.x}
}

// length returns the Euclidean norm of v.
func (v vertex) length() float64 {
	return math.Sqrt(v.squaredLength())
}

// squaredLength returns the dot product of v with itself without a square root.
func (v vertex) squaredLength() float64 {
	return v.x*v.x + v.y*v.y + v.z*v.z
}

// normalize returns a unit vector in the direction of v, or the zero vector if v
// has zero length.
func (v vertex) normalize() vertex {
	if v.length() == 0 {
		return vertex{0, 0, 0}
	}
	return v.mul(1 / v.length())
}

// crossProduct returns the cross product of v1 and v2.
func crossProduct(v1 vertex, v2 vertex) vertex {
	return vertex{v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x}
}

// dotProduct returns the dot product of v1 and v2.
func dotProduct(v1 vertex, v2 vertex) float64 {
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z
}

// getAllVertices returns the eight corners of vx in fixed order (binary
// encoding of local coordinates from 000 through 111).
func (vx voxel) getAllVertices() []vertex {
	return []vertex{
		// 0: 0, 0, 0
		// 1: 0, 0, 1
		// 2: 0, 1, 0
		// 3: 0, 1, 1
		// 4: 1, 0, 0
		// 5: 1, 0, 1
		// 6: 1, 1, 0
		// 7: 1, 1, 1
		vx.base,
		vx.base.add(vertex{0, 0, vx.size}),
		vx.base.add(vertex{0, vx.size, 0}),
		vx.base.add(vertex{0, vx.size, vx.size}),
		vx.base.add(vertex{vx.size, 0, 0}),
		vx.base.add(vertex{vx.size, 0, vx.size}),
		vx.base.add(vertex{vx.size, vx.size, 0}),
		vx.base.add(vertex{vx.size, vx.size, vx.size}),
	}
}

// getAllFaces returns the six axis-aligned quad faces of vx using local vertex
// indices 0 through 7 matching getAllVertices order.
func (vx voxel) getAllFaces() []face4 {
	return []face4{
		{vertices: [4]int{0, 1, 3, 2}},
		{vertices: [4]int{0, 1, 5, 4}},
		{vertices: [4]int{0, 2, 6, 4}},
		{vertices: [4]int{2, 3, 7, 6}},
		{vertices: [4]int{1, 3, 7, 5}},
		{vertices: [4]int{4, 5, 7, 6}},
	}
}

// getAllAxes returns three edge direction vectors (+X, +Y, +Z) of vx with
// magnitude size, used as SAT axes against triangle edges.
func (vx voxel) getAllAxes() []vertex {
	return []vertex{
		{vx.size, 0, 0},
		{0, vx.size, 0},
		{0, 0, vx.size},
	}
}

// subdivide splits vx into eight equal child voxels (standard octree octants).
func (vx voxel) subdivide() [8]voxel {
	size := vx.size / 2

	return [8]voxel{
		// 0: 0, 0, 0
		// 1: 0, 0, 1
		// 2: 0, 1, 0
		// 3: 0, 1, 1
		// 4: 1, 0, 0
		// 5: 1, 0, 1
		// 6: 1, 1, 0
		// 7: 1, 1, 1
		{base: vx.base, size: size},
		{base: vx.base.add(vertex{0, 0, size}), size: size},
		{base: vx.base.add(vertex{0, size, 0}), size: size},
		{base: vx.base.add(vertex{0, size, size}), size: size},
		{base: vx.base.add(vertex{size, 0, 0}), size: size},
		{base: vx.base.add(vertex{size, 0, size}), size: size},
		{base: vx.base.add(vertex{size, size, 0}), size: size},
		{base: vx.base.add(vertex{size, size, size}), size: size},
	}
}

// addIndex returns a copy of f4 with each vertex index increased by index
// (global offset when appending a voxel's local indices).
func (f4 face4) addIndex(index int) face4 {
	return face4{vertices: [4]int{f4.vertices[0] + index, f4.vertices[1] + index, f4.vertices[2] + index, f4.vertices[3] + index}}
}
