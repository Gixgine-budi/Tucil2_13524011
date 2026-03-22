package main

import (
	"container/list"
	"math"
)

// checkCollision reports whether the axis-aligned voxel vx intersects the
// triangular face fc in obj. It uses an AABB test, a plane straddle test, and
// separating-axis tests on edge cross products. Touching-only contact is not
// treated as a collision.
func checkCollision(vx voxel, fc face, obj *objFile) bool {
	// 1. Bounding box test
	fc_min_x := math.Min(obj.vertices[fc.vertices[0]].x, math.Min(obj.vertices[fc.vertices[1]].x, obj.vertices[fc.vertices[2]].x))
	fc_min_y := math.Min(obj.vertices[fc.vertices[0]].y, math.Min(obj.vertices[fc.vertices[1]].y, obj.vertices[fc.vertices[2]].y))
	fc_min_z := math.Min(obj.vertices[fc.vertices[0]].z, math.Min(obj.vertices[fc.vertices[1]].z, obj.vertices[fc.vertices[2]].z))
	fc_max_x := math.Max(obj.vertices[fc.vertices[0]].x, math.Max(obj.vertices[fc.vertices[1]].x, obj.vertices[fc.vertices[2]].x))
	fc_max_y := math.Max(obj.vertices[fc.vertices[0]].y, math.Max(obj.vertices[fc.vertices[1]].y, obj.vertices[fc.vertices[2]].y))
	fc_max_z := math.Max(obj.vertices[fc.vertices[0]].z, math.Max(obj.vertices[fc.vertices[1]].z, obj.vertices[fc.vertices[2]].z))

	vx_max_x := vx.base.x + vx.size
	vx_max_y := vx.base.y + vx.size
	vx_max_z := vx.base.z + vx.size

	// 1. AABB test
	if vx_max_x <= fc_min_x || vx.base.x >= fc_max_x ||
		vx_max_y <= fc_min_y || vx.base.y >= fc_max_y ||
		vx_max_z <= fc_min_z || vx.base.z >= fc_max_z {
		return false
	}

	// 2. Plane intersection test
	// A = fc.vertices[0], B = fc.vertices[1], C = fc.vertices[2]
	// plane_normal = (B - A) x (C - A)
	plane_normal := obj.vertices[fc.vertices[1]].sub(obj.vertices[fc.vertices[0]]).cross(obj.vertices[fc.vertices[2]].sub(obj.vertices[fc.vertices[0]]))
	vx_vertices := vx.getAllVertices()
	point_distance := make([]float64, 0)
	for _, vx_vertex := range vx_vertices {
		// point_distance = (vx_vertex - A) * plane_normal
		point_distance = append(point_distance, vx_vertex.sub(obj.vertices[fc.vertices[0]]).dot(plane_normal))
	}

	collision := false
	for _, one_distance := range point_distance {
		if one_distance*point_distance[0] < 0 {
			collision = true
		}
	}

	if !collision {
		return false
	}

	// 3. Edge-to-edge test
	for i := range 3 {
		// edge1 = (A - B)
		edge1 := obj.vertices[fc.vertices[i]].sub(obj.vertices[fc.vertices[(i+1)%3]])

		for _, edge2 := range vx.getAllAxes() {
			// axis = (A - B) x edge2
			axis := crossProduct(edge1, edge2)

			// skip degenerate axis (parallel edges give zero cross product)
			if axis.squaredLength() < 1e-20 {
				continue
			}

			// project vertices onto axis using scalar
			vx_min := vx_vertices[0].dot(axis)
			vx_max := vx_min
			for _, vx_vertex := range vx_vertices {
				s := vx_vertex.dot(axis)
				if s < vx_min {
					vx_min = s
				}
				if s > vx_max {
					vx_max = s
				}
			}

			fc_min := obj.vertices[fc.vertices[0]].dot(axis)
			fc_max := fc_min
			for _, fc_vertex := range fc.vertices {
				s := obj.vertices[fc_vertex].dot(axis)
				if s < fc_min {
					fc_min = s
				}
				if s > fc_max {
					fc_max = s
				}
			}

			// intervals overlap iff vx_max > fc_min AND vx_min < fc_max
			// if separated on this axis, we found a separating axis -> no collision
			if vx_max <= fc_min || vx_min >= fc_max {
				return false
			}
			// overlap on this axis, continue checking remaining axes
		}
	}

	// overlap on all axes -> collision
	return true
}

// exploreObj builds an octree over the mesh bounding cube by subdividing into
// eight children per level in parallel (goroutines per branch). subdivision_limit
// is the maximum depth at which leaf nodes record whether any triangle intersects
// their voxel.
func exploreObj(source *objFile, subdivision_limit int) octree {
	containing_cube := findContainingCube(source)
	t := octree{root: &containing_cube, depth: 0, fill: true}
	ch := make(chan *octree, 8)

	for _, vx := range containing_cube.subdivide() {
		go parallelExploreObj(source, &source.faces, vx, 1, subdivision_limit, ch)
	}

	for i := range 8 {
		t.children[i] = <-ch
	}

	return t
}

// parallelExploreObj tests containing_cube against faces in fc, sets fill flags,
// and either returns a leaf or spawns eight recursive explorations. Results are
// sent on ch when the subtree for this cube is complete.
func parallelExploreObj(source *objFile, fc *[]face, containing_cube voxel, depth int, subdivision_limit int, ch chan *octree) {
	t := octree{root: &containing_cube, depth: depth}
	collided_fc := make([]face, 0)

	for _, face := range *fc {
		if checkCollision(containing_cube, face, source) {
			collided_fc = append(collided_fc, face)
		}
	}

	if depth >= subdivision_limit {
		t.fill = len(collided_fc) > 0
		ch <- &t
		return
	}

	if len(collided_fc) > 0 {
		t.fill = true
	} else {
		t.fill = false
		ch <- &t
		return
	}

	sub_ch := make(chan *octree, 8)
	for _, vx := range containing_cube.subdivide() {
		go parallelExploreObj(source, &collided_fc, vx, depth+1, subdivision_limit, sub_ch)
	}

	for i := range 8 {
		t.children[i] = <-sub_ch
	}

	ch <- &t
}

// findContainingCube returns the smallest axis-aligned cube that encloses all
// vertices of source. The cube side length is the maximum of the axis-aligned
// bounding box spans in x, y, and z.
func findContainingCube(source *objFile) voxel {
	var (
		x_min, x_max float64
		y_min, y_max float64
		z_min, z_max float64
	)

	x_min = source.vertices[0].x
	x_max = source.vertices[0].x
	y_min = source.vertices[0].y
	y_max = source.vertices[0].y
	z_min = source.vertices[0].z
	z_max = source.vertices[0].z

	for _, vertex := range source.vertices {
		if vertex.x < x_min {
			x_min = vertex.x
		}
		if vertex.x > x_max {
			x_max = vertex.x
		}
		if vertex.y < y_min {
			y_min = vertex.y
		}
		if vertex.y > y_max {
			y_max = vertex.y
		}
		if vertex.z < z_min {
			z_min = vertex.z
		}
		if vertex.z > z_max {
			z_max = vertex.z
		}
	}

	return voxel{base: vertex{x: x_min, y: y_min, z: z_min}, size: math.Max(x_max-x_min, math.Max(y_max-y_min, z_max-z_min))}
}

// octToObj walks oct in breadth-first order, appends each filled leaf at depth
// maxDepth as a box mesh into dest, and returns per-depth node counts,
// per-depth counts of children skipped when a node is empty, and the number of
// leaf voxels emitted.
func octToObj(oct *octree, maxDepth int, dest *objFile4) ([]int, []int, int) {
	node_count := make([]int, maxDepth+1)
	no_explore_node := make([]int, maxDepth+1)
	voxel_count := 0
	node_count[0] = 0
	no_explore_node[0] = 0

	queue := list.New()
	queue.PushBack(oct)

	for queue.Len() > 0 {
		oct := queue.Remove(queue.Front()).(*octree)
		if oct.fill && oct.depth == maxDepth {
			addVoxelToObj(*oct.root, dest)
			voxel_count++
		}

		node_count[oct.depth]++
		if !oct.fill && oct.depth < maxDepth {
			no_explore_node[oct.depth+1] += 8
		}

		for _, child := range oct.children {
			if child != nil {
				queue.PushBack(child)
			}
		}
	}

	return node_count, no_explore_node, voxel_count
}

// addVoxelToObj appends the six quad faces and eight corner vertices of voxel to
// dest. Face indices reference vertices appended in this call using the current
// length of dest.vertices as a base offset.
func addVoxelToObj(voxel voxel, dest *objFile4) {
	for _, face := range voxel.getAllFaces() {
		dest.faces = append(dest.faces, face.addIndex(len(dest.vertices)))
	}
	for _, vertex := range voxel.getAllVertices() {
		dest.vertices = append(dest.vertices, vertex)
	}
}

// compactObj4 returns a new objFile4 with duplicate vertices merged and face
// indices rewritten to point at the unique vertex list. Face vertex indices in
// obj must refer to positions in obj.vertices.
func compactObj4(obj *objFile4) *objFile4 {
	new_obj := objFile4{vertices: make([]vertex, 0), faces: make([]face4, 0)}

	unique_vert := make(map[vertex]int)

	for _, vertex := range obj.vertices {
		if _, exists := unique_vert[vertex]; !exists {
			unique_vert[vertex] = len(unique_vert)
			new_obj.vertices = append(new_obj.vertices, vertex)
		}
	}

	for _, face := range obj.faces {
		new_obj.faces = append(new_obj.faces, face4{vertices: [4]int{
			unique_vert[obj.vertices[face.vertices[0]]],
			unique_vert[obj.vertices[face.vertices[1]]],
			unique_vert[obj.vertices[face.vertices[2]]],
			unique_vert[obj.vertices[face.vertices[3]]]}})
	}

	return &new_obj
}
