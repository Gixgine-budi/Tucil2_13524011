// Package main implements a command-line tool that voxelizes a triangle mesh from
// a Wavefront OBJ file, builds an octree of axis-aligned cubes, deduplicates
// vertices, and writes the resulting quad-dominant mesh to output4.obj.
package main

import (
	"bufio"
	"fmt"
	"os"
	"strconv"
	"strings"
	"time"
)

// main reads an input OBJ path and subdivision depth from stdin, runs the
// voxelization pipeline, prints timing and statistics, and writes output4.obj.
func main() {
	// stdin input file path
	cin := bufio.NewReader(os.Stdin)
	fmt.Print("Enter file path: ")
	text, _ := cin.ReadString('\n')
	text = strings.TrimSpace(text)
	fmt.Println()

	cin2 := bufio.NewScanner(os.Stdin)
	fmt.Println("Subdivision level is the number of times the cube is subdivided into 8 smaller cubes.")
	fmt.Println("For example, a subdivision level of 3 means the cube is subdivided into 8^3 = 512 smaller cubes.")
	fmt.Print("Enter subdivision level: ")
	cin2.Scan()
	subdivision_limit, err := strconv.Atoi(cin2.Text())
	if err != nil || subdivision_limit < 0 {
		fmt.Println("Invalid subdivision level")
		return
	}

	fmt.Println()
	fmt.Println()
	fmt.Println("============================================================")

	// scan and verify the .obj file according to spec
	start_time := time.Now()
	abs_time := start_time

	obj, err := scanObjFile(text)
	if err != nil {
		fmt.Println(err.Error())
		return
	}

	fmt.Println("Scanning .obj file completed")
	end_time := time.Now()
	fmt.Printf("Time taken to scan obj file: %s\n\n", end_time.Sub(start_time))
	start_time = end_time

	// generate voxel from obj file
	oct := exploreObj(&obj, subdivision_limit)

	fmt.Println("Octree generation completed")
	end_time = time.Now()
	fmt.Printf("Time taken to voxelize obj file: %s\n\n", end_time.Sub(start_time))
	start_time = end_time

	// generate obj file from octree
	end_time = time.Now()
	obj4 := objFile4{}
	node_count, no_explore_node, voxel_count := octToObj(&oct, subdivision_limit, &obj4)

	fmt.Println("Octree to OBJ data type conversion completed")
	end_time = time.Now()
	fmt.Printf("Time taken to convert octree to OBJ data type: %s\n\n", end_time.Sub(start_time))

	start_time = end_time
	obj4 = *compactObj4(&obj4)
	fmt.Println("OBJ data has been succesfully compacted")
	end_time = time.Now()
	fmt.Printf("Time taken to compact OBJ data: %s\n\n", end_time.Sub(start_time))

	fmt.Printf("Number of voxels: %d\n", voxel_count)
	fmt.Printf("Number of vertices: %d\n", len(obj4.vertices))
	fmt.Printf("Number of faces: %d\n", len(obj4.faces))
	fmt.Println()

	// displays how many octree nodes are generated
	fmt.Println("Number of nodes at each depth:")
	for i := 0; i <= subdivision_limit; i++ {
		fmt.Printf("%d\t: %d\n", i+1, node_count[i])
	}
	fmt.Println()
	fmt.Println("Number of nodes not explored at each depth:")
	for i := 0; i <= subdivision_limit; i++ {
		fmt.Printf("%d\t: %d\n", i+1, no_explore_node[i])
	}
	fmt.Println()

	start_time = end_time

	// write obj file to file
	err = writeObjFile("output4.obj", &obj4)
	if err != nil {
		fmt.Println(err.Error())
		fmt.Println("OBJ file writing failed")
		return
	}
	fmt.Println()
	fmt.Println("OBJ file writing completed")
	end_time = time.Now()
	fmt.Printf("Time taken to write obj file: %s\n", end_time.Sub(start_time))
	fmt.Println()
	fmt.Printf("Total time taken: %s\n", end_time.Sub(abs_time))
	fmt.Println()
}
