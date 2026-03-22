package main

import (
	"bufio"
	"os"
	"strconv"
	"strings"
)

// verifyError reports a semantic or syntax error while parsing an OBJ file.
type verifyError struct {
	message string
}

// Error implements the error interface.
func (e *verifyError) Error() string {
	return "Verify Error: " + e.message
}

// scanObjFile reads path as a minimal Wavefront OBJ containing only "v" vertex
// lines (three coordinates) and "f" face lines (three 1-based vertex indices
// forming a triangle). Indices in the returned objFile are 0-based.
func scanObjFile(path string) (objFile, error) {
	obj := objFile{}

	// reading .obj file
	file, err := os.Open(path)
	if err != nil {
		return objFile{}, err
	}
	defer file.Close()

	scanner := bufio.NewScanner(file)

	lineCount := 0

	for scanner.Scan() {
		lineCount++

		line := scanner.Text()
		words := strings.Split(line, " ")

		// verify if the line is a valid vertex or face
		switch words[0] {
		case "v":
			if len(words) != 4 {
				return objFile{}, &verifyError{message: "Invalid vertex format at line " + strconv.Itoa(lineCount)}
			}
			x, err := strconv.ParseFloat(words[1], 64)
			if err != nil {
				return objFile{}, &verifyError{message: "Invalid vertex format at line " + strconv.Itoa(lineCount)}
			}
			y, err := strconv.ParseFloat(words[2], 64)
			if err != nil {
				return objFile{}, &verifyError{message: "Invalid vertex format at line " + strconv.Itoa(lineCount)}
			}
			z, err := strconv.ParseFloat(words[3], 64)
			if err != nil {
				return objFile{}, &verifyError{message: "Invalid vertex format at line " + strconv.Itoa(lineCount)}
			}
			obj.vertices = append(obj.vertices, vertex{x: x, y: y, z: z})
		case "f":
			if len(words) != 4 {
				return objFile{}, &verifyError{message: "Invalid face format at line " + strconv.Itoa(lineCount) +
					": expected 3 vertices, got " + strconv.Itoa(len(words)-1)}
			}
			v1, err := strconv.Atoi(words[1])
			if err != nil {
				return objFile{}, &verifyError{message: "Invalid face format at line " + strconv.Itoa(lineCount) +
					": expected vertex index, got " + words[2]}
			}
			v2, err := strconv.Atoi(words[2])
			if err != nil {
				return objFile{}, &verifyError{message: "Invalid face format at line " + strconv.Itoa(lineCount) +
					": expected vertex index, got " + words[3]}
			}
			v3, err := strconv.Atoi(words[3])
			if err != nil {
				return objFile{}, &verifyError{message: "Invalid face format at line " + strconv.Itoa(lineCount) +
					": expected vertex index, got " + words[4]}
			}
			obj.faces = append(obj.faces, face{vertices: [3]int{v1 - 1, v2 - 1, v3 - 1}})
		default:
			return objFile{}, &verifyError{message: "Invalid line type at line " + strconv.Itoa(lineCount) +
				": expected v or f, got " + words[0]}
		}
	}

	return obj, nil
}

// writeObjFile writes obj to path as OBJ text: all "v" lines first (six decimal
// places), then "f" lines with four 1-based indices per quad. Uses a 2 MiB buffer
// for the underlying writer and strconv for formatting.
func writeObjFile(path string, obj *objFile4) error {
	file, err := os.Create(path)
	if err != nil {
		return err
	}
	defer file.Close()

	writer := bufio.NewWriterSize(file, 2*1024*1024)

	var buf []byte = make([]byte, 0, 2*1024*1024)
	for _, v := range obj.vertices {
		buf = append(buf, "v "...)
		buf = strconv.AppendFloat(buf, v.x, 'f', 6, 64)
		buf = append(buf, ' ')
		buf = strconv.AppendFloat(buf, v.y, 'f', 6, 64)
		buf = append(buf, ' ')
		buf = strconv.AppendFloat(buf, v.z, 'f', 6, 64)
		buf = append(buf, '\n')
	}
	_, err = writer.Write(buf)
	if err != nil {
		return err
	}
	for _, face := range obj.faces {
		buf = append(buf, "f "...)
		buf = strconv.AppendInt(buf, int64(face.vertices[0]+1), 10)
		buf = append(buf, ' ')
		buf = strconv.AppendInt(buf, int64(face.vertices[1]+1), 10)
		buf = append(buf, ' ')
		buf = strconv.AppendInt(buf, int64(face.vertices[2]+1), 10)
		buf = append(buf, ' ')
		buf = strconv.AppendInt(buf, int64(face.vertices[3]+1), 10)
		buf = append(buf, '\n')
	}
	_, err = writer.Write(buf)
	if err != nil {
		return err
	}
	writer.Flush()
	return nil
}
