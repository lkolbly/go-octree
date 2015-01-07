package main

import "./octree"
import "fmt"
import "math"
import "math/rand"
import "time"

type Foo struct {
	X int
}

func printTree(o octree.Octree) {
	fmt.Println()
	fmt.Println("Bounds:",o.Bounds)
	queue := []octree.OctreeNode{o.Root}
	for len(queue)>0 {
		fmt.Println(len(queue[0].Values))
		if queue[0].Valid {
			var i int
			for i=0; i<8; i++ {
				if queue[0].Children[i].Valid {
					fmt.Println(i)
				}
				queue = append(queue, queue[0].Children[i])
			}
		}
		queue = queue[1:]
	}
	fmt.Println()
}

type Point struct {
	Value Foo
	X float64
	Y float64
	Z float64
}

func getClosest(points []Point, x,y,z float64) (Point, float64) {
	closest := points[0]
	closest_dist := points[0].X + points[0].Y + points[0].Z
	for i := range points {
		dx := points[i].X - x
		dy := points[i].Y - y
		dz := points[i].Z - z
		dist := math.Sqrt(dx*dx+dy*dy+dz*dz)
		//fmt.Println(i,dist,points[i])
		if dist < closest_dist {
			closest_dist = dist
			closest = points[i]
		}
	}
	return closest, closest_dist
}

func timingTests(ot octree.SpatialDatabase, n int) {
	//ot := octree.NewOctree()

	// Add a bunch of values
	start := time.Now()
	var i int
	for i=0; i<n; i++ {
		x := rand.Float64() * 2.0 - 1.0
		y := rand.Float64() * 2.0 - 1.0
		z := rand.Float64() * 2.0 - 1.0
		ot.Add(Foo{i}, x,y,z)
	}
	end := time.Now()
	fmt.Println("Took",end.Sub(start),"to add",n,"elements")

	// Now do a bunch of nearest-neighbor searches
	start = time.Now()
	for i=0; i<n; i++ {
		x := rand.Float64() * 2.0 - 1.0
		y := rand.Float64() * 2.0 - 1.0
		z := rand.Float64() * 2.0 - 1.0
		ot.FindNearest(x,y,z)
		//fmt.Println("OK")
	}
	end = time.Now()
	fmt.Println("Took",end.Sub(start),"to do",n,"nearest-neighbor searches")
}

func main() {
	//v1 := Foo{1}
	//v2 := Foo{2}
	ot := octree.NewOctree()//octree.Octree{}
	//printTree(ot)

	/*ot.Add(v1, 0.4, 0.4, 0.4)
	printTree(ot)
	ot.Add(v2, -0.1, 0.2, 0.3)
	printTree(ot)*/

	// Add a bunch of random values...
	values := []Point{}
	var i int
	for i=0; i<100; i++ {
		x := rand.Float64() * 2.0 - 1.0
		y := rand.Float64() * 2.0 - 1.0
		z := rand.Float64() * 2.0 - 1.0
		ot.Add(Foo{i}, x, y, z)
		values = append(values, Point{Foo{i}, x, y, z})
		//fmt.Println(i)
	}

	// Check the find nearest...
	for i=0; i<100; i++ {
		x := rand.Float64() * 2.0 - 1.0
		y := rand.Float64() * 2.0 - 1.0
		z := rand.Float64() * 2.0 - 1.0
		v_pnt, dist := ot.FindNearest(x,y,z)
		v := v_pnt.GetElement().(Foo)
		v2, dist2 := getClosest(values, x,y,z)
		if v.X != v2.Value.X {
			fmt.Println("Didn't match:",v,v2.Value)
			fmt.Println("Point:",x,y,z)
			fmt.Println("dist:",dist)
			fmt.Println("dist2:",dist2)
			fmt.Println("Our pos:",v2.X,v2.Y,v2.Z)
			fmt.Println("Their pos:",v_pnt)
		} else {
			//fmt.Println("****OK")
		}
	}

	// Check adding points outside <-1,1,-1,1,-1,1>
	ot.Add(Foo{1000}, -2.0, 2.0, -2.0)
	v_pnt, _ := ot.FindNearest(-2.0,2.0,-1.9)
	fmt.Println(v_pnt)

	//return
	n := 64
	for i=0; i<15; i++ {
		ot = octree.NewOctree()
		timingTests(ot, n)
		lt := octree.NewList()
		timingTests(lt, n)
		n *= 2
		fmt.Println()
	}
}
