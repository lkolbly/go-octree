package octree

import "fmt"
import "math"

type SpatialValue interface {
	GetElement() interface{}
}

type SpatialDatabase interface {
	Add(interface{}, float64, float64, float64)
	FindNearest(float64, float64, float64) (SpatialValue, float64)
	IterShell(x,y,z float64, r_inner float64, r_outer float64) <-chan SpatialValue
}

func distance(x1,y1,z1, x2,y2,z2 float64) float64 {
	dx := x2-x1
	dy := y2-y1
	dz := z2-z1
	return math.Sqrt(dx*dx+dy*dy+dz*dz)
}

type Cuboid struct {
	X1, X2, Y1, Y2, Z1, Z2 float64
}

func (this Cuboid) contains(x,y,z float64) bool {
	if x > this.X1 && x < this.X2 && y > this.Y1 && y < this.Y2 && z > this.Z1 && z < this.Z2 {
		return true
	}
	return false
}

func (this Cuboid) intersectsSphereShell(x,y,z,r float64) bool {
	is_inside := distance(x,y,z, this.X1,this.Y1,this.Z1) < r
	if (distance(x,y,z, this.X1,this.Y1,this.X2) < r) != is_inside {
		return true
	}
	if (distance(x,y,z, this.X1,this.Y2,this.X1) < r) != is_inside {
		return true
	}
	if (distance(x,y,z, this.X1,this.Y2,this.X2) < r) != is_inside {
		return true
	}
	if (distance(x,y,z, this.X2,this.Y1,this.X1) < r) != is_inside {
		return true
	}
	if (distance(x,y,z, this.X2,this.Y1,this.X2) < r) != is_inside {
		return true
	}
	if (distance(x,y,z, this.X2,this.Y2,this.X1) < r) != is_inside {
		return true
	}
	if (distance(x,y,z, this.X2,this.Y2,this.X2) < r) != is_inside {
		return true
	}
	return false
}

// Returns the distance to the nearest side
func (this Cuboid) nearestSide(x, y, z float64) float64 {
	v1 := math.Abs(x-this.X1)
	v1 =  math.Min(v1, math.Abs(x-this.X2))

	v2 := math.Abs(y-this.Y1)
	v2 =  math.Min(v2, math.Abs(y-this.Y2))

	v3 :=  math.Abs(z-this.Z1)
	v3 = math.Min(v3, math.Abs(z-this.Z2))

	return math.Min(v1, math.Min(v2, v3))
}

// Returns the index in which the point resides
func (bounds Cuboid) getIndex(x, y, z float64) int {
	idx := 0
	if x > (bounds.X2-bounds.X1)/2.0 + bounds.X1 {
		idx = idx | 1
	}
	if y > (bounds.Y2-bounds.Y1)/2.0 + bounds.Y1 {
		idx = idx | 2
	}
	if z > (bounds.Z2-bounds.Z1)/2.0 + bounds.Z1 {
		idx = idx | 4
	}
	return idx
}

func (this Cuboid) subdivide(idx int) Cuboid {
	bounds := this

	if idx&1 != 0 {
		bounds.X1 = (bounds.X2-bounds.X1)/2.0 + bounds.X1
	} else {
		bounds.X2 = (bounds.X2-bounds.X1)/2.0 + bounds.X1
	}

	if idx&2 != 0 {
		bounds.Y1 = (bounds.Y2-bounds.Y1)/2.0 + bounds.Y1
	} else {
		bounds.Y2 = (bounds.Y2-bounds.Y1)/2.0 + bounds.Y1
	}

	if idx&4 != 0 {
		bounds.Z1 = (bounds.Z2-bounds.Z1)/2.0 + bounds.Z1
	} else {
		bounds.Z2 = (bounds.Z2-bounds.Z1)/2.0 + bounds.Z1
	}
	return bounds
}

type OctreeValue struct {
	//Valid bool
	X, Y, Z float64
	Element interface{}
}

func (this OctreeValue) GetElement() interface{} {
	return this.Element
}

type OctreeNode struct {
	Valid bool
	Parent *OctreeNode
	Children []OctreeNode
	//Value OctreeValue
	Values []OctreeValue //Cannot contain elements if we have valid children
}

func (this *OctreeNode) findNearest(depth int, bounds Cuboid, x,y,z float64) (bool, OctreeValue, float64) {
	if !this.Valid {
		var res OctreeValue
		//fmt.Println("Breaking early")
		return false, res, 0.0
	}
	//defer fmt.Println(depth, "Exiting")
	i := bounds.getIndex(x,y,z)
	new_bounds := bounds.subdivide(i)
	//fmt.Println(depth, "Checking child",i)
	ok, elem, dist := this.Children[i].findNearest(depth+1, new_bounds, x,y,z)

	// Check the distance
	//if ok {
	//fmt.Println(depth, dist,new_bounds.nearestSide(x,y,z),bounds)
//}
	if dist < new_bounds.nearestSide(x,y,z) && ok {
		return true, elem, dist
	}
	//fmt.Println(x,y,z)

	// We need to check the neighbors first
	best_elem := elem
	best_dist := dist
	if this.Children[0].Valid {
	//fmt.Println(depth, "Checking children...")
	for j := range this.Children {
		new_bounds = bounds.subdivide(j)
		//fmt.Println(depth, "Checking child",j)
		ok2, elem2, dist2 := this.Children[j].findNearest(depth+1, new_bounds, x,y,z)
		if ok2 {
			if dist2 < best_dist || !ok {
				best_dist = dist2
				best_elem = elem2
				//fmt.Println("Got best:",best_dist)
			}
			ok = true
		}
	}
	}
	if ok {
		return true, best_elem, best_dist
	}

	// Search our values
	var closest OctreeValue
	closest_dist := (bounds.X2-bounds.X1) * 30.0
	//fmt.Printf("%d Searching %d values\n",depth,len(this.Values))
	for j := range this.Values {
		dx := this.Values[j].X - x
		dy := this.Values[j].Y - y
		dz := this.Values[j].Z - z
		dist = math.Sqrt(dx*dx+dy*dy+dz*dz)
		//fmt.Println(dist, closest_dist)
		if dist < closest_dist {
			closest = this.Values[j]
			closest_dist = dist
		}
	}
	ok = closest_dist < (bounds.X2-bounds.X1)*20.0
	//fmt.Println("ok=",ok)
	//fmt.Println(depth,"Closest:",closest_dist,closest,ok)
	return ok, closest, closest_dist
}

func (this *OctreeNode) iterValue2(is_top bool) <- chan OctreeValue {
	ch := make(chan OctreeValue)
	go func(should_close bool) {
		if this == nil {
			if should_close {
				close(ch)
			}
			return
		}
		if !this.Valid {
			//fmt.Println("Leaving")
			if should_close {
				close(ch)
			}
			return
		}

		var i int
		for i=0; i<8; i++ {
			//fmt.Println("Iterating over child",i)
			for item := range this.Children[i].iterValue2(true) {
				ch <- item
			}
		}
		for i=0; i<len(this.Values); i++ {
			ch <- this.Values[i]
		}
		//fmt.Println(is_top)
		if should_close {
			close(ch)
		}
	}(is_top);
	return ch
}

func (this *OctreeNode) IterValues() <-chan OctreeValue {
	return this.iterValue2(true)
}

func (this *OctreeNode) IterShell(bounds Cuboid, x,y,z float64,
	r_inner float64, r_outer float64, ch chan<- SpatialValue) {
	// If our bounds are entirely outside the shell, leave now
	if !bounds.intersectsSphereShell(x,y,z, r_inner) && !bounds.intersectsSphereShell(x,y,z, r_outer) && !bounds.contains(x,y,z) {
		fmt.Println(bounds)
		return
	}

	// Go through our children
	for i := range this.Children {
		if this.Children[i].Valid {
			this.Children[i].IterShell(bounds.subdivide(i), x,y,z,r_inner,r_outer,ch)
		}
	}

	// Go through our values
	for i := range this.Values {
		dx := this.Values[i].X - x
		dy := this.Values[i].Y - y
		dz := this.Values[i].Z - z
		r := math.Sqrt(dx*dx+dy*dy+dz*dz)
		if r > r_inner && r < r_outer {
			ch <- this.Values[i]
		}
	}
}

func (this *OctreeNode) getParentBounds(cur_bounds Cuboid) Cuboid {
	bounds := cur_bounds
	parent := this.Parent
	if parent == nil {
		return cur_bounds // We're root!
	}

	// Which child am I?
	var i int
	for i=0; i<8; i++ {
		if &parent.Children[i] == this {
			if i&1 != 0 {
				bounds.X1 = 2.0 * (bounds.X1 - bounds.X2 / 2.0)
			} else {
				bounds.X2 = 2.0 * (bounds.X2 - bounds.X1 / 2.0)
			}

			if i&2 != 0 {
				bounds.Y1 = 2.0 * (bounds.Y1 - bounds.Y2 / 2.0)
			} else {
				bounds.Y2 = 2.0 * (bounds.Y2 - bounds.Y1 / 2.0)
			}

			if i&4 != 0 {
				bounds.Z1 = 2.0 * (bounds.Z1 - bounds.Z2 / 2.0)
			} else {
				bounds.Z2 = 2.0 * (bounds.Z2 - bounds.Z1 / 2.0)
			}
			return bounds
		}
	}
	fmt.Println("ERRRORRRORORORORORORORRO!")
	return bounds
}

type Octree struct {
	Root OctreeNode
	Bounds Cuboid
}

func NewOctree() SpatialDatabase {
	return &Octree{
		OctreeNode{true,nil,make([]OctreeNode, 8),[]OctreeValue{}},
		Cuboid{-1.0,1.0,-1.0,1.0,-1.0,1.0},
	}
}

func (this *Octree) getNode(x, y, z float64) (*OctreeNode,Cuboid) {
	// Iterate down
	n := &this.Root
	prev_n := n
	has_prev := false
	bounds := this.Bounds
	for {
		if !n.Valid {
			if has_prev {
				//fmt.Println(x,y,z,bounds)
				return prev_n, bounds
			}
			fmt.Println("ERERERERERERORORROROROROR")
			return &this.Root, bounds
		}

		idx := bounds.getIndex(x,y,z)
		/*idx := 0
		if x > (bounds.X2-bounds.X1)/2.0 + bounds.X1 {
			idx = idx | 1
		}
		if y > (bounds.Y2-bounds.Y1)/2.0 + bounds.Y1 {
			idx = idx | 2
		}
		if z > (bounds.Z2-bounds.Z1)/2.0 + bounds.Z1 {
			idx = idx | 4
		}*/

		prev_n = n
		has_prev = true
		n = &n.Children[idx]
		//fmt.Println("Going to child",idx)
		bounds = bounds.subdivide(idx)
		/*if idx&1 != 0 {
			bounds.X1 = (bounds.X2-bounds.X1)/2.0 + bounds.X1
		} else {
			bounds.X2 = (bounds.X2-bounds.X1)/2.0 + bounds.X1
		}

		if idx&2 != 0 {
			bounds.Y1 = (bounds.Y2-bounds.Y1)/2.0 + bounds.Y1
		} else {
			bounds.Y2 = (bounds.Y2-bounds.Y1)/2.0 + bounds.Y1
		}

		if idx&4 != 0 {
			bounds.Z1 = (bounds.Z2-bounds.Z1)/2.0 + bounds.Z1
		} else {
			bounds.Z2 = (bounds.Z2-bounds.Z1)/2.0 + bounds.Z1
		}*/
	}
}

func (this *Octree) Add(v interface{}, x float64, y float64, z float64) {
	// If x,y,z are outside the bounds, expand the bounds
	for !this.Bounds.contains(x,y,z) {
		idx := 0
		if x < this.Bounds.X1 {
			idx = idx | 1
		}
		if y < this.Bounds.Y1 {
			idx = idx | 2
		}
		if z < this.Bounds.Z1 {
			idx = idx | 4
		}

		new_root := OctreeNode{true, nil, make([]OctreeNode, 8), []OctreeValue{}}
		new_root.Children[idx] = this.Root
		this.Root = new_root

		bounds := this.Bounds
		if idx&1 != 0 {
			bounds.X1 = 2.0 * (bounds.X1 - bounds.X2 / 2.0)
		} else {
			bounds.X2 = 2.0 * (bounds.X2 - bounds.X1 / 2.0)
		}

		if idx&2 != 0 {
			bounds.Y1 = 2.0 * (bounds.Y1 - bounds.Y2 / 2.0)
		} else {
			bounds.Y2 = 2.0 * (bounds.Y2 - bounds.Y1 / 2.0)
		}

		if idx&4 != 0 {
			bounds.Z1 = 2.0 * (bounds.Z1 - bounds.Z2 / 2.0)
		} else {
			bounds.Z2 = 2.0 * (bounds.Z2 - bounds.Z1 / 2.0)
		}
		this.Bounds = bounds
	}

	// Get the node...
	n, _ := this.getNode(x,y,z)
	n.Values = append(n.Values, OctreeValue{x,y,z,v})

	//fmt.Printf("Adding at %f,%f,%f to %p\n",x,y,z, n)

	// Should we split?
	SPLIT_LENGTH := 500
	if len(n.Values) > SPLIT_LENGTH {
		var i int
		//fmt.Println("Creating child nodes...")
		for i=0; i<8; i++ {
			n.Children[i].Valid = true
			n.Children[i].Parent = n
			n.Children[i].Children = make([]OctreeNode, 8)
			n.Children[i].Values = []OctreeValue{}
		}
		values := n.Values
		n.Values = []OctreeValue{}
		for i=0; i<len(values); i++ {
			this.Add(values[i].Element, values[i].X, values[i].Y, values[i].Z)
		}
	}
}

func (this *Octree) FindNearest(x, y, z float64) (SpatialValue, float64) {
	ok, elem, dist := this.Root.findNearest(1, this.Bounds, x,y,z)
	if !ok {
		fmt.Println("Not OK!")
	}
	//fmt.Println(elem,dist)
	return elem,dist
}

func (this *Octree) IterShell(x,y,z float64, r_inner float64, r_outer float64) <-chan SpatialValue {
	ch := make(chan SpatialValue)
	go func() {
		this.Root.IterShell(this.Bounds, x,y,z, r_inner, r_outer, ch)
		close(ch)
	}()
	return ch
}

type SpatialList struct {
	values []OctreeValue
}

func (this *SpatialList) Add(v interface{}, x,y,z float64) {
	this.values = append(this.values, OctreeValue{x,y,z, v})
}

func (this *SpatialList) FindNearest(x,y,z float64) (SpatialValue, float64) {
	var closest SpatialValue
	closest_dist := 1000000.0 // TODO: Support larger things
	for i := range this.values {
		dx := x - this.values[i].X
		dy := y - this.values[i].Y
		dz := z - this.values[i].Z
		dist := math.Sqrt(dx*dx+dy*dy+dz*dz)
		if dist < closest_dist {
			closest = this.values[i]
			closest_dist = dist
		}
	}
	return closest, closest_dist
}

func (this *SpatialList) IterShell(x,y,z float64, r_inner float64, r_outer float64) <-chan SpatialValue {
	ch := make(chan SpatialValue)
	go func() {
		for i := range this.values {
			r := distance(x,y,z, this.values[i].X, this.values[i].Y, this.values[i].Z)
			if r < r_outer && r > r_inner {
				ch <- this.values[i]
			}
		}
		close(ch)
	}()
	return ch
}

func NewList() SpatialDatabase {
	return &SpatialList{[]OctreeValue{}}
}
