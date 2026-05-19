package main

import (
	"container/heap"
	"fmt"
	"math"
)

// d = sqrt((x2 - x1)^2 + (y2 - y1)^2)
func EuclideanDistance(x1, y1, x2, y2 float64) float64 {
	dx := x2 - x1
	dy := y2 - y1

	return math.Sqrt(dx*dx + dy*dy)
}

// d = |x2 - x1| + |y2 - y1|
func ManhattanDistance(x1, y1, x2, y2 float64) float64 {
	return math.Abs(x2-x1) + math.Abs(y2-y1)
}

// Haversine calculates the distance between two coordinates in kilometers.
func Haversine(lat1, lon1, lat2, lon2 float64) float64 {
	const R = 6371.0 // Earth radius in kilometers

	phi1 := lat1 * math.Pi / 180
	phi2 := lat2 * math.Pi / 180
	deltaPhi := (lat2 - lat1) * math.Pi / 180
	deltaLambda := (lon2 - lon1) * math.Pi / 180

	a := math.Sin(deltaPhi/2)*math.Sin(deltaPhi/2) +
		math.Cos(phi1)*math.Cos(phi2)*
			math.Sin(deltaLambda/2)*math.Sin(deltaLambda/2)

	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))

	return R * c
}

// A* pathfinding over the simple `roadNetwork` using segment endpoints as nodes.
// Returns a slice of (lat, lon) coordinates representing the path and the total distance in km.
type edge struct {
	to int
	w  float64
}

type pqItem struct {
	node  int
	f, g  float64
	index int
}

type priorityQueue []*pqItem

func (pq priorityQueue) Len() int           { return len(pq) }
func (pq priorityQueue) Less(i, j int) bool { return pq[i].f < pq[j].f }
func (pq priorityQueue) Swap(i, j int)      { pq[i], pq[j] = pq[j], pq[i]; pq[i].index = i; pq[j].index = j }
func (pq *priorityQueue) Push(x interface{}) {
	item := x.(*pqItem)
	item.index = len(*pq)
	*pq = append(*pq, item)
}
func (pq *priorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	item.index = -1
	*pq = old[0 : n-1]
	return item
}

// BuildGraph converts the global `roadNetwork` into nodes (unique endpoints) and adjacency list.
func BuildGraph() ([][2]float64, map[int][]edge) {
	nodes := make([][2]float64, 0)
	idx := make(map[string]int)
	addNode := func(lat, lon float64) int {
		key := fmt.Sprintf("%f,%f", lat, lon)
		if id, ok := idx[key]; ok {
			return id
		}
		id := len(nodes)
		nodes = append(nodes, [2]float64{lat, lon})
		idx[key] = id
		return id
	}

	adj := make(map[int][]edge)

	for _, seg := range roadNetwork {
		a := addNode(seg.Lat1, seg.Lon1)
		b := addNode(seg.Lat2, seg.Lon2)
		w := Haversine(nodes[a][0], nodes[a][1], nodes[b][0], nodes[b][1])
		adj[a] = append(adj[a], edge{to: b, w: w})
		adj[b] = append(adj[b], edge{to: a, w: w})
	}

	return nodes, adj
}

// findClosest returns index of the closest node to the given coordinate.
func findClosest(nodes [][2]float64, lat, lon float64) int {
	best := -1
	bestD := math.MaxFloat64
	for i, p := range nodes {
		d := Haversine(lat, lon, p[0], p[1])
		if d < bestD {
			bestD = d
			best = i
		}
	}
	return best
}

// AStar finds a path between start and goal coordinates over `roadNetwork` endpoints.
// If no path is found, returns empty slice and +Inf distance.
func AStar(startLat, startLon, goalLat, goalLon float64) ([][2]float64, float64) {
	nodes, adj := BuildGraph()
	if len(nodes) == 0 {
		return nil, math.Inf(1)
	}

	start := findClosest(nodes, startLat, startLon)
	goal := findClosest(nodes, goalLat, goalLon)

	// A* structures
	open := &priorityQueue{}
	heap.Init(open)
	startItem := &pqItem{node: start, g: 0, f: Haversine(nodes[start][0], nodes[start][1], nodes[goal][0], nodes[goal][1])}
	heap.Push(open, startItem)

	cameFrom := make(map[int]int)
	gScore := make(map[int]float64)
	for i := range nodes {
		gScore[i] = math.Inf(1)
	}
	gScore[start] = 0

	inOpen := make(map[int]bool)
	inOpen[start] = true

	for open.Len() > 0 {
		curItem := heap.Pop(open).(*pqItem)
		cur := curItem.node
		if cur == goal {
			// reconstruct
			path := make([][2]float64, 0)
			for at := goal; ; {
				path = append([][2]float64{nodes[at]}, path...)
				if at == start {
					break
				}
				at = cameFrom[at]
			}
			// prepend actual start and append actual goal if they differ from node positions
			if !(nodes[start][0] == startLat && nodes[start][1] == startLon) {
				path = append([][2]float64{{startLat, startLon}}, path...)
			}
			if !(nodes[goal][0] == goalLat && nodes[goal][1] == goalLon) {
				path = append(path, [2]float64{goalLat, goalLon})
			}
			// compute total distance
			total := 0.0
			for i := 0; i+1 < len(path); i++ {
				total += Haversine(path[i][0], path[i][1], path[i+1][0], path[i+1][1])
			}
			return path, total
		}

		inOpen[cur] = false

		for _, e := range adj[cur] {
			tentativeG := gScore[cur] + e.w
			if tentativeG < gScore[e.to] {
				cameFrom[e.to] = cur
				gScore[e.to] = tentativeG
				f := tentativeG + Haversine(nodes[e.to][0], nodes[e.to][1], nodes[goal][0], nodes[goal][1])
				if !inOpen[e.to] {
					heap.Push(open, &pqItem{node: e.to, g: tentativeG, f: f})
					inOpen[e.to] = true
				} else {
					// If already in open, push a new item with better f (lazy update)
					heap.Push(open, &pqItem{node: e.to, g: tentativeG, f: f})
				}
			}
		}
	}

	return nil, math.Inf(1)
}
