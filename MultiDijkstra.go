package main

import (
	"container/heap"
	"math"
)

type mdEdge struct {
	To     string
	Weight float64
}

type mdItem struct {
	Node  string
	Dist  float64
	index int
}

type mdPriorityQueue []*mdItem

func (pq mdPriorityQueue) Len() int { return len(pq) }

func (pq mdPriorityQueue) Less(i, j int) bool {
	return pq[i].Dist < pq[j].Dist
}

func (pq mdPriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].index = i
	pq[j].index = j
}

func (pq *mdPriorityQueue) Push(x interface{}) {
	item := x.(*mdItem)
	item.index = len(*pq)
	*pq = append(*pq, item)
}

func (pq *mdPriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	item.index = -1
	*pq = old[:n-1]
	return item
}

type DestinationResult struct {
	Destination string
	Distance    float64
	Path        []string
	Reachable   bool
}

// MultiDestinationDijkstra runs Dijkstra once from source and returns best paths
// only for the requested destinations.
func MultiDestinationDijkstra(
	graph map[string][]mdEdge,
	source string,
	destinations []string,
) map[string]DestinationResult {
	results := make(map[string]DestinationResult, len(destinations))
	if _, ok := graph[source]; !ok {
		for _, d := range destinations {
			results[d] = DestinationResult{Destination: d, Distance: math.Inf(1), Reachable: false}
		}
		return results
	}

	dist := make(map[string]float64, len(graph))
	prev := make(map[string]string, len(graph))
	visited := make(map[string]bool, len(graph))

	for node := range graph {
		dist[node] = math.Inf(1)
	}
	dist[source] = 0

	destSet := make(map[string]bool, len(destinations))
	remaining := 0
	for _, d := range destinations {
		destSet[d] = true
		if _, ok := graph[d]; ok {
			remaining++
		}
	}

	pq := &mdPriorityQueue{}
	heap.Init(pq)
	heap.Push(pq, &mdItem{Node: source, Dist: 0})

	for pq.Len() > 0 && remaining > 0 {
		cur := heap.Pop(pq).(*mdItem)
		u := cur.Node

		if visited[u] {
			continue
		}
		visited[u] = true

		if destSet[u] {
			remaining--
		}

		for _, e := range graph[u] {
			if visited[e.To] {
				continue
			}
			alt := dist[u] + e.Weight
			if alt < dist[e.To] {
				dist[e.To] = alt
				prev[e.To] = u
				heap.Push(pq, &mdItem{Node: e.To, Dist: alt})
			}
		}
	}

	for _, d := range destinations {
		d, exists := d, false
		if _, ok := graph[d]; ok {
			exists = true
		}

		if !exists || math.IsInf(dist[d], 1) {
			results[d] = DestinationResult{Destination: d, Distance: math.Inf(1), Reachable: false}
			continue
		}

		path := reconstructPath(prev, source, d)
		reachable := len(path) > 0 && path[0] == source && path[len(path)-1] == d
		results[d] = DestinationResult{
			Destination: d,
			Distance:    dist[d],
			Path:        path,
			Reachable:   reachable,
		}
	}

	return results
}

func reconstructPath(prev map[string]string, source, target string) []string {
	if source == target {
		return []string{source}
	}

	path := []string{target}
	for cur := target; cur != source; {
		p, ok := prev[cur]
		if !ok {
			return nil
		}
		path = append(path, p)
		cur = p
	}

	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}
	return path
}

