package mapmatch

import (
	"log"
	"math"
	"strconv"
	"sync"

	"github.com/RyanCarrier/dijkstra"
)

var (
	distanceMeterFactor = 110575.0
)

type FastMapMatcher struct {
	osmMap              *OsmMap
	points              []PointsData
	firstUnmatchedIndex int
	generalMutex        *sync.Mutex
}

type PointsData struct {
	OriginalCoordinate Coordinate
	MatchedProjection  *Projection
	candidateWays      []*Way
}

type Coordinate struct {
	Latitude  float64
	Longitude float64
}

type Point struct {
	Coordinate
	Index int
}

func (model *FastMapMatcher) MatchPoint(lat, lng float64) ([]Point, error) {
	if model.generalMutex == nil {
		model.generalMutex = &sync.Mutex{}
	}
	model.generalMutex.Lock()
	defer model.generalMutex.Unlock()

	pointData := PointsData{
		OriginalCoordinate: Coordinate{
			Latitude:  lat,
			Longitude: lng,
		},
	}
	model.points = append(model.points, pointData)

	return model.MatchLastNPoints(3)
}

func (model *FastMapMatcher) computeCondidateWays(candidateIndex int) {
	var result []*Way

	for _, w := range model.osmMap.Ways {
		var distanceFactor float64
		currentCoordinate := model.points[candidateIndex].OriginalCoordinate
		distanceFactor = computeDistanceFactor(currentCoordinate, w)
		if distanceFactor > 0.1 {
			result = append(result, w)
		}
	}

	model.points[candidateIndex].candidateWays = result
}

//MatchLastNPoints matches last N points, when N is 0, It will start from model.firstUnmatchedIndex and when N is negative, it will match all points.
func (model *FastMapMatcher) MatchLastNPoints(N int) ([]Point, error) {
	if model.firstUnmatchedIndex == 0 {
		if err := model.matchFirstPoint(); err != nil {
			return nil, err
		}
	}

	var (
		result                   = []Point{}
		lastMapRefreshIndex      = 0
		startIndex               = 0
		MaximumDistanceToRefresh = 200.0
		m, _                     = NewOsmMap(model.points[0].OriginalCoordinate)
	)

	model.osmMap = m
	//Computing starting index
	switch {
	case N < 0:
		startIndex = 1
	case N == 0:
		startIndex = model.firstUnmatchedIndex
	case N > 0:
		startIndex = len(model.points) - N
		if startIndex <= 1 {
			startIndex = 1
		}
	}
	log.Println("StartIndex:", startIndex)
	//Computing condidates for each point.
	for i := startIndex; i < len(model.points); i++ {
		//refreshing roads database if nessessary.
		if model.points[i].OriginalCoordinate.linearDistance(model.points[lastMapRefreshIndex].OriginalCoordinate)*distanceMeterFactor > MaximumDistanceToRefresh {
			m, _ = NewOsmMap(model.points[i].OriginalCoordinate)
			model.osmMap = m
			lastMapRefreshIndex = i
		}
		//model.points[i].candidateWays = model.computeCondidateWays(model.points[i].OriginalCoordinate,model.points[i-1].OriginalCoordinate)
		model.computeCondidateWays(i)
	}

	//Finding best condidate for each point.
	result = append(result, Point{
		Index:      0,
		Coordinate: model.points[0].MatchedProjection.Coordinate,
	})
	for i := startIndex; i < len(model.points); {
		//Finding all combinations.
		CombinationLimit := 5
		if len(model.points)-i < CombinationLimit {
			CombinationLimit = len(model.points) - i
		}

		var wg sync.WaitGroup
		pathWeights := make(map[int]float64)
		combinations := model.findCombinations(i, CombinationLimit)
		var mutex = &sync.Mutex{}
		for j, comb := range combinations {
			wg.Add(1)
			go func(k int, c []int) {
				w := model.findPathWeight(i, c, &wg)
				mutex.Lock()
				pathWeights[k] = w
				mutex.Unlock()
			}(j, comb)
		}
		wg.Wait()
		mutex.Lock()
		defer mutex.Unlock()
		log.Println("Done waiting")
		maxPathWeight := -math.MaxFloat64
		bestPathIndex := 0
		for k, v := range pathWeights {
			if v > maxPathWeight {
				maxPathWeight = v
				bestPathIndex = k
			}
		}
		for j, c := range combinations[bestPathIndex] {
			matchedWay := model.points[i+j].candidateWays[c]
			model.points[i+j].MatchedProjection = matchedWay.FindProjection(model.points[i+j].OriginalCoordinate)
			result = append(result, Point{
				Index:      i + j,
				Coordinate: model.points[i+j].MatchedProjection.Coordinate,
			})
		}
		i += CombinationLimit
	}

	return result, nil
}
func (model *FastMapMatcher) findPathWeight(startIndex int, combination []int, wg *sync.WaitGroup) float64 {
	defer wg.Done()
	result := 0.0
	for i, c := range combination {
		currentPoint := model.points[startIndex+i]
		currentWay := currentPoint.candidateWays[c]

		prevPoint := model.points[startIndex+i-1]
		var prevWay *Way
		if i == 0 {
			if prevPoint.MatchedProjection == nil {
				//Probably an error, we must restart. the path
				model.hardFixFromIndex(startIndex+i-1)			
			}
			prevWay = prevPoint.MatchedProjection.Arc
		} else {
			prevWay = prevPoint.candidateWays[combination[i-1]]
		}

		result += currentWay.normalProbability(currentPoint.OriginalCoordinate)
		result += model.shortestPathProbability(startIndex+i, currentWay, prevWay)
	}
	return result
}
func (model *FastMapMatcher) hardFixFromIndex(i int) {
	if model.points[i-1].MatchedProjection == nil {
		model.hardFixFromIndex(i-1)
	}
	model.points[i].MatchedProjection = model.points[i-1].MatchedProjection.Arc.FindProjection(model.points[i].OriginalCoordinate)
	model.points[i].candidateWays = append(model.points[i].candidateWays, model.points[i-1].MatchedProjection.Arc)
}
func (model *FastMapMatcher) shortestPathProbability(index int, currentArc, prevArc *Way) float64 {
	srcProjection := prevArc.FindProjection(model.points[index-1].OriginalCoordinate)
	desProjection := currentArc.FindProjection(model.points[index].OriginalCoordinate)
	distance := srcProjection.Coordinate.linearDistance(desProjection.Coordinate) * distanceMeterFactor
	//m, _ := NewOsmMap(model.PointCollection[index].EstimatedPoint.Lat, model.PointCollection[index].EstimatedPoint.Lng)
	shortestPath, err := dijkstraShortestPath(model.osmMap.Ways, srcProjection, desProjection)
	if err != nil {
		return 0
	}

	adjustResult := func(r float64) float64 {
		if r <= 0 {
			return 0
		}
		if r >= 1 {

			return 1
		}
		return r
	}

	result := adjustResult(distance/float64(shortestPath)) - 0.10
	return result
}

func (model *FastMapMatcher) findCombinations(startIndex, Limit int) (result [][]int) {
	//We should check for the return base
	if Limit == 1 {
		for idx := range model.points[startIndex].candidateWays {
			result = append(result, []int{idx})
		}
		return result
	}
	//for general case
	nextCombinations := model.findCombinations(startIndex+1, Limit-1)
	for i := range nextCombinations {
		for idx := range model.points[startIndex].candidateWays {
			sl := append([]int{idx}, nextCombinations[i]...)
			result = append(result, sl)
		}
	}
	return result
}

//Is used to match the starting point
func (model *FastMapMatcher) matchFirstPoint() (err error) {
	var arcs []*Way
	m, err := NewOsmMap(model.points[0].OriginalCoordinate)
	if err != nil {
		return
	}
	arcs = m.Ways
	//Find closest projection and way
	distance := math.MaxFloat64
	for _, w := range arcs {
		p := w.FindProjection(model.points[0].OriginalCoordinate)
		if p.Distance < distance {
			model.points[0].MatchedProjection = p
			distance = p.Distance
		}
	}
	model.firstUnmatchedIndex++
	return
}

//It will return the shortest path distance between src and des in meters.
func dijkstraShortestPath(arcs []*Way, srcProjection, desProjection *Projection) (float64, error) {
	graph := dijkstra.NewGraph()
	DirectedGraph := true

	//When both projections are on the same way, shortest path is the distance between them
	if srcProjection.FirstWayPoint.ID == desProjection.FirstWayPoint.ID && srcProjection.SecondWayPoint.ID == desProjection.SecondWayPoint.ID {
		return srcProjection.linearDistance(desProjection.Coordinate) * distanceMeterFactor, nil
	}

	//Building graph of roads
	for _, way := range arcs {
		for i := 0; i < len(way.Loc.Coordinates)-1; i++ {
			firstWayPoint := way.GetWayPointByIndex(i)
			secondWayPoint := way.GetWayPointByIndex(i + 1)

			graph.AddMappedVertex(strconv.Itoa(int(firstWayPoint.ID)))
			graph.AddMappedVertex(strconv.Itoa(int(secondWayPoint.ID)))

			distance := firstWayPoint.linearDistance(secondWayPoint) * distanceMeterFactor
			graph.AddMappedArc(strconv.Itoa(int(firstWayPoint.ID)), strconv.Itoa(int(secondWayPoint.ID)), int64(distance))
			if way.Tags["oneway"] != "yes" || !DirectedGraph {
				graph.AddMappedArc(strconv.Itoa(int(secondWayPoint.ID)), strconv.Itoa(int(firstWayPoint.ID)), int64(distance))
			}
		}
	}

	//Adding source and destinations
	srcVertex := graph.AddMappedVertex("src")
	desVertex := graph.AddMappedVertex("des")

	d := int64(srcProjection.SecondWayPoint.Coordinate.linearDistance(srcProjection.Coordinate) * distanceMeterFactor)
	graph.AddMappedArc("src", strconv.Itoa(int(srcProjection.SecondWayPoint.ID)), d)

	d = int64(srcProjection.FirstWayPoint.Coordinate.linearDistance(srcProjection.Coordinate) * distanceMeterFactor)
	graph.AddMappedArc("src", strconv.Itoa(int(srcProjection.FirstWayPoint.ID)), d)

	d = int64(desProjection.FirstWayPoint.Coordinate.linearDistance(desProjection.Coordinate) * distanceMeterFactor)
	graph.AddMappedArc(strconv.Itoa(int(desProjection.FirstWayPoint.ID)), "des", d)

	d = int64(desProjection.SecondWayPoint.Coordinate.linearDistance(desProjection.Coordinate) * distanceMeterFactor)
	graph.AddMappedArc(strconv.Itoa(int(desProjection.SecondWayPoint.ID)), "des", d)

	//Finding the result
	best, err := graph.Shortest(srcVertex, desVertex)
	if err != nil {
		return 0, err
	}
	return float64(best.Distance), err
}
