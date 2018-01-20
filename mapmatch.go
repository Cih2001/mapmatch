package mapmatch

import (
	"math"
	"log"
)

type FastMapMatcher struct {
	osmMap           *OsmMap
	points           []PointsData
	firstUnmatchedIndex int
}

type PointsData struct {
	OriginalCoordinate Coordinate
	MatchedProjection  *Projection
	candidateWays []Way
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
	pointData := PointsData{
		OriginalCoordinate: Coordinate{
			Latitude:  lat,
			Longitude: lng,
		},
	}
	model.points = append(model.points, pointData)

	
	return model.MatchLastNPoints(-1)
}

//MatchLastNPoints matches last N points, when N is 0, It will start from model.firstUnmatchedIndex and when N is negative, it will match all points.
func (model *FastMapMatcher) MatchLastNPoints(N int) ([]Point, error) {
	if model.firstUnmatchedIndex == 0 {
		if err := model.matchFirstPoint(); err != nil {
			return nil, err
		}
	}

	var (
		result = []Point{}
		lastMapRefreshIndex = 0
		startIndex = 0
		MaximumDistanceToRefresh = 200.0
		m, _ = NewOsmMap(model.points[0].OriginalCoordinate)
	)
	
	//Computing starting index
	switch {
	case N<0:
		startIndex = 1
	case N==0:
		startIndex = model.firstUnmatchedIndex
	case N>0:
		startIndex = len(model.points)-N
		if startIndex <= 1 {
			startIndex = 1 
		}
	}
	
	//Computing condidates for each point.
	for i := startIndex; i < len(model.points); i++ {
		//refreshing roads database if nessessary.
		if model.points[i].OriginalCoordinate.linearDistance(model.points[lastMapRefreshIndex].OriginalCoordinate)*110575 > MaximumDistanceToRefresh {
			m, _ = NewOsmMap(model.points[i].OriginalCoordinate)
			lastMapRefreshIndex = i
		}
		model.points[i].candidateWays = m.computeCondidateWays(model.points[i].OriginalCoordinate,model.points[i-1].OriginalCoordinate)
	}

	//Finding best condidate for each point.
	result = append(result, Point{
		Index: 0,
		Coordinate: model.points[0].MatchedProjection.Coordinate,
	})
	for i := startIndex; i < len(model.points); i++ {

		//Finding closest way among candidates
		maximumDF := -math.MaxFloat64
		var matchedWayIndex = -1
		print("\033[H\033[2J")
		for j, w := range model.points[i].candidateWays {
			df := computeDistanceFactor(model.points[i].OriginalCoordinate,w)
			if i == len(model.points)-1 {
				log.Println(w.ID,w.Tags["name"],df)
			}
			if df > maximumDF {
				matchedWayIndex = j
				maximumDF=df
			}
		}

		var matchedWay *Way
		if matchedWayIndex >= 0 {
			matchedWay = &model.points[i].candidateWays[matchedWayIndex]
		} else {
			matchedWay = model.points[i-1].MatchedProjection.Arc
		}
		model.points[i].MatchedProjection = matchedWay.FindProjection(model.points[i].OriginalCoordinate)
		log.Println("Direction of last two points: ", model.points[i-1].OriginalCoordinate.direction(model.points[i].OriginalCoordinate))
		log.Println("Direction of last two projec: ", model.points[i-1].MatchedProjection.direction(model.points[i].MatchedProjection.Coordinate))
		
		result = append(result, Point{
			Index: i,
			Coordinate: model.points[i].MatchedProjection.Coordinate,
		})
	}

	return result,nil
}


//Is used to match the starting point
func (model *FastMapMatcher) matchFirstPoint() (err error) {
	var arcs []Way
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
