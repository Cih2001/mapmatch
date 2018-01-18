package mapmatch

import (
	"math"
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
		var matchedWay *Way
		for _, w := range model.points[i].candidateWays {
			if df := computeDistanceFactor(model.points[i].OriginalCoordinate,w); df > maximumDF {
				matchedWay = &w
				maximumDF=df
			}
		}

		if matchedWay != nil {
			model.points[i].MatchedProjection = matchedWay.FindProjection(model.points[i].OriginalCoordinate)
		} else {
			model.points[i].MatchedProjection = model.points[i-1].MatchedProjection
		}
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
		}
	}
	model.firstUnmatchedIndex++
	return
}
