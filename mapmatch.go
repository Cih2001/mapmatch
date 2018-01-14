package mapmatch

import (
	"math"
	"log"
)

type FastMapMatcher struct {
	osmMap           *OsmMap
	points           []PointsData
	lastMatchedIndex int
}

type PointsData struct {
	OriginalCoordinate Coordinate
	MatchedProjection  *Projection
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
	result := []Point{}

	pointData := PointsData{
		OriginalCoordinate: Coordinate{
			Latitude:  lat,
			Longitude: lng,
		},
	}
	model.points = append(model.points, pointData)

	if model.lastMatchedIndex == 0 {
		if err := model.matchFirstPoint(); err != nil {
			return nil, err
		}
	}

	result = append(result, Point{
		Index: len(model.points) - 1,
		Coordinate: Coordinate{
			Latitude:  model.points[0].MatchedProjection.Latitude,
			Longitude: model.points[0].MatchedProjection.Longitude,
		},
	})

	return result, nil
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
		log.Println(p.Distance, p.Arc.ID)
		if p.Distance < distance {
			model.points[0].MatchedProjection = &p
		}
	}

	return
}
