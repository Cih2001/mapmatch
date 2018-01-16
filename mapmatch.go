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

	return result, nil
}

//For testing purpose
func (model *FastMapMatcher) ReturnRoadsPoints(lat, lng float64) []Point {

	result := []Point{}

	m, err := NewOsmMap(Coordinate{lat, lng})
	if err != nil {
		return nil
	}

	i := 0
	for _, w := range m.Ways {
		log.Println("Road ID:", w.ID)
		for _, p := range w.Loc.Coordinates {
			result = append(result, Point{
				Coordinate{p[1], p[0]},
				i,
			})
			i++
		}
	}

	return result
}

//For testing purpose
func (model *FastMapMatcher) ReturnAllProjections(lat, lng float64) []Point {
	result := []Point{}

	m, err := NewOsmMap(Coordinate{lat, lng})
	if err != nil {
		return nil
	}

	for _, w := range m.Ways {
		p := w.FindProjection(Coordinate{lat, lng})
		result = append(result,  Point{
			p.Coordinate,
			0,
		})
	}

	return result
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
			model.points[0].MatchedProjection = &p
		}
	}

	return
}
