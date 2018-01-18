package mapmatch

import (
	"log"
)

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
		result = append(result,  Point{
			p.FirstWayPoint.Coordinate,
			1,
		})
		result = append(result,  Point{
			p.SecondWayPoint.Coordinate,
			1,
		})
	}

	return result
}