package mapmatch

import (
	"math"
)

var (
	//To calibrate distance factor
	DistanceC float64 = 10
	DistanceA float64 = 0.1
	DistanceN float64 = 1.2
)
type OsmMap struct{
	Ways []Way
}


func (model *OsmMap) computeCondidateWays(toMatch,lastMatched Coordinate) []Way {
	var result []Way

	for _,w := range model.Ways {
		var distanceFactor float64
		distanceFactor = computeDistanceFactor(toMatch, w)


		//orientationFactor = weightOrientation(cor, model.PointCollection[i].EstimatedPoint, arc)
		//shortestPathWeight = model.weightShortestPath(model.PointCollection[i-1].Projection, model.PointCollection[i].EstimatedPoint, arc)

		//if distanceWeight+orientationWeight > 15 || distanceWeight > 8.5 {
		if distanceFactor > 1.5 {
			result = append(result,w)
		}
	}
	return result
}

func computeDistanceFactor(cor Coordinate, arc Way) float64 {
	projection := arc.FindProjection(cor)
	return DistanceC - DistanceA*math.Pow(projection.Distance*110575, DistanceN)
}

// func computeOrientationFactor(lastPoint, currentPoint Trajectory, arc OsmWay) float64 {
// 	projectionLastPoint := arc.ProjectPoint(lastPoint.Lat, lastPoint.Lng)
// 	projectionCurrentPoint := arc.ProjectPoint(currentPoint.Lat, currentPoint.Lng)
// 	projectedLastPoint := Trajectory{Lat: projectionLastPoint.Lat, Lng: projectionLastPoint.Lng}
// 	projectedCurrentPoint := Trajectory{Lat: projectionCurrentPoint.Lat, Lng: projectionCurrentPoint.Lng}
// 	return OrientationC * math.Pow(math.Cos(findAngle(lastPoint, currentPoint, projectedLastPoint, projectedCurrentPoint)), OrientationN)
// }
