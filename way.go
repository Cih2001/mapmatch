package mapmatch

import (
	"math"
)

type Way struct {
	ID  int64 `bson:"_id" xml:"id,attr"`
	Loc struct {
		Type        string      `bson:"type"`
		Coordinates [][]float64 `bson:"coordinates"`
	} `bson:"loc"`
	Tags        map[string]string `bson:"tags"`
	WayPointIDS []int64           `bson:"nodes"`
}

type Projection struct {
	Coordinate
	Arc            *Way
	Distance       float64
	FirstWayPoint  WayPoint
	SecondWayPoint WayPoint
	Direction      float64
	OneWay         bool
}

type WayPoint struct {
	Coordinate
	ID int64
}

func (model *Way) IsOneWay() bool {
	if model.Tags["oneway"] == "yes" {
		return true
	}
	return false
}

func (model *Way) GetWayPointByIndex(i int) WayPoint {
	return WayPoint{
		ID: model.WayPointIDS[i],
		Coordinate: Coordinate{
			Latitude:  model.Loc.Coordinates[i][1],
			Longitude: model.Loc.Coordinates[i][0],
		},
	}
}

func (model *Way) FindProjection(cor Coordinate) *Projection {
	result := Projection{
		Arc: model,
	}
	waypointsCount := len(model.Loc.Coordinates)
	for i := 0; i < waypointsCount-1; i++ {

		firstWayPoint := model.GetWayPointByIndex(i)
		secondWayPoint := model.GetWayPointByIndex(i + 1)

		distance, projectedCoordinate := perpendicularDistance(cor, firstWayPoint, secondWayPoint)
		if i == 0 || distance < result.Distance {
			result.Latitude = projectedCoordinate.Latitude
			result.Longitude = projectedCoordinate.Longitude
			result.Distance = distance
			result.FirstWayPoint = firstWayPoint
			result.SecondWayPoint = secondWayPoint
			
			//result.Direction = computeDirection(firstNode, secondNode)
		}

	}
	return &result
}

func linearDistance(lat1, lng1, lat2, lng2 float64) float64 {
	latDiff := (lat2 - lat1) * (lat2 - lat1)
	lngDiff := (lng2 - lng1) * (lng2 - lng1)
	return math.Sqrt(latDiff + lngDiff)
}

func (model Coordinate) linearDistance(des Coordinate) float64 {
	return linearDistance(model.Latitude, model.Longitude, des.Latitude, des.Longitude)
}

func (model WayPoint) linearDistance(des WayPoint) float64 {
	return model.Coordinate.linearDistance(des.Coordinate)
}

func perpendicularDistance(cor Coordinate, waypoint1, waypoint2 WayPoint) (distance float64, result Coordinate) {
	if waypoint1.ID == waypoint2.ID {
		return cor.linearDistance(waypoint1.Coordinate), waypoint1.Coordinate
	}
	m := (waypoint2.Latitude - waypoint1.Latitude) / (waypoint2.Longitude - waypoint1.Longitude)
	//result.Longitude = (m*(cor.Longitude-waypoint1.Latitude+m*waypoint1.Longitude) + cor.Longitude) * m / (m*m + 1)
	//result.Latitude = waypoint1.Latitude + m*(result.Longitude-waypoint1.Longitude)
	k := waypoint2.Latitude - m*waypoint2.Longitude
	
	
	result.Longitude = (cor.Longitude + m*cor.Latitude - m*k) / (m*m + 1)
	result.Latitude = m*result.Longitude + k
	distance = cor.linearDistance(result)
	
	if isInWayPointsLimits(result, waypoint1, waypoint2) {
		return
	}
	//result is not on the way, so we should return one of the waypoints.
	distance = waypoint1.Coordinate.linearDistance(cor)
	result.Longitude = waypoint1.Longitude
	result.Latitude = waypoint1.Latitude
	if waypoint2.Coordinate.linearDistance(cor) < distance {
		distance = waypoint2.Coordinate.linearDistance(cor)
		result.Longitude = waypoint2.Longitude
		result.Latitude = waypoint2.Latitude
	}
	return
}

func isInWayPointsLimits(cor Coordinate, w1, w2 WayPoint) bool {
	minLat := w1.Latitude
	maxLat := w2.Latitude
	minLng := w1.Longitude
	maxLng := w2.Longitude
	if maxLat < minLat {
		minLat, maxLat = maxLat, minLat
	}
	if maxLng < minLng {
		minLng, maxLng = maxLng, minLng
	}
	if minLng < cor.Longitude && cor.Longitude < maxLng && minLat < cor.Latitude && cor.Latitude < maxLat {
		return true
	}
	return false
}
