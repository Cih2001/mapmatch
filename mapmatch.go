package mapmatch

import (
	"gopkg.in/mgo.v2"
	"gopkg.in/mgo.v2/bson"
)

var session *mgo.Session

var (
	DBName             = "osm"
	WaysCollectionName = "ways"
	MongoHost          = "127.0.0.1:27017"

	//Minimum and maximum distance for loading maps base on a point
	//Used in mongo db.
	MinLoadingDistance int
	MaxLoadingDistance = 200
)

type FastMapMatcher struct {
	osmMap *OsmMap
	points []PointsData
}

type PointsData struct {
	OriginalLatitude  float64
	OriginalLongitude float64
	MatchedLatitude   float64
	MatchedLongitude  float64
}

type ResPoint struct {
	Index     int
	Latitude  float64
	Longitude float64
}

func (model *FastMapMatcher) MatchPoint(lat, lng float64) ([]ResPoint, error) {
	result := []ResPoint{}
	if model.osmMap == nil {
		om, err := NewOsmMap(lat, lng)
		if err != nil {
			return nil, err
		}
		model.osmMap = om
	}

	pointData := PointsData{
		OriginalLatitude:  lat,
		OriginalLongitude: lng,
	}

	model.points = append(model.points, pointData)

	result = append(result, ResPoint{
		Index:     len(model.points) - 1,
		Latitude:  model.osmMap.Ways[0].Loc.Coordinates[len(model.points)-1][1],
		Longitude: model.osmMap.Ways[0].Loc.Coordinates[len(model.points)-1][0],
	})

	return result, nil
}

func initializeDB() error {
	if session == nil {
		s, err := mgo.Dial(MongoHost)
		if err != nil {
			return err
		}
		session = s
	}
	return nil
}

func NewOsmMap(lat, lng float64) (*OsmMap, error) {
	err := initializeDB()
	if err != nil {
		return nil, err
	}

	result := new(OsmMap)

	err = session.DB(DBName).C(WaysCollectionName).Find(bson.M{
		"loc": bson.M{"$near": bson.M{
			"$geometry": bson.M{
				"type":        "Point",
				"coordinates": []float64{lng, lat},
			},
			"$minDistance": MinLoadingDistance,
			"$maxDistance": MaxLoadingDistance,
		}}}).All(&result.Ways)

	return result, err
}
