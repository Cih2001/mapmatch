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

func NewOsmMap(cor Coordinate) (*OsmMap, error) {
	err := initializeDB()
	if err != nil {
		return nil, err
	}

	result := new(OsmMap)

	err = session.DB(DBName).C(WaysCollectionName).Find(bson.M{
		"loc": bson.M{"$near": bson.M{
			"$geometry": bson.M{
				"type":        "Point",
				"coordinates": []float64{cor.Longitude, cor.Latitude},
			},
			"$minDistance": MinLoadingDistance,
			"$maxDistance": MaxLoadingDistance,
		}}}).All(&result.Ways)

	return result, err
}