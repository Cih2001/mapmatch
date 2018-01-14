package mapmatch

import (
	"time"
)

type Way struct {
	ID  int64 `bson:"_id" xml:"id,attr"`
	Loc struct {
		Type        string      `bson:"type"`
		Coordinates [][]float64 `bson:"coordinates"`
	} `bson:"loc"`
	Version   int               `bson:"ver"       xml:"version,attr"`
	Ts        time.Time         `bson:"ts"        xml:"timestamp,attr"`
	UID       int64             `bson:"uid"       xml:"uid,attr"`
	User      string            `bson:"user"      xml:"user,attr"`
	ChangeSet int64             `bson:"changeset" xml:"changeset,attr"`
	Tags      map[string]string `bson:"tags"`
	Nodes     []int64           `bson:"nodes"`
}
