package mapmatch

import (
	"math"
)

type OsmMap struct {
	Ways []*Way
}


func computeDistanceFactor(cor Coordinate, arc *Way) float64 {
	//return DistanceC - DistanceA*math.Pow(projection.Distance*110575, DistanceN)
	return normalProbability(cor,arc)*100
}

// A probability to measure how likely a GPS observation is matched to a candidate arc.
// It is based on normal distribution
func normalProbability(cor Coordinate, arc *Way) float64 {
	projection := arc.FindProjection(cor)
	distanceMeter := projection.Distance * 110575
	NormalMean := 0.0
	NormalDeviation := 20.0 //Amount of expected GPS error
	//Normal distribution formula
	probability := 1 / (math.Sqrt(2*math.Pi) * NormalDeviation) * math.Exp(-(math.Pow(distanceMeter-NormalMean, 2) / (2 * math.Pow(NormalDeviation, 2))))
	return probability
}

