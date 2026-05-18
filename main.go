package main

import (
	"fmt"
	"math"
	"strconv"
)

type GPSData struct {
	Latitude  float64
	Longitude float64
	Speed     float64
	Course    string // degrees as string (attempt to parse)
}

type Position struct {
	Latitude  float64
	Longitude float64
	RoadSegID int
	RoadName  string
	Direction string // direction of the street relative to travel
}

// Haversine calculates the distance between two coordinates in kilometers.
func Haversine(lat1, lon1, lat2, lon2 float64) float64 {
	const R = 6371.0 // Earth radius in kilometers

	phi1 := lat1 * math.Pi / 180
	phi2 := lat2 * math.Pi / 180
	deltaPhi := (lat2 - lat1) * math.Pi / 180
	deltaLambda := (lon2 - lon1) * math.Pi / 180

	a := math.Sin(deltaPhi/2)*math.Sin(deltaPhi/2) +
		math.Cos(phi1)*math.Cos(phi2)*
			math.Sin(deltaLambda/2)*math.Sin(deltaLambda/2)

	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))

	return R * c
}

// RoadSeg represents a simple road segment between two points.
type RoadSeg struct {
	ID       int
	Name     string
	Lat1     float64
	Lon1     float64
	Lat2     float64
	Lon2     float64
}

// sample in-memory road network (small demo)
var roadNetwork = []RoadSeg{
	{ID: 1, Name: "Main St", Lat1: 37.7749, Lon1: -122.4194, Lat2: 37.7790, Lon2: -122.4194},
	{ID: 2, Name: "1st Ave", Lat1: 37.7760, Lon1: -122.4220, Lat2: 37.7760, Lon2: -122.4150},
	{ID: 3, Name: "Market St", Lat1: 37.7740, Lon1: -122.4210, Lat2: 37.7780, Lon2: -122.4170},
}

// deg2rad helper
func deg2rad(d float64) float64 { return d * math.Pi / 180 }

// rad2deg helper
func rad2deg(r float64) float64 { return r * 180 / math.Pi }

// bearing returns bearing in degrees from point 1 to point 2
func bearing(lat1, lon1, lat2, lon2 float64) float64 {
	phi1 := deg2rad(lat1)
	phi2 := deg2rad(lat2)
	lambda1 := deg2rad(lon1)
	lambda2 := deg2rad(lon2)

	y := math.Sin(lambda2-lambda1) * math.Cos(phi2)
	x := math.Cos(phi1)*math.Sin(phi2) - math.Sin(phi1)*math.Cos(phi2)*math.Cos(lambda2-lambda1)
	theta := math.Atan2(y, x)
	return math.Mod(rad2deg(theta)+360, 360)
}

// pointToSegmentClosest returns the closest point (lat,lon) on the segment and distance in km.
// Uses equirectangular projection which is fine for short distances.
func pointToSegmentClosest(lat, lon float64, seg RoadSeg) (float64, float64, float64) {
	// Earth radius in meters for projection
	const R = 6371000.0

	// convert to radians
	lat1 := deg2rad(seg.Lat1)
	lat2 := deg2rad(seg.Lat2)
	latP := deg2rad(lat)

	// mean latitude for scaling
	latm := (lat1 + lat2 + latP) / 3.0
	cosLatm := math.Cos(latm)

	// project to local Cartesian (meters)
	x1 := (seg.Lon1 - 0) * cosLatm * R
	y1 := (seg.Lat1 - 0) * R
	x2 := (seg.Lon2 - 0) * cosLatm * R
	y2 := (seg.Lat2 - 0) * R
	xp := (lon - 0) * cosLatm * R
	yp := (lat - 0) * R

	dx := x2 - x1
	dy := y2 - y1
	if dx == 0 && dy == 0 {
		// degenerate segment
		d := math.Hypot(xp-x1, yp-y1)
		return seg.Lat1, seg.Lon1, d / 1000.0
	}

	t := ((xp-x1)*dx + (yp-y1)*dy) / (dx*dx+dy*dy)
	if t < 0 {
		// closest to start
		d := math.Hypot(xp-x1, yp-y1)
		return seg.Lat1, seg.Lon1, d / 1000.0
	}
	if t > 1 {
		// closest to end
		d := math.Hypot(xp-x2, yp-y2)
		return seg.Lat2, seg.Lon2, d / 1000.0
	}

	xc := x1 + t*dx
	yc := y1 + t*dy
	// convert back to lat/lon approximation
	latC := yc / R
	lonC := xc/(R*cosLatm)
	// convert radians to degrees
	latDeg := rad2deg(latC)
	lonDeg := rad2deg(lonC)
	d := math.Hypot(xp-xc, yp-yc)
	return latDeg, lonDeg, d / 1000.0
}

// MapMatching maps a single GPS signal to the most likely road segment and position.
func MapMatching(gps GPSData) Position {
	var best Position
	best.RoadSegID = -1
	best.Latitude = gps.Latitude
	best.Longitude = gps.Longitude

	// try to parse heading if available
	heading := -1.0
	if gps.Course != "" {
		if v, err := strconv.ParseFloat(gps.Course, 64); err == nil {
			heading = math.Mod(v+360, 360)
		}
	}

	const maxDistanceKm = 0.2 // 200 meters threshold
	minDist := math.MaxFloat64

	for _, seg := range roadNetwork {
		clLat, clLon, distKm := pointToSegmentClosest(gps.Latitude, gps.Longitude, seg)
		if distKm < minDist && distKm <= maxDistanceKm {
			// if heading is available, check bearing similarity
			segBear := bearing(seg.Lat1, seg.Lon1, seg.Lat2, seg.Lon2)
			good := true
			dir := "unknown"
			if heading >= 0 {
				diff := math.Abs(segBear - heading)
				if diff > 180 {
					diff = 360 - diff
				}
				// prefer same direction (within 90 deg)
				if diff > 90 {
					good = false
				}
				if good {
					if diff <= 90 {
						dir = "forward"
					} else {
						dir = "reverse"
					}
				}
			}

			if good {
				minDist = distKm
				best.RoadSegID = seg.ID
				best.RoadName = seg.Name
				best.Latitude = clLat
				best.Longitude = clLon
				if dir == "unknown" {
					// set based on segment bearing
					best.Direction = fmt.Sprintf("bearing: %.1f", segBear)
				} else {
					best.Direction = dir
				}
			}
		}
	}

	return best
}

func main() {
	// simple demo points
	gpsSamples := []GPSData{
		{Latitude: 37.7765, Longitude: -122.4194, Speed: 30, Course: "0"},   // near Main St
		{Latitude: 37.7760, Longitude: -122.4190, Speed: 10, Course: "90"},  // between roads
		{Latitude: 37.7760, Longitude: -122.4200, Speed: 5, Course: "270"},  // near 1st Ave
	}

	for i, g := range gpsSamples {
		pos := MapMatching(g)
		fmt.Printf("sample %d -> RoadSegID=%d Name=%s Pos=(%.6f, %.6f) Dir=%s\n",
			i+1, pos.RoadSegID, pos.RoadName, pos.Latitude, pos.Longitude, pos.Direction)
	}
}