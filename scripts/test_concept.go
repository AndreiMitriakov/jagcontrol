package main

import (
	"bufio"
	"os"
	"strconv"
	"strings"
)

type Sensor struct {
	frTemp float64 // front flipper temp
	rrTemp float64 // rear flipper temp
	frCur float64 // front flipper current
	rrCur float64 // rear flipper current
	battVolt float64 // battery voltage
	leftEnc float64 // battery voltage
	rightEnc float64 // battery voltage
	accelX float64
	accelY float64
	accelZ float64
	gyroX float64
	gyroY float64
	gyroZ float64
	yaw float64
	leftMotorCounts int64 // left side mean motor encoders counts
	rightMotorCounts int64 // right side mean motor encoders counts
	boardTemp float64 // board temperature
}

func main(){
	f, _ := os.OpenFile("string_data", os.O_RDONLY, 0644)
	scanner := bufio.NewScanner(f)
	sensor := Sensor{}
	for scanner.Scan() {
		line := scanner.Text()

		if strings.HasPrefix(line, "#") {
			strArray := strings.Split(line, ",")
			sensor.yaw, _ = strconv.ParseFloat(strArray[2], 64)
			sensor.gyroX, _ = strconv.ParseFloat(strArray[4], 64)
			sensor.gyroY, _ = strconv.ParseFloat(strArray[5], 64)
			sensor.gyroZ, _ = strconv.ParseFloat(strArray[6], 64)
			sensor.accelX, _ = strconv.ParseFloat(strArray[8], 64)
			sensor.accelY, _ = strconv.ParseFloat(strArray[9], 64)
			sensor.accelZ, _ = strconv.ParseFloat(strArray[10], 64)
		} else if strings.HasPrefix(line, "MM") {
			index := int32(line[2])
			line = line[0:4]
			if index == 2 {
				if strings.HasPrefix(line, "V") {

				} else if strings.HasPrefix(line, "AI") {

				} else if strings.HasPrefix(line, "A") {

				} else if strings.HasPrefix(line, "C") {

				}
			}
			// V(motorBoard data) AI(motor temp) A(current) C(position data)
		}

	}
}