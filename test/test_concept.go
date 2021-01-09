package main

import (
	"bufio"
	"os"
	"strconv"
	"strings"
)

type Sensor struct {
	frontFlipperCurrent float64 // front flipper current
	rearFlipperCurrent float64 // rear flipper current
	voltage float64
	accelX float64
	accelY float64
	accelZ float64
	gyroX float64
	gyroY float64
	gyroZ float64
	yaw float64
	leftMotorCountsFront int64 // left side mean motor encoders counts
	leftMotorCountsRear int64 // left side mean motor encoders counts
	rightMotorCountsFront int64 // left side mean motor encoders counts
	rightMotorCountsRear int64 // left side mean motor encoders counts
	frontFlipperCounts int64
	rearFlipperCounts int64
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
			// V(motorBoard data) AI(motor temp) A(current) C(position data)
			index := int32(line[2])
			line = line[0:4]
			if index == 2 && strings.HasPrefix(line, "V") {
				voltageInt, _ := strconv.ParseInt(strings.Split(line[2:], ":")[1], 10, 64)
				sensor.voltage = float64(voltageInt) / 10
			}
			if strings.HasPrefix(line, "C") {
				posEncoders := strings.Split(line[2:], ":")
				pos1, _ := strconv.ParseInt(posEncoders[0], 10, 64)
				pos2, _ := strconv.ParseInt(posEncoders[0], 10, 64)
				switch index {
				case 0:
					sensor.leftMotorCountsFront = pos1
					sensor.rightMotorCountsFront = pos2
				case 1:
					sensor.leftMotorCountsRear = pos1
					sensor.rightMotorCountsRear = pos2
				case 2:
					sensor.frontFlipperCounts = pos1
					sensor.rearFlipperCounts = pos2
				}
			}
			if index == 2 && strings.HasPrefix(line, "A"){
				currents := strings.Split(line[2:], ":")
				currentFront, _ := strconv.ParseInt(currents[0], 10, 64)
				currentRear, _ := strconv.ParseInt(currents[0], 10, 64)
				sensor.frontFlipperCurrent = float64(currentFront) / 10
				sensor.frontFlipperCurrent = float64(currentRear) / 10
			}
		}
		// fmt.Println(sensor)
	}
}