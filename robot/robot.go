package robot

import (
	"encoding/json"
	"fmt"
	"github.com/fatih/color"
	"io/ioutil"
	"log"
	"os"
	"os/exec"
	"strconv"
	"strings"
	"time"
)

type Robot struct {
	robInterface robotInterface
	robState State
	robRos rosMiddleware
	events Events
}

func (r *Robot) Init(test bool, prms ...float64){
	r.robInterface = robotInterface{}
	r.robInterface.init(test)
	r.robState = State{}
	r.robState.init(prms)
	r.robRos = rosMiddleware{}
	r.events = Events{1.0, false, false}
	r.robRos.init()
	r.stretch()
}

func (r *Robot) Close() {
	log.Println("Closing connections!")
	r.robInterface.close()
	r.robRos.close()
	r.stretch()
}

func  (r *Robot) catchKeyboardEvent(ch chan []byte) {
	b := make([]byte, 1)
	for {
		os.Stdin.Read(b)
		ch <- b
	}
}

func  (r *Robot) updateScreen(msg string){
	fmt.Print("\033[H\033[2J")
	fmt.Print(r.robState.StringRepr())
	fmt.Println()
	fmt.Print(r.robState.StateStringRepr())
	printVoltageBar(r.events.voltageRate)
	fmt.Println()
	if r.events.ffCurrentHigh || r.events.rrCurrentHigh {
		color.Set(color.BlinkSlow, color.FgRed)
		if r.events.ffCurrentHigh {
			fmt.Print("High current in the FRONT flipper motor")
		}
		if  r.events.rrCurrentHigh {
			fmt.Print("High current in the REAR flipper motor")
		}
		color.Unset()
	}
	fmt.Println()
	fmt.Print("Z, S, Q, D to control the robot\n")
	fmt.Print("A to stop the robot\n")
	fmt.Print("B, N to release, block motors\n")
	fmt.Print("O to take the initial position\n")
	fmt.Print("R, F, T, G to move flippers\n")
	fmt.Print("Y, H, U, J to move arm joints\n")
	fmt.Print("P to close connection\n")
	fmt.Println(msg)
}

func  (r *Robot) handleKeyPress(ch chan []byte, done chan<- bool) {
	var left, right, fr, rr, arm1, arm2 int32
	var msg string
	for key := range ch {
		switch key[0] {
		case 'z':
			left, right = r.robState.setVelocity(r.robState.linear + 0.10676, r.robState.angular)
			r.robInterface.writeVel(left, right)
		case 'a':
			left, right = r.robState.setVelocity(0.0, 0.0)
			r.robInterface.writeVel(left, right)
		case 's':
			left, right = r.robState.setVelocity(r.robState.linear - 0.10676, r.robState.angular)
			r.robInterface.writeVel(left, right)
		case 'q':
			left, right = r.robState.setVelocity(r.robState.linear, r.robState.angular - 0.10676)
			r.robInterface.writeVel(left, right)
		case 'd':
			left, right = r.robState.setVelocity(r.robState.linear, r.robState.angular + 0.10676)
			r.robInterface.writeVel(left, right)
		case 'r':
			fr, rr = r.robState.incFlipper(0.157, 0.0)
			r.robInterface.writeFlip(fr, rr)
		case 'f':
			fr, rr = r.robState.incFlipper(-0.157, 0.0)
			r.robInterface.writeFlip(fr, rr)
		case 't':
			fr, rr = r.robState.incFlipper(0.0, 0.157)
			r.robInterface.writeFlip(fr, rr)
		case 'g':
			fr, rr = r.robState.incFlipper(0.0, -0.157)
			r.robInterface.writeFlip(fr, rr)
		case 'y':
			arm1, arm2 = r.robState.incArm(0.157, 0.0)
			r.robInterface.writeArm(arm1, arm2)
		case 'h':
			arm1, arm2 = r.robState.incArm(-0.157, 0.0)
			r.robInterface.writeArm(arm1, arm2)
		case 'u':
			arm1, arm2 = r.robState.incArm(0.0, 0.157)
			r.robInterface.writeArm(arm1, arm2)
		case 'j':
			arm1, arm2 = r.robState.incArm(0.0, -0.157)
			r.robInterface.writeArm(arm1, arm2)
		case 'b':
			r.robInterface.releaseMotors()
		case 'n':
			r.robInterface.stopMotors()
		case 'o':
			r.stretch()
		case 'p':
			exec.Command("stty", "-F", "/dev/tty", "echo").Run()
			done <- true
		}
		state := r.robState.getState()
		r.saveState(state)
		r.updateScreen(msg)
	}
}

func (r *Robot) Keyboard(done chan<- bool) {
	// disable input buffering
	exec.Command("stty", "-F", "/dev/tty", "cbreak", "min", "1").Run()
	// do not display entered characters on the screen
	exec.Command("stty", "-F", "/dev/tty", "-echo").Run()
	ch := make(chan []byte, 10)
	go r.catchKeyboardEvent(ch)
	go r.handleKeyPress(ch, done)
}

func (r *Robot) handleRPC(msg []byte, res chan []byte, done chan<- bool) {
	var request map[string]interface{}
	json.Unmarshal(msg, &request)
	if _, ok := request["linear"]; ok {
		// -|angular| <- turn left
		left, right := r.robState.setVelocity(request["linear"].(float64), request["angular"].(float64))
		r.robInterface.writeVel(left, right)
	}
	if _, ok := request["front"]; ok {
		fr, rr := r.robState.incFlipper(request["front"].(float64), request["rear"].(float64))
		r.robInterface.writeFlip(fr, rr)
	}
	if _, ok := request["arm1"]; ok {
		arm1, arm2 := r.robState.incArm(request["arm1"].(float64), request["arm2"].(float64))
		r.robInterface.writeFlip(arm1, arm2)
	}
	if _, ok := request["cmd"]; ok {
		if request["cmd"] == 0.0 {
			done <- true
		}
	}
	state := r.robState.getState()
	r.saveState(state)
	jsonState, _ := json.Marshal(state)
	res <- jsonState
}

func (r *Robot) handleSensorData(sensorChan <-chan string) {
	for line := range sensorChan {
		if strings.HasPrefix(line, "#") {
			strArray := strings.Split(line, ",")
			r.robState.sensor.Yaw, _ = strconv.ParseFloat(strArray[2], 64)
			r.robState.sensor.GyroX, _ = strconv.ParseFloat(strArray[4], 64)
			r.robState.sensor.GyroY, _ = strconv.ParseFloat(strArray[5], 64)
			r.robState.sensor.GyroZ, _ = strconv.ParseFloat(strArray[6], 64)
			r.robState.sensor.AccelX, _ = strconv.ParseFloat(strArray[8], 64)
			r.robState.sensor.AccelY, _ = strconv.ParseFloat(strArray[9], 64)
			r.robState.sensor.AccelZ, _ = strconv.ParseFloat(strArray[10], 64)
		} else if strings.HasPrefix(line, "MM") {
			// V(motorBoard data) AI(motor temp) A(current) C(position data)
			index := int32(line[2])
			line = line[0:4]
			if index == 2 && strings.HasPrefix(line, "V") {
				voltageInt, _ := strconv.ParseInt(strings.Split(line[2:], ":")[1], 10, 64)
				r.robState.sensor.Voltage = float64(voltageInt) / 10
			}
			if strings.HasPrefix(line, "C") {
				posEncoders := strings.Split(line[2:], ":")
				pos1, _ := strconv.ParseInt(posEncoders[0], 10, 64)
				pos2, _ := strconv.ParseInt(posEncoders[0], 10, 64)
				switch index {
				case 0:
					r.robState.sensor.LeftMotorCountsFront = pos1
					r.robState.sensor.RightMotorCountsFront = pos2
				case 1:
					r.robState.sensor.LeftMotorCountsRear = pos1
					r.robState.sensor.RightMotorCountsRear = pos2
				case 2:
					r.robState.sensor.FrontFlipperCounts = pos1
					r.robState.sensor.RearFlipperCounts = pos2
				}
			}
			if index == 2 && strings.HasPrefix(line, "A"){
				currents := strings.Split(line[2:], ":")
				currentFront, _ := strconv.ParseInt(currents[0], 10, 64)
				currentRear, _ := strconv.ParseInt(currents[0], 10, 64)
				r.robState.sensor.FrontFlipperCurrent = float64(currentFront) / 10
				r.robState.sensor.FrontFlipperCurrent = float64(currentRear) / 10
			}
		}
	}
}

func (r *Robot) Sensors() {
	sensorData := make(chan string)
	go r.robInterface.readSensors(sensorData)
	go r.handleSensorData(sensorData)

	ticker := time.NewTicker(200 * time.Millisecond)
	for {
		select {
		case <-ticker.C:
			r.robRos.publishState(r.robState.getSensorJson())
		}
	}
}

func (r *Robot) RPC(done chan<- bool) {
	r.robInterface.releaseMotors()
	req := make(chan []byte)
	res := make(chan []byte)
	go r.robRos.consume(req, res)
	for {
		msg := <-req
		go r.handleRPC(msg, res, done)
	}
}
func (r *Robot) Security() {
	ticker := time.NewTicker(100 * time.Millisecond)
	for {
		select {
		case <-ticker.C:
			r.events.voltageRate = int64(1.0 - (25.2 - r.robState.sensor.Voltage) / (25.2 - 18.0))*10
			if r.robState.sensor.FrontFlipperCurrent / 25.0 > 0.8 {
				r.events.ffCurrentHigh = true
			} else {
				r.events.ffCurrentHigh = false
			}
			if r.robState.sensor.RearFlipperCurrent / 25.0 > 0.8 {
				r.events.rrCurrentHigh = true
			} else {
				r.events.rrCurrentHigh = false
			}
		}
	}
}

func (r *Robot) saveState(state map[string]float64) {
	delete(state, "linear")
	delete(state, "angular")
	jsonState, _ := json.Marshal(state)
	_ = ioutil.WriteFile("state.json", jsonState, 0644)
}

func (r *Robot) stretch() {
	// This method analyse the last saved state and stretchs its parts to the initial position
	state := map[string]float64{}
	jsonState, _ := ioutil.ReadFile("state.json")
	_ = json.Unmarshal(jsonState, &state)
	r.robState.front = state["front"]
	r.robState.rear = state["rear"]
	r.robState.arm1 = state["arm1"]
	r.robState.arm2 = state["arm2"]
	for k, v := range state {
		state[k] = -v
	}
	fr, rr := r.robState.incFlipper(state["front"], state["rear"])
	r.robInterface.writeFlip(fr, rr)
	arm1, arm2 := r.robState.incArm(state["arm1"], state["arm2"])
	r.robInterface.writeFlip(arm1, arm2)
	r.saveState(r.robState.getState())
}
