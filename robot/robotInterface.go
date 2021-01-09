package robot

import (
	"bufio"
	"log"
	"net"
	"strconv"
	"strings"
	"time"
)

/*
	This package is mentioned to provide communication with the robot:
		I maintain tcp/ip connection with the robot,
		  transform sending raw_data into a string protocol command,
		  serialize command,
		  send it.
		II receive information about the robot base state.
*/


type robotInterface struct {
	connectionBase net.Conn
	connectionArm net.Conn
}

func (p *robotInterface) init(test bool){
	// 192.168.0.60:10001 base
	// 192.168.0.63:10001 arm
	// connection to base
	var addrBase, addrArm string
	if test {
		addrBase = "localhost:10001"
		addrArm = "localhost:10002"
	} else {
		addrBase = "192.168.0.60:10001"
		addrArm = "192.168.0.63:10001"
	}
	conn_base, err := net.Dial("tcp", addrBase)
	if err != nil {
		panic("No connection to the robot base!")
	}
	// connection to arm
	conn_arm, err := net.Dial("tcp", addrArm)
	if err != nil {
		panic("No connection to the robot arm!")
	}
	p.connectionBase = conn_base
	p.connectionArm = conn_arm
	go p.ping()
}

func (p *robotInterface) close() {
	var err interface{}
	if p.connectionArm != nil {
		err = p.connectionArm.Close()
		if err != nil {
			panic("Can not close the arm connection")
		}
		p.connectionArm = nil
	}
	if p.connectionBase != nil {
		err = p.connectionBase.Close()
		p.connectionBase = nil
		if err != nil {
			panic("Can not close the base connection")
		}
	}
}

func (p *robotInterface) writeVel(left, right int32) {
	var cmd string = "MMW !M " + strconv.Itoa(int(-left)) + " " + strconv.Itoa(int(right)) + "\r\n"
	p.writeToBaseBoard(cmd)
	// fmt.Println("Written %d bytes %s", n)
}

func (p *robotInterface) writeFlip(fr, rr int32) {
	// front
	// "MM2 !PR 1 " + QString::number(leftFrontCmd) + "\r\n"
	// "MM2 !PR 2 " + QString::number(rightFrontCmd) + "\r\n";
	var cmd = "MM2 !PR 1 " + strconv.Itoa(int(fr)) + "\r\n"
	p.writeToBaseBoard(cmd)
	cmd = "MM2 !PR 2 " + strconv.Itoa(int(rr)) + "\r\n"
	p.writeToBaseBoard(cmd)
	// rear
	// "MM3 !PR 1 " + QString::number(leftRearCmd) + "\r\n";
	// "MM3 !PR 2 " + QString::number(rightRearCmd) + "\r\n"
	// cmd = "MM3 !PR 1 " + strconv.Itoa(int(rr)) + "\r\n"
	// p.writeToBaseBoard(cmd)
	// cmd = "MM3 !PR 2 " + strconv.Itoa(int(rr)) + "\r\n"
	// p.writeToBaseBoard(cmd)
}

func (p *robotInterface) writeArm(arm1, arm2 int32) {
	// '!PR 1 '+str(joint1)+'\r'
	// '!PR 2 '+str(joint1)+'\r'
	var cmd string
	cmd = "!PR 1 " + strconv.Itoa(int(arm1)) + "\r"
	p.writeToArmBoard(cmd)
	cmd = "!PR 2 " + strconv.Itoa(int(-arm2)) + "\r"
	p.writeToArmBoard(cmd)
}


func (p *robotInterface) stopMotors() {
	stopBase := "MMW !EX\r\n"
	p.writeToBaseBoard(stopBase)
	stopFF := "MM2 !EX\r\n"
	p.writeToBaseBoard(stopFF)
	stopRF := "MM3 !EX\r\n"
	p.writeToBaseBoard(stopRF)
}

func (p *robotInterface) readSensors(sensorChan chan<- string) {
	reader := bufio.NewReader(p.connectionBase)
	for {
		ba, _, err := reader.ReadLine()
		if err != nil {
			str := strings.Trim(string(ba), "[]")
			sensorChan <- str
		} else {
			log.Println("Can not read from the robot")
		}

	}
}

func (p *robotInterface) ping() {
	ticker := time.NewTicker(200 * time.Millisecond)
	cmd := "PING\r\n"
	for {
		select {
		case <-ticker.C:
			p.connectionBase.Write([]byte(cmd))
		}
	}
}


func (p *robotInterface) releaseMotors() {
	releaseBase := "MMW !MG\r\n"
	p.writeToBaseBoard(releaseBase)
	releaseFF := "MM2 !MG\r\n"
	p.writeToBaseBoard(releaseFF)
	releaseRR := "MM3 !EX\r\n"
	p.writeToBaseBoard(releaseRR)
}

func (p *robotInterface) writeToBaseBoard(cmd string) {
	_, err := p.connectionBase.Write([]byte(cmd))
	failOnError(err, "")
}

func (p *robotInterface) writeToArmBoard(cmd string) {
	_, err := p.connectionArm.Write([]byte(cmd))
	failOnError(err, "")
}
