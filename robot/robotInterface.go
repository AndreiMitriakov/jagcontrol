package robot

import (
	"net"
)

type robotInterface struct {
	connectionBase net.Conn
	connectionArm net.Conn
}

func (p *robotInterface) init(){
	// 192.168.0.60:10001 base
	// 192.168.0.63:10001 arm
	// connection to base
	conn_base, err := net.Dial("tcp", "localhost:10001")
	if err != nil {
		panic("No connection to the robot base!")
	}
	// connection to arm
	conn_arm, err := net.Dial("tcp", "localhost:10002")
	if err != nil {
		panic("No connection to the robot arm!")
	}
	p.connectionBase = conn_base
	p.connectionArm = conn_arm
}

