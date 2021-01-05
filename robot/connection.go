package robot

import "net"

type connection struct {
	conn net.Dialer
}

func (p *connection) init(){

}