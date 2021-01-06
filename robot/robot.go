package robot

import (
	"fmt"
	"os"
	"os/exec"
)

type Robot struct {
	robInterface robotInterface
	robState State
}

func (r *Robot) Init(prms ...float32){
	r.robInterface = robotInterface{}
	r.robInterface.init()
	r.robState = State{}
	r.robState.init(prms)
}

func (r *Robot) Close() {
	r.robInterface.close()
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
		case 's':
			left, right = r.robState.setVelocity(r.robState.linear - 0.10676, r.robState.angular)
			r.robInterface.writeVel(left, right)
		case 'q':
			left, right = r.robState.setVelocity(r.robState.linear, r.robState.angular + 0.10676)
			r.robInterface.writeVel(left, right)
		case 'd':
			left, right = r.robState.setVelocity(r.robState.linear, r.robState.angular - 0.10676)
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
		case 'p':
			r.robInterface.close()
			exec.Command("stty", "-F", "/dev/tty", "echo").Run()
			msg = "Closing connection!"
			fmt.Println(msg)
			done <- true
		}
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

func (r *Robot) RPC() {
	fmt.Println("RPC")
}


