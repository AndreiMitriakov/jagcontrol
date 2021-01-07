package main

import (
	"flag"
	"github.com/AndreiMitriakov/jagcontrol/robot"
	"log"
)

func main(){
<<<<<<< HEAD
	r := robot.Robot{}
	r.Init(0, 0, 0, 0, 0, 0)
	r.Operate()
=======
	boolTest := flag.Bool("test", false, "working in test mode?")
	flag.Parse()
	log.Printf("Working in real mode [%t]\n", !*boolTest)
>>>>>>> 2182e14ad05a80f8166031e8484424816f7d4a23

	jaguar := robot.Robot{}
	jaguar.Init(*boolTest, 0., 0., 0., 0., 0., 0.)
	defer jaguar.Close()
	done := make(chan bool)
	go jaguar.Keyboard(done)
	go jaguar.RPC(done)
	<-done
}
