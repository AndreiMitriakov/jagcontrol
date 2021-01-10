package robot

import (
	"fmt"
	"github.com/fatih/color"
	"log"
)

func failOnError(err error, msg string) {
	if err != nil {
		log.Fatalf("%s: %s", msg, err)
	}
}

type Events struct {
	voltageRate int64
	ffCurrentHigh bool
	rrCurrentHigh bool
}

func printVoltageBar(level int64) {
	elem := "="
	for i:=1; i<int(level+1); i++{
		switch i {
		case 1, 2:
			color.Set(color.FgRed)
		case 3, 4:
			color.Set(color.FgYellow)
		case 5, 6:
			color.Set(color.FgHiYellow)
		case 7, 8:
			color.Set(color.FgHiGreen)
		case 9, 10:
			color.Set(color.FgGreen)
		}
		fmt.Print(elem)
	}
	fmt.Print("\n")
	color.Unset()
}