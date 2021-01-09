package robot

import (
	"fmt"
	"github.com/streadway/amqp"
)

type rosMiddleware struct {
	conn amqp.Connection
	ch amqp.Channel
	stateCh amqp.Channel
	stateQueue amqp.Queue
}

func (m *rosMiddleware) init() {
	conn, err := amqp.Dial("amqp://guest:guest@localhost:5672/")
	failOnError(err, "Failed to connect to RabbitMQ")
	m.conn = *conn

	ch, err := conn.Channel()
	failOnError(err, "Failed to open a channel")
	m.ch = *ch
	fmt.Println("Connection AMQP")


	stateCh, err := conn.Channel()
	failOnError(err, "Failed to open a channel")
	m.stateCh = *stateCh
	// State queue
	m.stateQueue, err = m.stateCh.QueueDeclare(
		"robotState", // name
		false,   // durable
		false,   // delete when unused
		false,   // exclusive
		false,   // no-wait
		nil,     // arguments
	)
	failOnError(err, "Failed to declare the state queue")
}

func (m *rosMiddleware) consume(req, res chan []byte){
	q, err := m.ch.QueueDeclare("rpc_queue", false, false, false, false, nil)
	failOnError(err, "Failed to declare a queue")

	err = m.ch.Qos(1, 0, false)
	failOnError(err, "Failed to set QoS")

	msgs, err := m.ch.Consume(q.Name,"", false, false, false, false, nil)
	failOnError(err, "Failed to register a consumer")

	for d := range msgs {
		req <- d.Body
		err = m.ch.Publish("", d.ReplyTo, false, false,
			amqp.Publishing{ ContentType: "text/plain", CorrelationId: d.CorrelationId, Body: <-res},
		)
		failOnError(err, "Failed to publish a message")
		d.Ack(false)
	}
}

func (m *rosMiddleware) publishState(body string) {
	err := m.stateCh.Publish(
		"",     // exchange
		m.stateQueue.Name, // routing key
		false,  // mandatory
		false,  // immediate
		amqp.Publishing {
			ContentType: "text/plain",
			Body:        []byte(body),
		})
	failOnError(err, "Failed to publish a message")
}

func (m *rosMiddleware) close() {
	m.ch.Close()
	// m.conn.Close()
}