package robot

import (
	"fmt"
	"github.com/streadway/amqp"
)

type rosMiddleware struct {
	conn amqp.Connection
	ch amqp.Channel
}

func (m *rosMiddleware) init() {
	conn, err := amqp.Dial("amqp://guest:guest@localhost:5672/")
	failOnError(err, "Failed to connect to RabbitMQ")
	m.conn = *conn
	ch, err := conn.Channel()
	failOnError(err, "Failed to open a channel")
	m.ch = *ch
	fmt.Println("Connection AMQP")
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

func (m *rosMiddleware) close() {
	// m.ch.Close()
	m.conn.Close()
}