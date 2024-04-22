//sample base on https://blog.behroozbc.ir/mqtt-client-with-mqttnet-4-and-c
using MQTTnet.Client;
using MQTTnet.Extensions.ManagedClient;
using MQTTnet;
using System.Text.Json;

var YELLOW_TOPIC = "test-topic-student-pak-yellow";
var GREEN_TOPIC = "test-topic-student-pak-green";
var BLUE_TOPIC = "test-topic-student-pak-blue";
var MaxValue = 255;

IManagedMqttClient _mqttClient = new MqttFactory().CreateManagedMqttClient();

var builder = new MqttClientOptionsBuilder().WithClientId(Guid.NewGuid().ToString())
    .WithTcpServer("test.mosquitto.org");

var options = new ManagedMqttClientOptionsBuilder()
                        .WithAutoReconnectDelay(TimeSpan.FromSeconds(60))
                        .WithClientOptions(builder.Build())
                        .Build();

_mqttClient.ConnectedAsync += _mqttClient_ConnectedAsync;
_mqttClient.DisconnectedAsync += _mqttClient_DisconnectedAsync;
_mqttClient.ConnectingFailedAsync += _mqttClient_ConnectingFailedAsync;
_mqttClient.ApplicationMessageReceivedAsync += _mqqtClient_MessageReceived;
await _mqttClient.StartAsync(options);
await _mqttClient.SubscribeAsync(BLUE_TOPIC);
var rand = new Random();
while (true)
{
    var topic = rand.Next(2) == 0 ? YELLOW_TOPIC : GREEN_TOPIC;
    var json = JsonSerializer.Serialize(new { Value = rand.Next(MaxValue + 1), Sent = DateTime.UtcNow });
    await _mqttClient.EnqueueAsync(topic, json);
    Console.WriteLine($"send: {json} to {topic}");

    await Task.Delay(TimeSpan.FromSeconds(2));
}

Task _mqttClient_ConnectedAsync(MqttClientConnectedEventArgs arg)
{
    Console.WriteLine("Connected");
    return Task.CompletedTask;
};

Task _mqttClient_DisconnectedAsync(MqttClientDisconnectedEventArgs arg)
{
    Console.WriteLine("Disconnected");
    return Task.CompletedTask;
};

Task _mqttClient_ConnectingFailedAsync(ConnectingFailedEventArgs arg)
{
    Console.WriteLine("Connection failed check network or broker!");
    return Task.CompletedTask;
}

Task _mqqtClient_MessageReceived(MqttApplicationMessageReceivedEventArgs msg)
{
    var payload = msg.ApplicationMessage.PayloadSegment;
    if (payload.Count == 0)
        return Task.CompletedTask;

    var jsonDoc = JsonDocument.Parse(payload);
    var root = jsonDoc.RootElement;
    if (root.TryGetProperty("value", out var valueElement))
    {
        try
        {
            var value = valueElement.GetInt16();
            Console.WriteLine($"rec: {msg.ApplicationMessage.Topic} - {value}");
        }
        catch (Exception e)
        {
            Console.WriteLine(e.Message);
        }
    }

    return Task.CompletedTask;
}
