using MQTTnet;
using MQTTnet.Client;
using MQTTnet.Client.Options;
using Serilog;

var logger = new LoggerConfiguration()
    .WriteTo.Console()
    .CreateLogger();

var factory = new MqttFactory();
var options = new MqttClientOptionsBuilder()
    .WithClientId("dotnet-mqtt-logger")
    .WithTcpServer("192.168.43.187")            // MQTT broker address
    .WithCleanSession()
    .Build();

var mqttClient = factory.CreateMqttClient();

logger.Information("Connecting to MQTT server...");
await mqttClient.ConnectAsync(options, CancellationToken.None);
while (mqttClient.IsConnected is false)
{
    logger.Information("Waiting for connection...");
}

logger.Information("Connected to MQTT server.");

mqttClient.UseApplicationMessageReceivedHandler(eventArgs =>
{
    logger.Information($"{eventArgs.ApplicationMessage.ConvertPayloadToString()}");
});

await mqttClient.SubscribeAsync("esp/log");

var key = new ConsoleKeyInfo();

while (key != new ConsoleKeyInfo('Q', ConsoleKey.Q, false, false, true))
{
    key = Console.ReadKey();
}

logger.Information("Exiting...");
await mqttClient.DisconnectAsync();
logger.Information("Exited.");
