using ESPController.Models;
using MQTTnet;
using MQTTnet.Client;
using MQTTnet.Client.Options;
using Spectre.Console;
// ReSharper disable SuggestBaseTypeForParameter

const string mqttMainTopic = "esp/main";

var factory = new MqttFactory();
var options = new MqttClientOptionsBuilder()
    .WithClientId("dotnet-mqtt-client")
    .WithTcpServer("192.168.43.187")
    .WithCleanSession()
    .Build();

var mqttClient = factory.CreateMqttClient();

AnsiConsole.WriteLine("Connecting to MQTT broker...");
await mqttClient.ConnectAsync(options, CancellationToken.None);
while (mqttClient.IsConnected is false)
{
    AnsiConsole.WriteLine("Waiting for connection...");
}

AnsiConsole.WriteLine("Connected to MQTT broker.");

while (true)
{
    var input = AnsiConsole.Ask<string>("Command => ");
    var commands = input.Split(" ");
    var command = commands[0];
    var param = commands.Skip(1).ToArray();
    switch (command)
    {
        case "exit":
            Environment.Exit(0);
            break;
        case "mode":
            var mode = param[0];
            param = param.Skip(1).ToArray();
            ModeExecutor(mode, param);
            break;
        case "command":
            var cmd = param[0];
            param = param.Skip(1).ToArray();
            CommandExecutor(cmd, param);
            break;
    }
}

void ModeExecutor(string mode, string[] param)
{
    switch (mode)
    {
        case "single":
            var singleData = ModeJsonStrings.GetModeJsonString(Modes.Single, param);
            MqttPublish(singleData).Wait();
            break;
        case "clock":
            var clockData = ModeJsonStrings.GetModeJsonString(Modes.Clock, param);
            MqttPublish(clockData).Wait();
            break;
        case "picture":
            var pictureData = ModeJsonStrings.GetModeJsonString(Modes.Picture, param);
            var payloads = pictureData.Split("||");
            foreach (var data in payloads)
            {
                MqttPublish(data).Wait();
            }
            break;
        case "picture_group":
            var pictureGroupData = ModeJsonStrings.GetModeJsonString(Modes.PictureGroup, param);
            var payloadsGroup = pictureGroupData.Split("&&");
            var payload = payloadsGroup.Select(x => x.Split("||")).ToList();
            var inf = param[1] == "inf";
            if (inf)
            {
                while (true)
                {
                    foreach (var picturePayloads in payload)
                    {
                        foreach (var picturePayload in picturePayloads)
                        {
                            MqttPublish(picturePayload).Wait();
                            Thread.Sleep(100);
                        }
                        Thread.Sleep(1000);
                    }
                }
            }
            else
            {
                if (int.TryParse(param[1], out var times) is false)
                {
                    throw new Exception("Invalid times");
                }
                for (var i = 0; i < times; i++)
                {
                    foreach (var picturePayloads in payload)
                    {
                        foreach (var picturePayload in picturePayloads)
                        {
                            MqttPublish(picturePayload).Wait();
                            Thread.Sleep(100);
                        }
                        Thread.Sleep(1000);
                    }
                }
            }
            break;
        case "stop":
            var stopData = ModeJsonStrings.GetModeJsonString(Modes.Stop, Array.Empty<string>());
            MqttPublish(stopData).Wait();
            break;
    }
}
void CommandExecutor(string command, string[] param)
{
    switch (command)
    {
        case "set":
            var setParam = param.Skip(1).ToArray();
            CommandSetExecutor(param[0], setParam);
            break;
    }
}
void CommandSetExecutor(string command, string[] param)
{
    switch (command)
    {
        case "general_lightness":
            var generalLightnessData = CommandJsonString.GetCommandJsonString(Commands.SetGeneralLightness, param);
            MqttPublish(generalLightnessData).Wait();
            break;
        case "digit_color":
            var digitColorData = CommandJsonString.GetCommandJsonString(Commands.SetDigitColor, param);
            MqttPublish(digitColorData).Wait();
            break;
    }
}
async Task MqttPublish(string payload)
{
    await mqttClient.PublishAsync(mqttMainTopic, payload);
}
