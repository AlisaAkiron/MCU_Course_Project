using System.Text.Json;
using ESPController.Models;
using MQTTnet;
using MQTTnet.Client;
using MQTTnet.Client.Options;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;
using Spectre.Console;

var factory = new MqttFactory();
var options = new MqttClientOptionsBuilder()
    .WithClientId("dotnet-mqtt-client")
    .WithTcpServer("server.liamsho.xyz")
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
    var mode = AnsiConsole.Prompt(new SelectionPrompt<string>()
        .Title("Select mode")
        .PageSize(10)
        .MoreChoicesText("More...")
        .AddChoices("single", "clock", "frame_animation", "command", "stop"));
    switch (mode)
    {
        case "single":
            var singleX = AnsiConsole.Ask<int>("X: ");
            var singleY = AnsiConsole.Ask<int>("Y: ");
            var singleColorH = AnsiConsole.Ask<int>("HSL H: ");
            var singleColorS = AnsiConsole.Ask<int>("HSL S: ");
            var singleColorL = AnsiConsole.Ask<int>("HSL L: ");
            var singleObj = new Base<SingleDisplayMode>("single",
                new SingleDisplayMode(singleX, singleY, singleColorH, singleColorS, singleColorL));
            var singlePayload = JsonSerializer.Serialize(singleObj);
            await mqttClient.PublishAsync("esp32", singlePayload);
            AnsiConsole.WriteLine("\"Single\" mode data sent.");
            break;
        case "clock":
            var clockTz = AnsiConsole.Ask<int>("Timezone: ");
            var clockObj = new Base<ClockDisplayMode>("clock",
                new ClockDisplayMode(clockTz));
            var clockPayload = JsonSerializer.Serialize(clockObj);
            await mqttClient.PublishAsync("esp32", clockPayload);
            AnsiConsole.WriteLine("\"Clock\" mode data sent.");
            break;
        case "frame_animation":
            var frameAnimationObj = new Base<EmptyMode>("frame_animation", new EmptyMode());
            var frameAnimationPayload = JsonSerializer.Serialize(frameAnimationObj);
            await mqttClient.PublishAsync("esp32", frameAnimationPayload);
            AnsiConsole.WriteLine("\"Frame animation\" mode data sent.");
            break;
        case "command":
            var commandType = AnsiConsole.Prompt(new SelectionPrompt<string>()
                .Title("Select command type")
                .PageSize(10)
                .MoreChoicesText("More...")
                .AddChoices("set", "frame"));
            switch (commandType)
            {
                case "set":
                    var commandSetParam = AnsiConsole.Prompt(new SelectionPrompt<string>()
                        .Title("Select set command parameter")
                        .PageSize(10)
                        .MoreChoicesText("More...")
                        .AddChoices("general_lightness", "digit_color"));
                    switch (commandSetParam)
                    {
                        case "general_lightness":
                            var commandSetGeneralLightness = AnsiConsole.Ask<int>("Lightness: ");
                            var commandSetGeneralLightnessObj = new Base<CommandSetGeneralLightnessMode>("command",
                                new CommandSetGeneralLightnessMode("set", "general_lightness", commandSetGeneralLightness));
                            var commandSetGeneralLightnessPayload = JsonSerializer.Serialize(commandSetGeneralLightnessObj);
                            await mqttClient.PublishAsync("esp32", commandSetGeneralLightnessPayload);
                            AnsiConsole.WriteLine("\"Command - SET\" mode set \"general lightness\" param data sent.");
                            break;
                        case "digit_color":
                            var commandSetDigitColorType = AnsiConsole.Prompt(new SelectionPrompt<string>()
                                .Title("Select digit color type")
                                .PageSize(10)
                                .MoreChoicesText("More...")
                                .AddChoices("hsl", "rgb"));
                            var commandSetDigitColorC1 = AnsiConsole.Ask<int>(commandSetDigitColorType == "hsl" ? "H: " : "R: ");
                            var commandSetDigitColorC2 = AnsiConsole.Ask<int>(commandSetDigitColorType == "hsl" ? "S: " : "G: ");
                            var commandSetDigitColorC3 = AnsiConsole.Ask<int>(commandSetDigitColorType == "hsl" ? "L: " : "B: ");
                            var commandSetDigitColorObj = new Base<CommandSetDigitColorMode>("command",
                                new CommandSetDigitColorMode("type", "digit_color", commandSetDigitColorType, commandSetDigitColorC1, commandSetDigitColorC2, commandSetDigitColorC3));
                            var commandSetDigitColorPayload = JsonSerializer.Serialize(commandSetDigitColorObj);
                            await mqttClient.PublishAsync("esp32", commandSetDigitColorPayload);
                            AnsiConsole.WriteLine("\"Command - SET\" mode set \"digit color\" param data sent.");
                            break;
                    }
                    break;
                case "frame":
                    var commandFrameType = AnsiConsole.Prompt(new SelectionPrompt<string>()
                        .Title("Select frame type")
                        .PageSize(10)
                        .MoreChoicesText("More...")
                        .AddChoices("add", "empty"));
                    switch (commandFrameType)
                    {
                        case "add":
                            var commandFrameDirectory = AnsiConsole.Ask<string>("Directory: ");
                            var di = new DirectoryInfo(commandFrameDirectory);
                            if (di.Exists is false)
                            {
                                AnsiConsole.WriteLine("Directory not found.");
                                break;
                            }
                            var fis = di.GetFiles();
                            fis = fis
                                .Where(x => x.FullName.ToLower().EndsWith("png") ||
                                            x.FullName.ToLower().EndsWith("jpg") ||
                                            x.FullName.ToLower().EndsWith("jpeg") ||
                                            x.FullName.ToLower().EndsWith("bmp"))
                                .OrderBy(x => x.Name).ToArray();
                            AnsiConsole.WriteLine("Total frames: " + fis.Length);
                            var rgb = new List<List<RgbColor>>();
                            foreach (var fi in fis)
                            {
                                var data = GetImageBytes(fi.FullName);
                                rgb.Add(new List<RgbColor>(data.Select(d => new RgbColor(d.B, d.G, d.R))));
                            }

                            var commandFrameAddObjList = new List<Base<CommandFrameAddMode>>();
                            foreach (var c in rgb)
                            {
                                var prev = c.Take(128).ToList();
                                var next = c.Skip(128).Take(128).ToList();
                                var commandFrameAddObj1 = new Base<CommandFrameAddMode>("command",
                                    new CommandFrameAddMode(prev, "prev"));
                                var commandFrameAddObj2 = new Base<CommandFrameAddMode>("command",
                                    new CommandFrameAddMode(prev, "next"));
                                commandFrameAddObjList.Add(commandFrameAddObj1);
                                commandFrameAddObjList.Add(commandFrameAddObj2);
                            }
                            for (var i = 1; i <= commandFrameAddObjList.Count; i++)
                            {
                                var commandFrameAddObjPayload = JsonSerializer.Serialize(commandFrameAddObjList[i - 1]);
                                await mqttClient.PublishAsync("esp32", commandFrameAddObjPayload);
                                Thread.Sleep(10000);
                                AnsiConsole.WriteLine($"\"Command - FRAME\" mode add frame data sent. ({i}/{commandFrameAddObjList.Count})");
                            }
                            break;
                        case "empty":
                            var commandFrameEmptyObj = new Base<CommandFrameEmptyMode>("command", new CommandFrameEmptyMode());
                            var commandFrameEmptyObjPayload = JsonSerializer.Serialize(commandFrameEmptyObj);
                            await mqttClient.PublishAsync("esp32", commandFrameEmptyObjPayload);
                            AnsiConsole.WriteLine("\"Command - FRAME\" mode empty frame data sent.");
                            break;
                    }
                    break;
            }
            break;
        case "stop":
            var stopObj = new Base<EmptyMode>("stop", new EmptyMode());
            var stopPayload = JsonSerializer.Serialize(stopObj);
            await mqttClient.PublishAsync("esp32", stopPayload);
            AnsiConsole.WriteLine("\"Stop\" mode data sent.");
            break;
    }
}

List<Rgb24> GetImageBytes(string path)
{
    var image = Image.Load<Rgb24>(path);
    return image.TryGetSinglePixelSpan(out var ds) ? ds.ToArray().ToList() : null;
}
