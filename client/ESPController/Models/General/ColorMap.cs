using System.Text.Json.Serialization;
using SixLabors.ImageSharp.PixelFormats;

namespace ESPController.Models.General;

public class ColorMap
{
    [JsonPropertyName("r")]
    public int R { get; set; }
    [JsonPropertyName("g")]
    public int G { get; set; }
    [JsonPropertyName("b")]
    public int B { get; set; }
    [JsonPropertyName("x")]
    public int X { get; set; }
    [JsonPropertyName("y")]
    public int Y { get; set; }
}
