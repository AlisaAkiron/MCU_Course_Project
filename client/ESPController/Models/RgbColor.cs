using System.Text.Json.Serialization;

namespace ESPController.Models;

public record RgbColor(int Red, int Green, int Blue)
{
    [JsonPropertyName("r")] public int Red { get; set; } = Red;
    [JsonPropertyName("g")] public int Green { get; set; } = Green;
    [JsonPropertyName("b")] public int Blue { get; set; } = Blue;
}
