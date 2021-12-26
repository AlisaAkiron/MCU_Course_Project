using System.Text.Json.Serialization;

namespace ESPController.Models;

public record SingleDisplayMode(int X, int Y, int H, int S, int L) : IData
{
    [JsonPropertyName("x")]
    public int X { get; set; } = X;
    [JsonPropertyName("y")]
    public int Y { get; set; } = Y;
    [JsonPropertyName("h")]
    public int H { get; set; } = H;
    [JsonPropertyName("s")]
    public int S { get; set; } = S;
    [JsonPropertyName("l")]
    public int L { get; set; } = L;
}
