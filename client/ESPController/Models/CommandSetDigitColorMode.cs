using System.Text.Json.Serialization;

namespace ESPController.Models;

public record CommandSetDigitColorMode(string Type, string Parameter, string ColorType, int ColorParam1, int ColorParam2, int ColorParam3) : IData
{
    [JsonPropertyName("type")] public string Type { get; set; } = Type;
    [JsonPropertyName("param")] public string Parameter { get; set; } = Parameter;
    [JsonPropertyName("color_type")] public string ColorType { get; set; } = ColorType;
    [JsonPropertyName("c1")] public int ColorParam1 { get; set; } = ColorParam1;
    [JsonPropertyName("c2")] public int ColorParam2 { get; set; } = ColorParam2;
    [JsonPropertyName("c3")] public int ColorParam3 { get; set; } = ColorParam3;
}
