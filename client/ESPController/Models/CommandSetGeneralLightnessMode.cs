using System.Text.Json.Serialization;

namespace ESPController.Models;

public record CommandSetGeneralLightnessMode(string Type, string Parameter, int Value) : IData
{
    [JsonPropertyName("type")] public string Type { get; set; } = Type;
    [JsonPropertyName("param")] public string Parameter { get; set; } = Parameter;
    [JsonPropertyName("value")] public int Value { get; set; } = Value;
}
