using System.Text.Json.Serialization;

namespace ESPController.Models;

public record Base<T>(string Mode, T Data) where T : IData
{
    [JsonPropertyName("mode")]
    public string Mode { get; set; } = Mode;

    [JsonPropertyName("data")]
    public T Data { get; set; } = Data;
}
