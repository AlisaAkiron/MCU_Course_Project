using System.Text.Json.Serialization;

namespace ESPController.Models;

public record CommandFrameEmptyMode : IData
{
    [JsonPropertyName("type")] public string Type { get; set; } = "frame";
    [JsonPropertyName("oper")] public string Operation { get; set; } = "empty";
}
