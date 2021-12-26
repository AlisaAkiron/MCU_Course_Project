using System.Text.Json.Serialization;

namespace ESPController.Models;

public record CommandFrameAddMode(List<RgbColor> Frame, string Part): IData
{
    [JsonPropertyName("type")] public string Type { get; set; } = "frame";
    [JsonPropertyName("oper")] public string Operation { get; set; } = "add";
    [JsonPropertyName("part")] public string Part { get; set; } = Part;
    [JsonPropertyName("frame")] public List<RgbColor> Frame { get; set; } = Frame;
}
