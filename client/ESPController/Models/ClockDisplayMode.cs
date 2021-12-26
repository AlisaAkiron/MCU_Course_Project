using System.Text.Json.Serialization;

namespace ESPController.Models;

public record ClockDisplayMode(int Timezone) : IData
{
    [JsonPropertyName("tz")]
    public int Timezone { get; set; } = Timezone;
}
