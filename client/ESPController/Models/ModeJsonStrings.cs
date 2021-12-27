using System.Text.Json;
using ESPController.Models.General;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;

namespace ESPController.Models;

public static class ModeJsonStrings
{
    private const string SingleMode = "{\"type\":\"mode\",\"data\":{\"mode\":\"single\",\"value\":{\"x\":{0},\"y\":{1},\"r\":{2},\"g\":{3},\"b\":{4}}}}";
    private const string ClockMode = "{\"type\":\"mode\",\"data\":{\"mode\":\"clock\",\"value\":{\"tz\":{0}}}}";
    private const string PictureMode = "{\"type\":\"mode\",\"data\":{\"mode\":\"picture\",\"value\":{\"part\":{0},\"frame\":{1}}}}";
    private const string StopMode = "{\"type\":\"mode\",\"data\":{\"mode\":\"stop\"}}";

    public static string GetModeJsonString(Modes mode, string[] param)
    {
        switch (mode)
        {
            case Modes.Single:
                if (param.Length != 5)
                {
                    throw new ArgumentException("Invalid number of parameters");
                }
                var str = SingleMode;
                for (var i = 0; i < 5; i++)
                {
                    str = str.Replace("{" + i + "}", param[i]);
                }
                return str;
            case Modes.Clock:
                if (param.Length != 1)
                {
                    throw new ArgumentException("Invalid number of parameters");
                }
                return ClockMode.Replace("{0}", param[0]);
            case Modes.Picture:
                if (param.Length != 1)
                {
                    throw new ArgumentException("Invalid number of parameters");
                }
                var fi = new FileInfo(param[0]);
                if (fi.Exists is false)
                {
                    throw new FileNotFoundException("File not found");
                }
                var imageColorMap = GetImageBytes(fi.FullName);
                var splitMap = new List<List<ColorMap>>();
                for (var i = 1; i <= 8; i++)
                {
                    splitMap.Add(imageColorMap.Skip((i - 1) * 32).Take(32).ToList());
                }
                var serializedStrings = splitMap
                    .Select(x => JsonSerializer.Serialize(x)).ToList();
                var payloads = new List<string>();
                for (var i = 1; i <= 8; i++)
                {
                    payloads.Add(PictureMode
                        .Replace("{0}", i.ToString())
                        .Replace("{1}", serializedStrings[i - 1]));
                }
                var payload = payloads.Aggregate((x, y) => x + "||" + y);
                return payload;
            case Modes.Stop:
                return StopMode;
            default:
                throw new ArgumentOutOfRangeException(nameof(mode), mode, null);
        }
    }

    private static List<ColorMap> GetImageBytes(string path)
    {
        var image = Image.Load<Rgb24>(path);
        if (image.TryGetSinglePixelSpan(out var ds) is false)
        {
            return null;
        }
        var rgbGroup = ds.ToArray();
        var currentX = 0;
        var currentY = 0;
        var pixels = new List<ColorMap>();
        foreach (var rgb in rgbGroup)
        {
            pixels.Add(new ColorMap()
            {
                R = rgb.R,
                G = rgb.G,
                B = rgb.B,
                X = currentX,
                Y = currentY
            });
            currentX += 1;
            if (currentX != 32)
            {
                continue;
            }
            currentX = 0;
            currentY++;
        }
        return pixels;
    }
}

