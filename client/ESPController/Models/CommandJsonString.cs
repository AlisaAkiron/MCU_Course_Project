namespace ESPController.Models;

public static class CommandJsonString
{
    private const string SetGeneralLightness =
        "{\"type\":\"command\",\"data\":{\"command\":\"set_general_lightness\",\"param\":{\"lightness\":{0}}}}";
    private const string SetDigitColor = "{\"type\":\"command\",\"data\":{\"command\":\"set_digit_color\",\"param\":{\"r\":{0},\"g\":{1},\"b\":{2}}}}";
    private const string AddFrame =
        "{\"type\":\"command\",\"data\":{\"command\":\"add_frame\",\"param\":{\"part\":{0},\"frame\":{1}}}}";
    public static string GetCommandJsonString(Commands commands, string[] param)
    {
        switch (commands)
        {
            case Commands.SetGeneralLightness:
                if (param.Length != 1)
                {
                    throw new ArgumentException("Invalid number of parameters");
                }
                return SetGeneralLightness.Replace("{0}", param[0]);
            case Commands.SetDigitColor:
                if (param.Length != 3)
                {
                    throw new ArgumentException("Invalid number of parameters");
                }
                var str = SetDigitColor;
                for (var i = 0; i < 3; i++)
                {
                    str = str.Replace("{" + i + "}", param[i]);
                }
                return str;
            default:
                throw new ArgumentOutOfRangeException(nameof(commands), commands, null);
        }
    }
}
