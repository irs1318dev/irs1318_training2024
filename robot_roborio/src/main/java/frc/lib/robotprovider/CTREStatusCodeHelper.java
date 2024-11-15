package frc.lib.robotprovider;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusCode;

public class CTREStatusCodeHelper
{
    public static void printError(StatusCode sc, String id, String operation)
    {
        if (sc != StatusCode.OK)
        {
            System.err.println(String.format("%s: %s failed with %s", id, operation, sc.toString()));
        }
    }

    public static void printError(ErrorCode sc, String id, String operation)
    {
        if (sc != ErrorCode.OK)
        {
            System.err.println(String.format("%s: %s failed with %s", id, operation, sc.toString()));
        }
    }
}
