package frc.lib.robotprovider;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusCode;

public class CTREStatusCodeHelper
{
    public static void printError(StatusCode sc, String operation)
    {
        if (sc != StatusCode.OK)
        {
            System.err.println(operation + " failed with " + sc.toString());
        }
    }

    public static void printError(ErrorCode sc, String operation)
    {
        if (sc != ErrorCode.OK)
        {
            System.err.println(operation + " failed with " + sc.toString());
        }
    }
}
