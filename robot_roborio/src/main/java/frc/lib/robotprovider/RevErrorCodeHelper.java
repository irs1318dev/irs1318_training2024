package frc.lib.robotprovider;

import com.revrobotics.REVLibError;;

public class RevErrorCodeHelper
{
    public static void printError(REVLibError ec, String id, String operation)
    {
        if (ec != REVLibError.kOk)
        {
            System.err.println(String.format("%s: %s failed with %s", id, operation, ec.toString()));
        }
    }
}
