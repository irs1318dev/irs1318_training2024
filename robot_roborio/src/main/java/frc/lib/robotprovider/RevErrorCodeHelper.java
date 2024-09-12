package frc.lib.robotprovider;

import com.revrobotics.REVLibError;;

public class RevErrorCodeHelper
{
    public static void printError(REVLibError ec, String operation)
    {
        if (ec != REVLibError.kOk)
        {
            System.err.println(operation + " failed with " + ec.toString());
        }
    }
}
