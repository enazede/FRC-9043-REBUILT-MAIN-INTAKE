package frc.robot;

import frc.robot.constants.GyroConstants;

public class GetYaw {
    public static double getHeading() {
        return Math.IEEEremainder(-GyroConstants.gyro.getAngle(), 360);
    }
}
