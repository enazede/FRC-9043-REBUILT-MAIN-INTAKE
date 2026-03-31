package frc.robot.constants;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class GyroConstants {
    public static AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
}
