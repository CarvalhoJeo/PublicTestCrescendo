package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.math.SwerveMath;

public class SwerveConstants {
  public static final double MAX_VEL = Units.feetToMeters(14.5);

  public static final double DriveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4),
      6.12, 1);
  public static final double SteeringConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8, 1);

  public static final double ROBOT_MASS = 52;
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, 0.173), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final boolean ACELL_CORREC = true;
  public static final double WHEEL_LOCK_TIME = 10;
}
