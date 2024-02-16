package frc.robot.Joysticks;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;

public class DriverController implements IDriverController {

  private static DriverController mInstance = null;

  public static DriverController getInstance() {
    if (mInstance == null) {
      mInstance = new DriverController();
    }

    return mInstance;
  }

  final CommandXboxController driverController;

  private DriverController() {
    driverController = new CommandXboxController(0);
  }

  @Override
  public double getXtranslation() {
    return -MathUtil.applyDeadband(driverController.getLeftX(), JoystickConstants.DEADBAND);
  }

  @Override
  public double getYtranslation() {
    return -MathUtil.applyDeadband(driverController.getLeftY(), JoystickConstants.DEADBAND);
  }

  @Override
  public double getCOS_Joystick() {
    return -driverController.getRightX();
  }

  @Override
  public double getSIN_Joystick() {
    return -driverController.getRightY();
  }

  @Override
  public double getThrottle() {
    return Math.pow(driverController.getRightTriggerAxis(), 3);
  }

  @Override
  public boolean turboActivate() {
    return 0.2 < driverController.getLeftTriggerAxis();
  }

  @Override
  public boolean notUsingJoystick() {
    return frc.Java_Is_UnderControl.Util.Util.inRange(getCOS_Joystick(),
        -JoystickConstants.DEADBAND,
        0.2)
        && frc.Java_Is_UnderControl.Util.Util.inRange(getSIN_Joystick(), -JoystickConstants.DEADBAND,
            JoystickConstants.DEADBAND);
  }

  @Override
  public Trigger setHeadingBack() {
    return driverController.a();
  }

  @Override
  public Trigger setHeadingFront() {
    return driverController.y();
  }

  @Override
  public Trigger setHeadingLeft() {
    return driverController.x();
  }

  @Override
  public Trigger setHeadingRight() {
    return driverController.b();
  }

}
