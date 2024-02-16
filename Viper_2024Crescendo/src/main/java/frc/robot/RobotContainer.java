// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Joysticks.ControlBoard;
import frc.robot.commands.Swerve.SwerveTeleopControl;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SwerveSubsystem swerve;

  private final SendableChooser<Command> autoChooser;

  XboxController driverXbox = new XboxController(0);
  public ControlBoard mControlBoard = ControlBoard.getInstance();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.swerve = SwerveSubsystem.getInstance(new File(Filesystem.getDeployDirectory(),
        "swerve/swerve"));
    NamedCommands.registerCommand("Shoot", new PrintCommand("SHOOT"));
    NamedCommands.registerCommand("Intake", new PrintCommand("INTAKING"));

    autoChooser = AutoBuilder.buildAutoChooser("New Auto");
    SmartDashboard.putData("Aut Chooser", autoChooser);

    configureBindings();
    swerve.setDefaultCommand(new SwerveTeleopControl(swerve));

  }

  private void configureBindings() {
    mControlBoard.setHeadingFront().onTrue(new InstantCommand(swerve::setAngle0));
    mControlBoard.setHeadingBack().onTrue(new InstantCommand(swerve::setAngle180));
    mControlBoard.setHeadingLeft().onTrue(new InstantCommand(swerve::setAngle90));
    mControlBoard.setHeadingRight().onTrue(new InstantCommand(swerve::setAngleNeg90));
    new JoystickButton(driverXbox, 5).onTrue(new InstantCommand(swerve::setAngle45));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  public void setHeadingCorrection(boolean active) {
    swerve.setHeadingCorrection(active);
  }

}
