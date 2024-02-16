// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Java_Is_UnderControl.Limelight.LimelightHelpers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CommandsConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Joysticks.ControlBoard;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase implements ISwerve {

  private SwerveDrive swerveDrive;

  private ControlBoard controller = ControlBoard.getInstance();

  public boolean notUsingLLforOdometry;

  private static SwerveSubsystem mInstance = null;

  private double headingDegrees;

  private double headingRadians;

  private Rotation2d angleAlign = new Rotation2d(Units.degreesToRadians(0));

  PIDController pidTrackApril = new PIDController(CommandsConstants.P_TrackSpeaker, CommandsConstants.I_TrackSpeaker,
      CommandsConstants.D_TrackSpeaker);

  Pose2d poseVision;

  Field2d aprField2d = new Field2d();

  public static SwerveSubsystem getInstance(File directory) {
    if (mInstance == null) {
      mInstance = new SwerveSubsystem(directory);
    }

    return mInstance;
  }

  public static SwerveSubsystem getInstance() {
    if (mInstance == null) {
      DriverStation.reportError("YAGSL Configuration not provided", true);
    }
    return mInstance;
  }

  private SwerveSubsystem(File directory) {
    SmartDashboard.putData("POSE APRIL", aprField2d);
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    setupYAGSL_JSON(directory);
    setupGyroToAlliance();
    setupPathPlanner();
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
    headingDegrees = getHeading().getDegrees();
    headingRadians = getHeading().getRadians();
    SmartDashboard.putNumber("Heading Degrees", headingDegrees);
    SmartDashboard.putNumber("Heading Radians", headingRadians);
    SmartDashboard.putNumber("Heading Odometry", swerveDrive.getOdometryHeading().getDegrees());
    if (!notUsingLLforOdometry) {
      LimelightHelpers.setPipelineIndex("", 0);
      // poseVision = LimelightHelpers.getBotPose2d_wpiBlue("");
      // aprField2d.setRobotPose(poseVision);
    } else {
      LimelightHelpers.setPipelineIndex("", 1);
    }
  }

  @Override
  public void driveAlignAngleButton() {
    if (controller.notUsingJoystick()) {
      ChassisSpeeds speeds = getTargetSpeedsAngle(controller.getYtranslation(), controller.getXtranslation(),
          angleAlign.getDegrees());
      SmartDashboard.putString("Chassis Speed", speeds.toString());
      driveFO(speeds);
    } else {
      driveAlignAngleJoy();
    }
  }

  @Override
  public void driveAlignAngleJoy() {
    ChassisSpeeds speeds = getTargetSpeedsAngleJoystick(controller.getYtranslation(), controller.getXtranslation(),
        controller.getCOS_Joystick(), controller.getSIN_Joystick());
    SmartDashboard.putString("Chassis Speed", speeds.toString());
    driveFO(speeds);
  }

  @Override
  public void driveAlignSpeaker() {
  }

  @Override
  public void intakeNote() {

  }

  @Override
  public void lockSwerve() {

  }

  private void setupGyroToAlliance() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      swerveDrive.setGyro(new Rotation3d(0, 0, Units.degreesToRadians(180)));
    } else {
      swerveDrive.zeroGyro();
    }
  }

  private void setupYAGSL_JSON(File directory) {
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveConstants.MAX_VEL,
          SwerveConstants.SteeringConversionFactor,
          SwerveConstants.DriveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner() {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            AutoConstants.TranslationPID,
            // Translation PID constants
            AutoConstants.angleAutoPID,
            // Rotation PID constants
            4.5,
            // Max module speed, in m/s
            swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
            // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig()
        // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  private void driveFO(ChassisSpeeds speeds) {
    swerveDrive.driveFieldOriented(speeds);
  }

  private void driveOmegaPIDfo(ChassisSpeeds speeds, double setpoint, double measurement, PIDController pid) {
    speeds = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
        pid.calculate(measurement, setpoint));
    SmartDashboard.putNumber("Omega PID", speeds.omegaRadiansPerSecond);
    swerveDrive.driveFieldOriented(speeds);
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    SmartDashboard.putNumber("Pose inicial X", initialHolonomicPose.getX());
    SmartDashboard.putNumber("Pose inicial Y", initialHolonomicPose.getY());
    SmartDashboard.putNumber("Angulo Inicial", initialHolonomicPose.getRotation().getDegrees());
    swerveDrive.resetOdometry(initialHolonomicPose);

  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    chassisSpeeds = new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond);
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public void setHeadingCorrection(boolean active) {
    swerveDrive.setHeadingCorrection(active);
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  private Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  private ChassisSpeeds getTargetSpeedsAngle(double xInput, double yInput, double angle) {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      xInput = -xInput;
      yInput = -yInput;
    }
    double angleTarget = angle;
    ChassisSpeeds targetSpeeds;
    SmartDashboard.putNumber("Angle Target Speeds", angleTarget);
    double angleRadians = Units.degreesToRadians(angle);
    targetSpeeds = swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angleRadians,
        headingRadians, SwerveConstants.MAX_VEL);
    return targetSpeeds;
  }

  private ChassisSpeeds getTargetSpeedsAngleJoystick(double xInput, double yInput, double headingX, double headingY) {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      xInput = -xInput;
      yInput = -yInput;
      headingX = -headingX;
      headingY = -headingY;
    }
    double angleTarget = Math.atan2(headingX, headingY);
    ChassisSpeeds targetSpeeds;
    SmartDashboard.putNumber("Angle Target Speeds", Units.radiansToDegrees(angleTarget));
    targetSpeeds = swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY,
        headingRadians, SwerveConstants.MAX_VEL);
    targetSpeeds = discretize(targetSpeeds);
    return targetSpeeds;
  }

  // This is useful for compensating for translational skew when translating and
  // rotating a swerve drivetrain.
  public ChassisSpeeds discretize(ChassisSpeeds speeds) {
    double dt = 0.02;
    var desiredDeltaPose = new Pose2d(
        speeds.vxMetersPerSecond * dt,
        speeds.vyMetersPerSecond * dt,
        new Rotation2d(speeds.omegaRadiansPerSecond * dt * 4));
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  public void lock() {
    swerveDrive.lockPose();
  }

  public void addVisionReading() {

    // Pose2d poseVision = new Pose2d(3, 3, Rotation2d.fromDegrees(0));
    /*
     * poseVision = new Pose2d(
     * new Translation2d(
     * filter.calculate(poseVision.getX()),
     * filter.calculate(poseVision.getY())),
     * new Rotation2d(
     * filter.calculate(poseVision.getRotation().getDegrees())));
     */
    if (LimelightHelpers.getTV("")) {
      swerveDrive.addVisionMeasurement(poseVision, Timer.getFPGATimestamp());
    }
  }

  public double trackSpeaker() {
    LimelightHelpers.setPipelineIndex("", 1);
    return pidTrackApril.calculate(LimelightHelpers.getTX(""), 0);
  }

  /*
   * ANGLE CONTROL
   */

  public void setAngle90() {
    angleAlign = new Rotation2d(Units.degreesToRadians(90));
  }

  public void setAngle180() {
    angleAlign = new Rotation2d(Units.degreesToRadians(180));
  }

  public void setAngleNeg90() {
    angleAlign = new Rotation2d(Units.degreesToRadians(-90));
  }

  public void setAngle0() {
    angleAlign = new Rotation2d(Units.degreesToRadians(0));
  }

  public void setAngle45() {
    angleAlign = new Rotation2d(Units.degreesToRadians(45));
  }
}
