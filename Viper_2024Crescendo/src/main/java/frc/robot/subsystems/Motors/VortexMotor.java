package frc.robot.subsystems.Motors;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DriverStation;

public class VortexMotor implements Motor {
  public CANSparkFlex motor;

  public RelativeEncoder encoder;

  public SparkPIDController pid;

  private boolean factoryDefaultOccurred = false;

  public VortexMotor(int motorID) {
    this.motor = new CANSparkFlex(motorID, MotorType.kBrushless);
    factoryDefault();
    clearStickyFaults();
    encoder = motor.getEncoder();
    pid = motor.getPIDController();
    pid.setFeedbackDevice(encoder);
  }

  private void configureSparkFlex(Supplier<REVLibError> config) {
    for (int i = 0; i < maximumRetries; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
    }
    DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
  }

  @Override
  public void setVoltageCompensation(double nominalVoltage) {
    configureSparkFlex(() -> motor.enableVoltageCompensation(nominalVoltage));
  }

  @Override
  public void setCurrentLimit(int currentLimit) {
    configureSparkFlex(() -> motor.setSmartCurrentLimit(currentLimit));
  }

  @Override
  public void setLoopRampRate(double rampRate) {
    configureSparkFlex(() -> motor.setOpenLoopRampRate(rampRate));
    configureSparkFlex(() -> motor.setClosedLoopRampRate(rampRate));
  }

  @Override
  public Object getMotor() {
    return motor;
  }

  @Override
  public void factoryDefault() {
    if (!factoryDefaultOccurred) {
      configureSparkFlex(motor::restoreFactoryDefaults);
      factoryDefaultOccurred = true;
    }
  }

  @Override
  public void clearStickyFaults() {
    configureSparkFlex(motor::clearFaults);
  }

  @Override
  public void setPositionFactor(double positionConversionFactor) {
    configureSparkFlex(() -> encoder.setPositionConversionFactor(positionConversionFactor));
    configureSparkFlex(() -> encoder.setVelocityConversionFactor(positionConversionFactor / 60));
    configureCANStatusFrames(10, 20, 20, 500, 500);
  }

  @Override
  public void setVelocityFactor(double factor) {
    configureSparkFlex(() -> encoder.setVelocityConversionFactor(factor));
    configureCANStatusFrames(10, 20, 20, 500, 500);
  }

  @Override
  public void configurePIDF(double P, double I, double D, double F, double Izone) {

    int pidSlot = 0;
    configureSparkFlex(() -> pid.setP(P, pidSlot));
    configureSparkFlex(() -> pid.setI(I, pidSlot));
    configureSparkFlex(() -> pid.setD(D, pidSlot));
    configureSparkFlex(() -> pid.setFF(F, pidSlot));
    configureSparkFlex(() -> pid.setIZone(Izone, pidSlot));
  }

  @Override
  public void configurePIDF(double P, double I, double D, double F) {
    configurePIDF(P, I, D, F, 0);
  }

  @Override
  public void configurePIDWrapping(double minInput, double maxInput) {
    configureSparkFlex(() -> pid.setPositionPIDWrappingEnabled(true));
    configureSparkFlex(() -> pid.setPositionPIDWrappingMinInput(minInput));
    configureSparkFlex(() -> pid.setPositionPIDWrappingMaxInput(maxInput));
  }

  /**
   * Set the CAN status frames.
   *
   * CANStatus0 Applied Output, Faults, Sticky Faults, Is Follower
   * CANStatus1 Motor Velocity, Motor Temperature, Motor Voltage, Motor
   * Current
   * CANStatus2 Motor Position
   * CANStatus3 Analog Sensor Voltage, Analog Sensor Velocity, Analog
   * Sensor Position
   * CANStatus4 Alternate Encoder Velocity, Alternate Encoder Position
   */
  public void configureCANStatusFrames(
      int CANStatus0, int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4) {
    configureSparkFlex(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, CANStatus0));
    configureSparkFlex(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, CANStatus1));
    configureSparkFlex(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, CANStatus2));
    configureSparkFlex(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, CANStatus3));
    configureSparkFlex(() -> motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, CANStatus4));
    // TODO: Configure Status Frame 5 and 6 if necessary
    // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
  }

  @Override
  public void setMotorBrake(boolean isBrakeMode) {
    configureSparkFlex(() -> motor.setIdleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast));
  }

  @Override
  public void setInverted(boolean inverted) {
    motor.setInverted(inverted);
  }

  @Override
  public void burnFlash() {
    try {
      Thread.sleep(200);
    } catch (Exception e) {
    }
    configureSparkFlex(() -> motor.burnFlash());
  }

  @Override
  public void set(double percentOutput) {
    motor.set(percentOutput);
  }

  @Override
  public void setPositionReference(double position) {
    configureSparkFlex(() -> pid.setReference(position, ControlType.kPosition, 0));
  }

  @Override
  public void setPositionReferenceArbFF(double setpoint, double feedforward) {
    configureSparkFlex(() -> pid.setReference(setpoint, ControlType.kPosition, 0, feedforward));
  }

  @Override
  public void setVelocityReference(double velocity, double feedforward) {
    configureSparkFlex(() -> pid.setReference(velocity, ControlType.kVelocity, 0, feedforward));
  }

  @Override
  public double getVoltage() {
    return motor.getAppliedOutput() * motor.getBusVoltage();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public void setPosition(double position) {
    configureSparkFlex(() -> encoder.setPosition(position));
  }

  @Override
  public void setInvertedEncoder(boolean inverted) {
    configureSparkFlex(() -> encoder.setInverted(inverted));
  }
}
