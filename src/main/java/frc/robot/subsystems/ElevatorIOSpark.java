package frc.robot.subsystems;

import static frc.robot.subsystems.SparkUtil.ifOk;
import static frc.robot.subsystems.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;

public class ElevatorIOSpark implements ElevatorIO {
  static final int leftCanId = 9;
  static final int rightCanId = 9;

  static final int currentLimit = 60;

  static final boolean leftInverted = false;
  static final boolean rightInverted = true;

  static final double Kp = 0.0;
  static final double Kd = 0.0;
  static final double Ks = 0.0;
  static final double Kv = 0.1;

  class TempEncoder implements RelativeEncoder {
    public double position;
    public double velocity;

    @Override
    public double getPosition() {
      return position;
    }

    @Override
    public double getVelocity() {
      return velocity;
    }

    @Override
    public REVLibError setPosition(double position) {
      this.position = position;
      return REVLibError.kOk;
    }
  }

  private final SparkMax left = new SparkMax(leftCanId, MotorType.kBrushless);
  private final SparkMax right = new SparkMax(rightCanId, MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = left.getEncoder();
  private final RelativeEncoder rightEncoder = right.getEncoder();
  private final RelativeEncoder armEncoder = new TempEncoder();
  private final SparkClosedLoopController leftController = left.getClosedLoopController();
  private final SparkClosedLoopController rightController = right.getClosedLoopController();

  public ElevatorIOSpark() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    config.closedLoop.pid(Kp, 0.0, Kd);
    config
        .encoder
        .positionConversionFactor(
            2 * Math.PI / ElevatorSubsystem.motorReduction) // Rotor Rotations -> Wheel Radians
        .velocityConversionFactor(
            (2 * Math.PI) / 60.0 / ElevatorSubsystem.motorReduction) // Rotor RPM -> Wheel Rad/Sec
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    config.inverted(leftInverted);
    tryUntilOk(
        left,
        5,
        () ->
            left.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    config.inverted(rightInverted);
    tryUntilOk(
        right,
        5,
        () ->
            right.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    ifOk(left, leftEncoder::getPosition, (value) -> inputs.leftPositionRad = value);
    ifOk(left, leftEncoder::getVelocity, (value) -> inputs.leftVelocityRadPerSec = value);
    ifOk(
        left,
        new DoubleSupplier[] {left::getAppliedOutput, left::getBusVoltage},
        (values) -> inputs.leftAppliedVolts = values[0] * values[1]);
    ifOk(left, left::getOutputCurrent, (values) -> inputs.rightCurrentAmps = values);

    inputs.armPositionRad = armEncoder.getPosition();
    inputs.armVelocityRadPerSec = armEncoder.getVelocity();

    ifOk(right, rightEncoder::getPosition, (value) -> inputs.rightPositionRad = value);
    ifOk(right, rightEncoder::getVelocity, (value) -> inputs.rightVelocityRadPerSec = value);
    ifOk(
        right,
        new DoubleSupplier[] {right::getAppliedOutput, right::getBusVoltage},
        (values) -> inputs.rightAppliedVolts = values[0] * values[1]);
    ifOk(right, right::getOutputCurrent, (values) -> inputs.rightCurrentAmps = values);
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
  }

  @Override
  public void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    leftController.setSetpoint(
        leftRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, leftFFVolts);
    rightController.setSetpoint(
        rightRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, rightFFVolts);
  }
}
