package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

public class ElevatorSubsystem extends SubsystemBase {
  public static final double minHeight = Units.inchesToMeters(0.0);
  public static final double maxHeight = Units.inchesToMeters(30.0);
  public static final double totalHeight = maxHeight - minHeight;
  public static final double centerWidth = Units.inchesToMeters(5.0);
  public static final double beltSegmentMin = centerWidth;
  public static final double beltSegmentMax =
      Math.sqrt(maxHeight * maxHeight + centerWidth * centerWidth);
  public static final double beltLength = 2 * totalHeight + 2 * beltSegmentMin + 2 * beltSegmentMax;

  public static final double wheelRadius = Units.inchesToMeters(2.0);
  public static final double armRadius = Units.inchesToMeters(5.0);
  public static final double motorReduction = 5;
  public static final double armReduction = 120;

  public static double GetBeltPosition(double position) {
    return (position * wheelRadius / armReduction);
  }
  ;

  public static double GetArmPosition(double linearPosition) {
    return (linearPosition / armReduction) / armRadius;
  }
  ;

  public static double GetArmVelocity(double linearSpeed) {
    return (linearSpeed / armRadius) / armReduction;
  }
  ;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  LoggedMechanism2d mechanism = new LoggedMechanism2d(3, 3);

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
    // mechanism.
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("MyMechanism", mechanism);
  }

  @AutoLogOutput
  public double getLeftPositionMeters() {
    return inputs.leftPositionRad * wheelRadius;
  }
}
