package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  class Settings {
    public static final double Kp = 0.0;
    public static final double Kd = 0.0;
    public static final double Ks = 0.0;
    public static final double Kv = 0.1;
  }

  private ElevatorSim sim = new ElevatorSim(0, 0, null, 0, 0, false, 0, null);
  private DCMotorSim left;
  private DCMotorSim right;
  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  private boolean closedLoop = false;
  private PIDController leftPID = new PIDController(Settings.Kp, 0.0, Settings.Kd);
  private PIDController rightPID = new PIDController(Settings.Kp, 0.0, Settings.Kd);
  private double leftFFVolts = 0.0;
  private double rightFFVolts = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (closedLoop) {
      leftAppliedVolts = leftFFVolts + leftPID.calculate(left.getAngularVelocityRadPerSec());
      rightAppliedVolts = rightFFVolts + rightPID.calculate(right.getAngularVelocityRadPerSec());
    }

    // Update simulation state
    var armVolts = leftAppliedVolts - rightAppliedVolts;
    sim.setInputVoltage(leftAppliedVolts);
    // sim.setInputs(
    //     MathUtil.clamp(leftAppliedVolts, -12.0, 12.0),
    //     MathUtil.clamp(rightAppliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.leftPositionRad = left.getAngularPositionRad();
    inputs.leftVelocityRadPerSec = left.getAngularVelocityRadPerSec();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = left.getCurrentDrawAmps();

    inputs.rightPositionRad = right.getAngularPositionRad();
    inputs.rightVelocityRadPerSec = right.getAngularVelocityRadPerSec();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = right.getCurrentDrawAmps();

    inputs.armPositionRad = 0;
    inputs.armVelocityRadPerSec = 0;

    inputs.elevatorPosition = 0;
    inputs.elevatorVelocity = 0;
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    closedLoop = false;
    leftAppliedVolts = leftVolts;
    rightAppliedVolts = rightVolts;
  }

  @Override
  public void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    closedLoop = true;
    this.leftFFVolts = leftFFVolts;
    this.rightFFVolts = rightFFVolts;
    leftPID.setSetpoint(leftRadPerSec);
    rightPID.setSetpoint(rightRadPerSec);
  }
}
