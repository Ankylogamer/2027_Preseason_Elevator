package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  static final DCMotor gearbox = DCMotor.getNEO(2);
  static final double Kp = 0.0;
  static final double Kd = 0.0;
  static final double Ks = 0.0;
  static final double Kv = 0.1;

  private ElevatorSim sim =
      new ElevatorSim(
          Kv, 0, gearbox, ElevatorSubsystem.minHeight, ElevatorSubsystem.maxHeight, false, 0, null);
  private DCMotorSim left;
  private DCMotorSim right;
  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  private boolean closedLoop = false;
  private PIDController leftPID = new PIDController(Kp, 0.0, Kd);
  private PIDController rightPID = new PIDController(Kp, 0.0, Kd);
  private double leftFFVolts = 0.0;
  private double rightFFVolts = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (closedLoop) {
      leftAppliedVolts = leftFFVolts + leftPID.calculate(left.getAngularVelocityRadPerSec());
      rightAppliedVolts = rightFFVolts + rightPID.calculate(right.getAngularVelocityRadPerSec());
    }

    // Update simulation state
    var elevatorVolts = leftAppliedVolts - rightAppliedVolts; // up and down

    sim.setInputVoltage(MathUtil.clamp(elevatorVolts, -12, 12));
    sim.update(0.02);

    inputs.leftPositionRad = left.getAngularPositionRad();
    inputs.leftVelocityRadPerSec = left.getAngularVelocityRadPerSec();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = left.getCurrentDrawAmps();

    inputs.rightPositionRad = right.getAngularPositionRad();
    inputs.rightVelocityRadPerSec = right.getAngularVelocityRadPerSec();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = right.getCurrentDrawAmps();

    double beltPosition =
        ElevatorSubsystem.GetBeltPosition(inputs.leftPositionRad - inputs.rightPositionRad);
    double beltVelocity = inputs.leftVelocityRadPerSec - inputs.rightVelocityRadPerSec;

    inputs.armPositionRad = ElevatorSubsystem.GetArmPosition(beltPosition);
    inputs.armVelocityRadPerSec = ElevatorSubsystem.GetArmVelocity(beltVelocity);

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
