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

    ElevatorSim elevator =
        new ElevatorSim(
            Kv, 1, gearbox, ElevatorSubsystem.minHeight, ElevatorSubsystem.maxHeight,
            false, 0);

    DCMotorSim left;
    DCMotorSim right;

    double leftAppliedVolts = 0.0;
    double rightAppliedVolts = 0.0;
    boolean closedLoop = false;
    PIDController leftPID = new PIDController(Kp, 0.0, Kd);
    PIDController rightPID = new PIDController(Kp, 0.0, Kd);
    double leftFFVolts = 0.0;
    double rightFFVolts = 0.0;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (closedLoop) {
            leftAppliedVolts = leftFFVolts + leftPID.calculate(left.getAngularVelocityRadPerSec());
            rightAppliedVolts = rightFFVolts + rightPID.calculate(right.getAngularVelocityRadPerSec());
        }
      
        var common = Math.min(Math.abs(leftAppliedVolts), Math.abs(rightAppliedVolts));
        var feed = Math.signum(leftAppliedVolts) + Math.signum(rightAppliedVolts) * common / 2;
      
        // Update simulation state
        var elevatorVolts = feed; // leftAppliedVolts - rightAppliedVolts; // up and down
      
        elevator.setInputVoltage(MathUtil.clamp(elevatorVolts, -12, 12));
        elevator.update(0.02);
      
        left.setInputVoltage(leftAppliedVolts);
        right.setInputVoltage(leftAppliedVolts);
        left.update(0.02);
        right.update(0.02);
      
        inputs.leftPositionRad = left.getAngularPositionRad();
        inputs.leftVelocityRadPerSec = left.getAngularVelocityRadPerSec();
        inputs.leftAppliedVolts = leftAppliedVolts;
        inputs.leftCurrentAmps = left.getCurrentDrawAmps();
      
        inputs.rightPositionRad = right.getAngularPositionRad();
        inputs.rightVelocityRadPerSec = right.getAngularVelocityRadPerSec();
        inputs.rightAppliedVolts = rightAppliedVolts;
        inputs.rightCurrentAmps = right.getCurrentDrawAmps();
      
        // how much belt fed into the elevator
        double beltPositionLeft = ElevatorSubsystem.GetBeltPosition(inputs.leftPositionRad);
        double beltPositionRight = ElevatorSubsystem.GetBeltPosition(inputs.rightPositionRad);
        double beltHeight = ElevatorSubsystem.GetHeight(beltPositionLeft + beltPositionRight);
      
        double beltPosition = ElevatorSubsystem.GetBeltPosition(inputs.rightPositionRad - inputs.leftPositionRad);
        double beltVelocity = inputs.rightVelocityRadPerSec - inputs.leftVelocityRadPerSec;
      
        inputs.armPositionRad = ElevatorSubsystem.GetArmPosition(beltPosition);
        inputs.armVelocityRadPerSec = ElevatorSubsystem.GetArmVelocity(beltVelocity);
      
        inputs.elevatorPosition = beltHeight;
        inputs.elevatorVelocity = 0;
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        closedLoop = false;
        leftAppliedVolts = leftVolts;
        rightAppliedVolts = rightVolts;
    }

    @Override
    public void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
        closedLoop = true;
        this.leftFFVolts = leftFFVolts;
        this.rightFFVolts = rightFFVolts;
        leftPID.setSetpoint(leftRadPerSec);
        rightPID.setSetpoint(rightRadPerSec);
    }
}
