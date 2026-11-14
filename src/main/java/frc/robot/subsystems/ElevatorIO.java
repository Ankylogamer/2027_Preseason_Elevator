package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double leftPositionRad = 0.0;
        public double leftVelocityRadPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;
        
        public double elevatorPosition = 0.0; // linear m
        public double elevatorVelocity = 0.0; // linear m/s
        
        public double armPositionRad = 0.0;
        public double armVelocityRadPerSec = 0.0;
        
        public double rightPositionRad = 0.0;
        public double rightVelocityRadPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {}

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double leftVolts, double rightVolts) {}

    /** Run closed loop at the specified velocity. */
    public default void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {}
}
