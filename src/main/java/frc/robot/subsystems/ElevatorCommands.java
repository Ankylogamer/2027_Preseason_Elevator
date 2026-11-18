package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ElevatorCommands {
    private static final double DEADBAND = 0.1;
    private static final double maxSpeedMetersPerSec = 1.0;


    public static Command ElevCommand(ElevatorSubsystem elevator, DoubleSupplier elevatorSupplier, DoubleSupplier armSupplier) {
        return Commands.run(
        () -> {
          // Apply deadband
          double riseCommand = MathUtil.applyDeadband(elevatorSupplier.getAsDouble(), DEADBAND);
          double rotateCommand = MathUtil.applyDeadband(armSupplier.getAsDouble(), DEADBAND);

          // Calculate speeds - arcade drive is like up-down + rotate
          DifferentialDrive.WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(riseCommand, rotateCommand, true);

          // Apply output
          elevator.runClosedLoop(speeds.left * maxSpeedMetersPerSec, speeds.right * maxSpeedMetersPerSec);
        },
        elevator);
    }
}
