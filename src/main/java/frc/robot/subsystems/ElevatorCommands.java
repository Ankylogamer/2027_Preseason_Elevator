package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ElevatorCommands {
    static final double DEADBAND = 0.1;
    static final double maxSpeed = edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond(5676);
    static final double maxSpeedVertical = 0.001;
    static final double maxSpeedRotate = 5.0;


    public static Command ElevCommand(ElevatorSubsystem elevator, DoubleSupplier elevatorSupplier, DoubleSupplier armSupplier) {
        return Commands.run(() -> {
                // Apply deadband
                double riseCommand = MathUtil.applyDeadband(elevatorSupplier.getAsDouble(), DEADBAND);
                double rotateCommand = MathUtil.applyDeadband(armSupplier.getAsDouble(), DEADBAND);

                // Calculate speeds - arcade drive is like up-down + rotate
                WheelSpeeds speeds = getWheelSpeeds(riseCommand, rotateCommand, true);

                // Apply output
                elevator.runClosedLoop(speeds.left, speeds.right);
            }, elevator);
    }

    static class WheelSpeeds {
        /** Left wheel speed. */
        public double left;
    
        /** Right wheel speed. */
        public double right;
    
        /** Constructs a WheelSpeeds with zeroes for left and right speeds. */
        public WheelSpeeds() {}
    
        /**
         * Constructs a WheelSpeeds.
         *
         * @param left The left speed [-1.0..1.0].
         * @param right The right speed [-1.0..1.0].
         */
        public WheelSpeeds(double left, double right) {
            this.left = left;
            this.right = right;
        }
    }
    
    static WheelSpeeds getWheelSpeeds(double rise, double rotate, boolean squareInputs) {
        rise = MathUtil.clamp(rise, -1.0, 1.0);
        rotate = MathUtil.clamp(rotate, -1.0, 1.0);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (squareInputs) {
          rise = MathUtil.copyDirectionPow(rise, 2);
          rotate = MathUtil.copyDirectionPow(rotate, 2);
        }

        double xSpeed = rise * maxSpeedVertical;
        double zRotation = rotate * maxSpeedRotate;

        double leftSpeed = xSpeed - zRotation;
        double rightSpeed = xSpeed + zRotation;

        // Find the maximum possible value of (throttle + turn) along the vector
        // that the joystick is pointing, then desaturate the wheel speeds
        double greaterInput = Math.max(Math.abs(xSpeed), Math.abs(zRotation));
        double lesserInput = Math.min(Math.abs(xSpeed), Math.abs(zRotation));
        if (greaterInput == 0.0) {
          return new WheelSpeeds(0.0, 0.0);
        }
        double saturatedInput = (greaterInput + lesserInput) / maxSpeed;
        leftSpeed /= saturatedInput;
        rightSpeed /= saturatedInput;

        return new WheelSpeeds(leftSpeed, rightSpeed);
    }

}
