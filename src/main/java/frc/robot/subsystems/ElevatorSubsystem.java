package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class ElevatorSubsystem extends SubsystemBase {
    public static final double minHeight = Units.inchesToMeters(0.0);
    public static final double maxHeight = Units.inchesToMeters(50.0);
    public static final double totalHeight = maxHeight - minHeight;
    public static final double centerWidth = Units.inchesToMeters(5.0);
    public static final double beltSegmentMin = centerWidth;
    public static final double beltSegmentMax = Math.sqrt(maxHeight * maxHeight + centerWidth * centerWidth);
    public static final double beltLength = 2 * totalHeight + 2 * beltSegmentMin + 2 * beltSegmentMax;

    public static final double wheelRadius = Units.inchesToMeters(2.0);
    public static final double armRadius = Units.inchesToMeters(5.0);
    public static final double motorReduction = 5; // 5:1 means motor is running 5x faster that output
    public static final double armReduction = 120;

    private final double kS = Constants.currentMode == Mode.SIM ? ElevatorIOSim.Ks : ElevatorIOSpark.Ks;
    private final double kV = Constants.currentMode == Mode.SIM ? ElevatorIOSim.Kv : ElevatorIOSpark.Kv;

    LoggedMechanism2d mech = new LoggedMechanism2d(1.2, 2);
    LoggedMechanismRoot2d root = mech.getRoot("climber", 0.6, 0);
    LoggedMechanismLigament2d m_elevator = root.append(new LoggedMechanismLigament2d("elevator", minHeight, 90));
    LoggedMechanismLigament2d m_wrist = m_elevator.append(new LoggedMechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));


    public static double GetBeltPosition(double angularPosition) {
        return (angularPosition * wheelRadius);
    }

    public static double GetHeight(double beltFeed) {
        beltFeed += centerWidth; // add the initial feed from left
        beltFeed += centerWidth; // add the inital feed from right
        double height = Math.sqrt((beltFeed * beltFeed) / 4 - (centerWidth * centerWidth));
        if (!Double.isNaN(height)) { // not too low
            if (height > maxHeight) {
                return Double.NaN;
            }
        }
        return height;
    }

    public static double GetArmPosition(double linearPosition) {
        return (linearPosition / armReduction) / armRadius;
    }

    public static double GetArmVelocity(double linearSpeed) {
        return (linearSpeed / armRadius) / armReduction;
    }

    final ElevatorIO io;
    final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        m_elevator.setLength(inputs.elevatorPosition);
        m_wrist.setAngle(getArmDegrees());
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("ArmMech", mech);
    }


    @AutoLogOutput
    public double getLeftPositionMeters() {
        return inputs.leftPositionRad * wheelRadius;
    }

    @AutoLogOutput
    public double getRightPositionMeters() {
        return inputs.rightPositionRad * wheelRadius;
    }

    public void runOpenLoop(double leftVolts, double rightVolts) {
        io.setVoltage(leftVolts, rightVolts);
    }

    public void runClosedLoop(double leftMetersPerSec, double rightMetersPerSec) {
       //double leftRadPerSec = leftMetersPerSec / wheelRadius;
        //double rightRadPerSec = rightMetersPerSec / wheelRadius;
        double leftRadPerSec = leftMetersPerSec * (motorReduction / (2*Math.PI*wheelRadius));
        double rightRadPerSec = rightMetersPerSec * (motorReduction / (2*Math.PI*wheelRadius));
        /*TODO:Distance per Second multiplied by rotations per distance traveled should get you rotations per second ((D/T)*(R/D) = R/T)
        and if the distance traveled is equal to the rotations times the circumfrence dvided by the motor reduction (D=Rot*(2πr)/(MoRe)) 
        then if you replaced that distance traveled for the distanced travel in rotations per distance traveled then the Rotations would
        cancel out and you would be left with distance per second times motor reduction divided by the circumference set equal to 
        rotations per second((D/T)*((MoRe)/(2πr)=(R/T))*/

        Logger.recordOutput("Elevator/LeftSetpointRadPerSec", leftRadPerSec);
        Logger.recordOutput("Elevator/RightSetpointRadPerSec", rightRadPerSec);
        
        double leftFFVolts = kS * Math.signum(leftRadPerSec) + kV * leftRadPerSec;
        double rightFFVolts = kS * Math.signum(rightRadPerSec) + kV * rightRadPerSec;

        io.setVelocity(leftRadPerSec, rightRadPerSec, leftFFVolts, rightFFVolts);
    }

    /** Stops the drive. */
    public void stop() {
        runOpenLoop(0.0, 0.0);
    }

    double GetHeight() {
        return GetHeight(GetBeltPosition(inputs.leftPositionRad) + GetBeltPosition(inputs.rightPositionRad));
    }

    double getArmDegrees() {
        return  Units.radiansToDegrees(inputs.armPositionRad);
    }

    // Test functions

    void lower() {
        io.setVoltage(-12, -12);
        periodic();
    }
    void raise() {
        io.setVoltage(12, 12);
        periodic();
    }
    void rotate(double leftVolts, double rightVolts) {
        io.setVoltage(leftVolts, rightVolts);
        periodic();
    }
}   
