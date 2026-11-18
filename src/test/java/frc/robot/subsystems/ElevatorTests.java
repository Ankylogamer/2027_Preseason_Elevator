package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.HAL;

public class ElevatorTests {

    @Test
    public void SubsystemStartTest() {
        HAL.initialize(500, 0);
        ElevatorIOSim io = new ElevatorIOSim();
        ElevatorSubsystem subsystem = new ElevatorSubsystem(io);

        double eh = 0; // height of ElevatorSim
        var lpm = subsystem.getLeftPositionMeters();
        var rpm = subsystem.getRightPositionMeters();
        assertEquals(lpm, rpm);
        //assertEquals(lpm, ElevatorSubsystem.centerWidth);
        assertEquals(0.0, subsystem.GetHeight(), 0.001);
        subsystem.lower(); // io sim detects belt constraint
        subsystem.stop();
        assertEquals(0.0, subsystem.GetHeight(), 0.001);

        subsystem.rotate(12, 12);printPosition(subsystem);
        subsystem.rotate(12, 12);printPosition(subsystem);
        subsystem.rotate(12, 12);printPosition(subsystem);
        subsystem.rotate(12, 12);printPosition(subsystem);
        subsystem.rotate(12, 12);printPosition(subsystem);
        subsystem.rotate(12, 12);printPosition(subsystem);

        while (subsystem.GetHeight() < ElevatorSubsystem.maxHeight - 0.2) {
            subsystem.raise();
        }
        subsystem.raise(); System.out.println(Inches.convertFrom(subsystem.GetHeight(), Meters));
        subsystem.raise(); System.out.println(Inches.convertFrom(subsystem.GetHeight(), Meters));
        subsystem.raise(); System.out.println(Inches.convertFrom(subsystem.GetHeight(), Meters));
        subsystem.raise(); System.out.println(Inches.convertFrom(subsystem.GetHeight(), Meters));
        subsystem.raise(); System.out.println(Inches.convertFrom(subsystem.GetHeight(), Meters));
        subsystem.raise(); System.out.println(Inches.convertFrom(subsystem.GetHeight(), Meters));
        subsystem.raise(); System.out.println(Inches.convertFrom(subsystem.GetHeight(), Meters));
        subsystem.raise(); System.out.println(Inches.convertFrom(subsystem.GetHeight(), Meters));
        subsystem.raise(); System.out.println(Inches.convertFrom(subsystem.GetHeight(), Meters));
        subsystem.raise(); System.out.println(Inches.convertFrom(subsystem.GetHeight(), Meters));
        subsystem.raise(); System.out.println(Inches.convertFrom(subsystem.GetHeight(), Meters));
    }
    
    @Test
    public void SubsystemRotateTest() {
        HAL.initialize(500, 0);
        ElevatorIOSim io = new ElevatorIOSim();
        ElevatorSubsystem subsystem = new ElevatorSubsystem(io);
        printPosition(subsystem);
        subsystem.rotate(12, 12);printPosition(subsystem);
        subsystem.rotate(4, 12);printPosition(subsystem);
        subsystem.rotate(3, 12);printPosition(subsystem);
        subsystem.rotate(2, 12);printPosition(subsystem);
        subsystem.rotate(1, 12);printPosition(subsystem);
        subsystem.rotate(0, 12);printPosition(subsystem);
        subsystem.rotate(-1, 12);printPosition(subsystem);
        subsystem.rotate(-2, 12);printPosition(subsystem);
        subsystem.rotate(-3, 12);printPosition(subsystem);
        subsystem.rotate(-4, 12);printPosition(subsystem);
        subsystem.rotate(-5, 12);printPosition(subsystem);
        subsystem.rotate(-6, 12);printPosition(subsystem);
        subsystem.rotate(-7, 12);printPosition(subsystem);
        subsystem.rotate(-8, 12);printPosition(subsystem);
        subsystem.rotate(-9, 12);printPosition(subsystem);
        subsystem.rotate(-10, 12);printPosition(subsystem);
        subsystem.rotate(-11, 12);printPosition(subsystem);
        subsystem.rotate(-12, 12);printPosition(subsystem);
        subsystem.rotate(-12, 12);printPosition(subsystem);
        subsystem.rotate(-12, 12);printPosition(subsystem);
        subsystem.rotate(-12, 12);printPosition(subsystem);
        subsystem.rotate(-12, 12);printPosition(subsystem);
        subsystem.rotate(-12, 12);printPosition(subsystem);
        subsystem.rotate(-12, 12);printPosition(subsystem);
        subsystem.rotate(-12, 12);printPosition(subsystem);
    }

    void printPosition(ElevatorSubsystem subsystem) {
        System.out.printf("%.2f %.2f %.2f %.2f%n",
            Inches.convertFrom(subsystem.getLeftPositionMeters(), Meters),
            Inches.convertFrom(subsystem.getRightPositionMeters(), Meters),
            Inches.convertFrom(subsystem.GetHeight(), Meters),
            subsystem.getArmDegrees()
        );
    }


    @Test
    public void LiftTest() {
        HAL.initialize(500, 0);
        ElevatorIOSim io = new ElevatorIOSim();
        ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
        double eh = 0; // height of ElevatorSim
        
        io.setVoltage(12, 12); // pull elevator up
        periodic(io, inputs);
        eh = io.elevator.getPositionMeters();
        periodic(io, inputs);
        eh = io.elevator.getPositionMeters();
        periodic(io, inputs);
        eh = io.elevator.getPositionMeters();
    }

    @Test
    public void RotateTest() {
        HAL.initialize(500, 0);
        ElevatorIOSim io = new ElevatorIOSim();
        ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
        double seh = 0; // height of ElevatorSim
        double eh = 0; // height of ElevatorSim
        double arm = 0; // angle
        
        io.setVoltage(-12, 12); // spin arm
        periodic(io, inputs);
        seh = io.elevator.getPositionMeters();
        eh = inputs.elevatorPosition;
        arm = inputs.armPositionRad;
        periodic(io, inputs);
        seh = io.elevator.getPositionMeters();
        eh = inputs.elevatorPosition;
        arm = inputs.armPositionRad;
        periodic(io, inputs);
        seh = io.elevator.getPositionMeters();
        eh = inputs.elevatorPosition;
        arm = inputs.armPositionRad;
    }

    static void periodic(ElevatorIO io, ElevatorIOInputsAutoLogged inputs) {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }
}
