package frc.robot.subsystems;

import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;

public class ElevatorTests {

    @Test
    public void RotateTest() {

        ElevatorIOSim io = new ElevatorIOSim();
        ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
        double eh = 0; // hieght of ElevatorSim

        io.setVoltage(12, 12); // pull elevator up
        periodic(io, inputs);
        eh = io.elevator.getPositionMeters();
        periodic(io, inputs);
        periodic(io, inputs);
    }

    static void periodic(ElevatorIO io, ElevatorIOInputsAutoLogged inputs) {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }
}
