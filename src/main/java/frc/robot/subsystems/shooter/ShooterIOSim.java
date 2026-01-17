package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.kD;
import static frc.robot.subsystems.shooter.ShooterConstants.kI;
import static frc.robot.subsystems.shooter.ShooterConstants.kP;
import static frc.robot.subsystems.shooter.ShooterConstants.kV;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    private FlywheelSim sim;
    private PIDController pid;

    public ShooterIOSim() {
        this.sim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.01, 1), DCMotor.getNEO(1));
        this.pid = new PIDController(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble());
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        sim.update(0.02);

        inputs.shootAppliedVolts = sim.getInputVoltage();
        inputs.shootVelocity = sim.getAngularVelocityRadPerSec();
        inputs.shootCurrent = new double[] { sim.getCurrentDrawAmps() };

        sim.setInputVoltage(
                pid.calculate(sim.getAngularVelocityRadPerSec()) + kV.getAsDouble() * pid.getSetpoint());
    }

    @Override
    public void setShootVelocity(double velocity) {
        pid.setSetpoint(velocity);
    }

    @Override
    public void setShootVoltage(double voltage) {
        pid.setSetpoint(0.0);
        sim.setInputVoltage(0.0);
    }

}
