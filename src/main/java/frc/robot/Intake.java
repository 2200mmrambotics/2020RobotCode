package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake {

    // Motors and solenoid
    Motor motor;
    DoubleSolenoid intakeArm;

    // Constructor of Intake object. Sets the motor to run off of motorPort and intakeArm to use channels declared.
    public Intake(boolean isPracticeBot) {
        motor = new Motor(5, "IntakeRoller", 1);
        if (isPracticeBot) {
            intakeArm = new DoubleSolenoid(30, 1, 6);
        } else {
            intakeArm = new DoubleSolenoid(30,  3, 2);
        }
    }

    // Check for if the intake solenoid is in reverse. Returns true if the solenoid is in reverse mode
    public boolean isDown() {
        if (intakeArm.get()==Value.kReverse) {
            return true;
        }
        return false;
    }

    // Bring the intake up
    public void up() {
        intakeArm.set(Value.kForward);
    }

    // Bring the intake down
    public void down() {
        intakeArm.set(Value.kReverse);
    }

    // Roll balls in
    public void in() {
        motor.setPercentOutput(1.0);
    }

    public void inSlow() {
        motor.setPercentOutput(2.0/3.0);
    }

    // Roll balls out of system
    public void out() {
        motor.setPercentOutput(-1.0);
    }

    // Stop rollers
    public void stop() {
        motor.setPercentOutput(0);
    }
}