package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive {

    // Limits and States
    boolean highGear = false;
    boolean cutoff = false;
    double forwardLimit = 1.0;
    double turnLimit = 0.8;
    double minQuick = 0.3;
    double revolutionsPerMetre = 2.0916;
    double encoderCountsPerRevolution = 16384;

    // Motors And PID Controllers
    CANSparkMax leftFrontDrive;
    CANSparkMax leftRearDrive;
    CANSparkMax rightFrontDrive;
    CANSparkMax rightRearDrive;
    CANPIDController leftController1;
    CANPIDController leftController2;
    CANPIDController rightController1;
    CANPIDController rightController2;

    AHRS navx = new AHRS(SPI.Port.kMXP);

    // Encoders
    CANEncoder leftFrontEncoder;
    CANEncoder leftRearEncoder;
    CANEncoder rightFrontEncoder;
    CANEncoder rightRearEncoder;
    Encoder leftDriveEncoder;
    Encoder rightDriveEncoder;

    // Solenoids
    DoubleSolenoid gearShift;

    // Motor Control For Teleop
    SpeedControllerGroup leftMotors;
    SpeedControllerGroup rightMotors;
    DifferentialDrive drive;

    // Create a Drive object and setup encoders and motors
    public Drive(boolean isPracticeBot) {
        leftFrontDrive = new CANSparkMax(1, MotorType.kBrushless);
        leftRearDrive = new CANSparkMax(2, MotorType.kBrushless);
        rightFrontDrive = new CANSparkMax(3, MotorType.kBrushless);
        rightRearDrive = new CANSparkMax(4, MotorType.kBrushless);
        leftController1 = new CANPIDController(leftFrontDrive);
        leftController2 = new CANPIDController(leftRearDrive);
        rightController1 = new CANPIDController(rightFrontDrive);
        rightController2 = new CANPIDController(rightRearDrive);

        leftFrontEncoder = new CANEncoder(leftFrontDrive);
        leftRearEncoder = new CANEncoder(leftRearDrive);
        rightFrontEncoder = new CANEncoder(rightFrontDrive);
        rightRearEncoder = new CANEncoder(rightRearDrive);
        leftDriveEncoder = new Encoder(0, 1);
        rightDriveEncoder = new Encoder(2, 3);

        if (isPracticeBot) {
            gearShift = new DoubleSolenoid(30, 0, 7);
            leftDriveEncoder.setDistancePerPulse((-1.0 / encoderCountsPerRevolution) * revolutionsPerMetre);
            rightDriveEncoder.setDistancePerPulse((1.0 / encoderCountsPerRevolution) * revolutionsPerMetre);
        } else {
            gearShift = new DoubleSolenoid(30, 4, 1);
            leftDriveEncoder.setDistancePerPulse((1.0 / encoderCountsPerRevolution) * revolutionsPerMetre);
            rightDriveEncoder.setDistancePerPulse((-1.0 / encoderCountsPerRevolution) * revolutionsPerMetre);
        }

        leftMotors = new SpeedControllerGroup(leftFrontDrive, leftRearDrive);
        rightMotors = new SpeedControllerGroup(rightFrontDrive, rightRearDrive);
        drive = new DifferentialDrive(leftMotors, rightMotors);

        leftFrontDrive.setIdleMode(IdleMode.kCoast);
        leftRearDrive.setIdleMode(IdleMode.kCoast);
        rightFrontDrive.setIdleMode(IdleMode.kCoast);
        rightRearDrive.setIdleMode(IdleMode.kCoast);
        
        resetEncoders();
    }

    public void resetEncoders() {
        leftFrontEncoder.setPosition(0);
        leftRearEncoder.setPosition(0);
        rightFrontEncoder.setPosition(0);
        rightRearEncoder.setPosition(0);
        leftDriveEncoder.reset();
        rightDriveEncoder.reset();
    }

    public double driveDistance(double distance, double voltage, double angle) {
        double currentDistanceL = leftDistance();
        double distanceErrorL = distance-currentDistanceL;
        double currentDistanceR = rightDistance();
        double distanceErrorR = distance-currentDistanceR;

        double navxAngle = navx.getAngle();
        SmartDashboard.putNumber("Navx Angle: ", navxAngle);
        double angleError = navxAngle-angle;
        angleError = Robot.scaleAndClamp(angleError, -3, 3, -0.4, 0.4);
        angleError*=-1;
        SmartDashboard.putNumber("Angle Error: ", angleError);
        /*if (distanceError > 0.5) {
            shiftGear(true);
        } else {
            shiftGear(false);
        }*/

        shiftGear(false);

        if (Math.abs(distanceErrorL)>0 && Math.abs(distanceErrorR)>0) {
            drive.arcadeDrive(voltage/12.0, angleError);
        } else {
            drive.arcadeDrive(0, 0);
        }
        return (distanceErrorL < distanceErrorR) ? distanceErrorL : distanceErrorR;
    }

    public double turnDegrees(double degrees, double voltage) {
        return turnDegrees(degrees, voltage, 0.6);
    }

    public double turnDegrees(double degrees, double voltage, double turnFactor) {
        double currentDegrees = navx.getAngle();
        double angleError = degrees-currentDegrees;
        if (angleError>-5 && angleError<5) {
            drive.arcadeDrive(0, 0);
        } else if (angleError<-5) {
            drive.curvatureDrive(voltage / 12.0, (-1*turnFactor), false);
        } else if (angleError>5) {
            drive.curvatureDrive(voltage / 12.0, turnFactor, false);
        }
        return angleError;
    }

    // Grab the distance travelled by the left drive
    private double leftDistance() {
        return leftDriveEncoder.getDistance();
    }

    // Grab the distance travelled by the right drive
    private double rightDistance() {
        return rightDriveEncoder.getDistance();
    }

    // Grab the average distance traveled by the drive encoders
    public double distance() {
        final double leftDistance = leftDistance();
        final double rightDistance = rightDistance();
        return (leftDistance + rightDistance) / 2.0;
    }

    // Grab the speed of the left side of the drive encoders
    private double leftSpeed() {
        return leftDriveEncoder.getRate();
    }

    // Grab the speed of the right side of the drive encoders
    private double rightSpeed() {
        return rightDriveEncoder.getRate();
    }

    // Grab the average speed of the drive encoders
    public double speed() {
        final double rSpeed = rightSpeed();
        final double lSpeed = leftSpeed();
        return (rSpeed + lSpeed) / 2.0;
    }

    // Print the speeds to the dashboard
    public void printSpeed() {
        /*SmartDashboard.putNumber("Left Front Velocity", leftFrontEncoder.getVelocity());
        SmartDashboard.putNumber("Left Rear Velocity", leftRearEncoder.getVelocity());
        SmartDashboard.putNumber("Right Front Velocity", rightFrontEncoder.getVelocity());
        SmartDashboard.putNumber("Right Rear Velocity", rightRearEncoder.getVelocity());
        SmartDashboard.putNumber("Left Drive Velocity", leftSpeed());
        SmartDashboard.putNumber("Right Drive Velocity", rightSpeed());*/
    }

    // Print encoder position to the dashboard
    public void printPosition() {
       /* SmartDashboard.putNumber("Left Front Position", leftFrontEncoder.getPosition());
        SmartDashboard.putNumber("Left Rear Position", leftRearEncoder.getPosition());
        SmartDashboard.putNumber("Right Front Position", rightFrontEncoder.getPosition());
        SmartDashboard.putNumber("Right Rear Position", rightRearEncoder.getPosition());*/
        SmartDashboard.putNumber("Left Drive Position", leftDriveEncoder.getDistance());
        SmartDashboard.putNumber("Right Drive Position", rightDriveEncoder.getDistance());
    }

    // Print motor temperature to the dashboard
    public void printTemperature() {
        SmartDashboard.putNumber("Left Front Motor Temperature: ", leftFrontDrive.getMotorTemperature());
        SmartDashboard.putNumber("Left Rear Motor Temperature: ", leftRearDrive.getMotorTemperature());
        SmartDashboard.putNumber("Right Front Motor Temperature: ", rightFrontDrive.getMotorTemperature());
        SmartDashboard.putNumber("Right Rear Motor Temperature: ", rightRearDrive.getMotorTemperature());
    }

    // Change sensitivity of curve drive quick turn
    public void setMinimumQuickTurn(final double minQuick) {
        this.minQuick = minQuick;
    }

    // Change turn radius to shrink or increase the amount the robot turns
    public void setTurnLimit(final double turnLimit) {
        this.turnLimit = turnLimit;
    }

    // Change the maximum speed of the robot
    public void setForwardLimit(final double forwardLimit) {
        this.forwardLimit = forwardLimit;
    }

    // Switch between cutoff mode and range mode
    public void changeCutoffOrRange(final boolean controlMode) {
        cutoff = controlMode;
    }

    // Toggle version of shift gear
    public void shiftGear() {
        shiftGear(highGear);
    }

    // Shift between low and high gear
    public void shiftGear(final boolean highGear) {
        if (highGear) {
            gearShift.set(Value.kReverse);
            setTurnLimit(0.55);
            this.highGear = false;
        } else {
            gearShift.set(Value.kForward);
            setTurnLimit(0.8);
            this.highGear = true;
        }
    }

    // Check if the robot should be in quick turn mode. returns true if robot should
    // be in quick turn mode
    private boolean checkQuickTurn(final double forward) {
        if (forward < minQuick && forward > (-1 * minQuick)) {
            return true;
        } else {
            return false;
        }
    }

    // Cutoff input
    private double cutoff(final double input, final double cut) {
        if (input > cut) {
            return cut;
        } else if (input < (-1 * cut)) {
            return cut * -1;
        } else {
            return input;
        }
    }

    // Run both sides of the robot independently
    public void tankDrive(final double leftSpeed, final double rightSpeed) {
        tankDrive(leftSpeed, rightSpeed, false);
    }

    // Run both sides of the robot independently. Square the inputs for more low
    // speed control
    public void tankDrive(double leftSpeed, double rightSpeed, final boolean square) {
        if (cutoff) {
            leftSpeed = cutoff(leftSpeed, forwardLimit);
            rightSpeed = cutoff(rightSpeed, forwardLimit);
        } else {
            leftSpeed = Robot.scaleAndClamp(leftSpeed, -1.0, 1.0, (forwardLimit * -1), forwardLimit);
            rightSpeed = Robot.scaleAndClamp(rightSpeed, -1.0, 1.0, (forwardLimit * -1), forwardLimit);
        }
        drive.tankDrive(leftSpeed, rightSpeed, square);
    }

    // Run robot drive using one stick of a controller
    public void arcadeDrive(final double forward, final double turnRadius) {
        arcadeDrive(forward, turnRadius, false);
    }

    // Run robot drive using one stick of a controller. Square the inputs for more
    // low speed control
    public void arcadeDrive(double forward, double turnRadius, final boolean square) {
        if (cutoff) {
            forward = cutoff(forward, forwardLimit);
            turnRadius = cutoff(turnRadius, turnLimit);
        } else {
            forward = Robot.scaleAndClamp(forward, -1.0, 1.0, (-1 * forwardLimit), forwardLimit);
            turnRadius = Robot.scaleAndClamp(turnRadius, -1.0, 1.0, (-1 * turnLimit), turnLimit);
        }
        drive.arcadeDrive(forward, turnRadius, square);
    }

    // Run robot drive using one stick of a controller. Robot drives around the
    // radius of a circle. When below a specific speed, robot turns fast
    public void curveDrive(final double forward, final double turnRadius) {
        curveDrive(forward, turnRadius, false);
    }

    // Run robot drive using one stick of a controller. Robot drives around the
    // radius of a circle. When below a specific speed, robot turns fast. Square
    // inputs for more control at low speed
    public void curveDrive(double forward, double turnRadius, final boolean square) {
        if (cutoff) {
            forward = cutoff(forward, forwardLimit);
            turnRadius = cutoff(turnRadius, turnLimit);
        } else {
            forward = Robot.scaleAndClamp(forward, -1.0, 1.0, (-1 * forwardLimit), forwardLimit);
            turnRadius = Robot.scaleAndClamp(turnRadius, -1.0, 1.0, (-1 * turnLimit), turnLimit);
        }

        final boolean quickTurn = checkQuickTurn(forward);
        if (square) {
            forward *= forward;
            turnRadius *= turnRadius;
        }
        drive.curvatureDrive(forward, turnRadius, quickTurn);
    }

}