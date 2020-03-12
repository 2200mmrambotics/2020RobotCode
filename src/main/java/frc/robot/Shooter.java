package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    public Motor topWheelMotor;
    public Motor bottomWheelMotor;
    public Motor tiltMotor;
    public Motor panMotor;
    public Motor middleFondlerMotor;
    public Motor leftFondlerMotor;
    public Motor rightFondlerMotor;

    private DigitalInput leftBallSensor;
    private DigitalInput middleBallSensor;
    private DigitalInput rightBallSensor;
    private DigitalInput middleBallSensor2;
    private DigitalInput leftRearBallSensor;
    private DigitalInput rightRearBallSensor;
    private boolean unjamMode = false;

    private DigitalInput tiltLimit;

    private int left_sensed;
    private int right_sensed;
    // private int middle_sensed;

    boolean trenchTilt = false;

    private boolean weSpinNow;
    private boolean isPracticeBot;
    private double limitPanLeft;
    private double limitPanRight;
    private double limitTilt;
    public int numberBallsShot = 0;
    private double fondleRPM = 1000;
    private double feedRPM = 1200;
    private double topRPM = 2250;
    private double bottomRPM = 4500;
    public double defaultThreePtPanAdjustCoeff = 0.33;
    public double highResThreePtPanAdjustCoeff = 0.33 * 0.33;
    public double threePtPanAdjustCoeff = defaultThreePtPanAdjustCoeff;


    public NetworkTable table;

    private boolean hasZeroedPan;
    private boolean hasZeroedTilt;
    boolean stowed;

    private boolean lastLoopWasAtTiltPosition;
    private boolean lastloopWasAtPanPosition;

   /* private boolean hopperStatusSaved;
    private boolean middleBallSaved;
    private boolean leftBallSaved;
    private boolean rightBallSaved;*/
    
    boolean safeToFondle = false;
    boolean lastLoopMiddleBallSensor = false;
    int middle_sensed = 0;

    Timer middleBallFalseEdgeDetection = new Timer();

    private BallFondlerStates ballFondlerState;

    boolean shootOverride;
    boolean activeAim;
    boolean shootWhenReady;
    boolean prePan;

    AHRS navx;

    public Shooter(boolean isPracticeBot, AHRS navx) {
        this.isPracticeBot = isPracticeBot;
        if (isPracticeBot) {
            limitPanLeft = -120;
            limitPanRight = 165;
            limitTilt = 46;
        } else {
            limitPanLeft = -120;
            limitPanRight = 165;
            limitTilt = 46;
        }
        this.navx = navx;

        topWheelMotor = new Motor(13, "top", 8192, 0.03, 0, 0, 0.017);
        bottomWheelMotor = new Motor(14, "bottom", 8192, 0.2, 0, 0, 0.012);
        tiltMotor = new Motor(12, "tilt", 304.3478269, 1.0, 0, 0, 0);
        panMotor = new Motor(11, "pan", 111.11111696969696969696969696969696969, 0.7, 0, 0, 0);
        middleFondlerMotor = new Motor(8, "middleFondler", 4096, 0, 0, 0, 0.15);
        leftFondlerMotor = new Motor(6, "leftFondler", 4096, 0, 0, 0, 0.35); // TODO find counts per unit
        rightFondlerMotor = new Motor(7, "rightFondler", 4096, 0, 0, 0, 0.35); // TODO find counts per unit

        leftBallSensor = new DigitalInput(11);
        middleBallSensor = new DigitalInput(10);
        rightBallSensor = new DigitalInput(12);
        middleBallSensor2 = new DigitalInput(13);

        tiltLimit = new DigitalInput(4); // true when not pressed, false when pressed

        // topWheelMotor.motor.configPeakCurrentLimit(5);
        // bottomWheelMotor.motor.configPeakCurrentLimit(5);
        tiltMotor.motor.configPeakCurrentLimit(15);
        panMotor.motor.configPeakCurrentLimit(15);
        // feedMotor.motor.configPeakCurrentLimit(5);

        leftFondlerMotor.motor.setSensorPhase(true);
        panMotor.motor.setSensorPhase(true);
        if (isPracticeBot) {
            topWheelMotor.motor.setSensorPhase(false);
            middleFondlerMotor.motor.setSensorPhase(true);
            bottomWheelMotor.motor.setSensorPhase(false);
        } else {
            topWheelMotor.motor.setSensorPhase(true);
            middleFondlerMotor.motor.setSensorPhase(false);
            bottomWheelMotor.motor.setSensorPhase(true);
        }

        leftFondlerMotor.motor.setNeutralMode(NeutralMode.Brake);
        rightFondlerMotor.motor.setNeutralMode(NeutralMode.Brake);
        middleFondlerMotor.motor.setNeutralMode(NeutralMode.Brake);
        topWheelMotor.motor.setNeutralMode(NeutralMode.Coast);
        bottomWheelMotor.motor.setNeutralMode(NeutralMode.Coast);
        tiltMotor.motor.setNeutralMode(NeutralMode.Brake);
        panMotor.motor.setNeutralMode(NeutralMode.Brake);

        tiltMotor.motor.configPeakOutputReverse(-1.0);
        tiltMotor.motor.configPeakOutputForward(1.0);

        tiltMotor.motor.setSelectedSensorPosition(14000); // 14000 = top
        hasZeroedTilt = true;

        weSpinNow = false;

        table = NetworkTableInstance.getDefault().getTable("limelight");

        hasZeroedPan = false;
        stowed = false;

        lastLoopWasAtTiltPosition = false;
        lastloopWasAtPanPosition = false;

        /*hopperStatusSaved = false;
        middleBallSaved = false;
        leftBallSaved = false;
        rightBallSaved = false;
        rearLeftBallSaved = false;
        rearRightBallSaved = false;*/

        ballFondlerState = BallFondlerStates.STOPPED;

        shootOverride = false;
        activeAim = false;
        shootWhenReady = false;
        prePan = false;

        printPIDF();
    }

    public void enableActiveAim() {
        activeAim = true;
        table.getEntry("ledMode").setNumber(3);
        table.getEntry("camMode").setNumber(0);
    }

    public void disableActiveAim() {
        activeAim = false;
        table.getEntry("ledMode").setNumber(1);
        table.getEntry("camMode").setNumber(1);
    }

    public void enablePrePan(){
        prePan = true;
    }

    public void disablePrePan(){
        prePan = false;

    }
    public void enableShootWhenReady() {
        shootWhenReady = true;
    }

    public void disableShootWhenReady() {
        shootWhenReady = false;
    }

    public void enableShootOverride() {
        shootOverride = true;
    }

    public void disableShootOverride() {
        shootOverride = false;
    }

    public void wheelControl() {
        if (shootOverride || shootWhenReady || activeAim) {
            topWheelMotor.setVelocity(topRPM);
            bottomWheelMotor.setVelocity(bottomRPM);
        } else {
            topWheelMotor.setPercentOutput(0);
            bottomWheelMotor.setPercentOutput(0);
        }
    }
/*
*it out
*/
    public void printPIDF() {
        bottomWheelMotor.printPIDF();
        topWheelMotor.printPIDF();
        /*tiltMotor.printPIDF();
        panMotor.printPIDF();
        middleFondlerMotor.printPIDF();
        leftFondlerMotor.printPIDF();
        rightFondlerMotor.printPIDF();*/
    }

    public void printSpeed(double driveSpeed) {
            SmartDashboard.putBoolean("Colour Wheel Mode: ", weSpinNow);
            SmartDashboard.putNumber("Top Wheel Motor RPM",
                    topWheelMotor.countsToUnits(topWheelMotor.motor.getSelectedSensorVelocity() * 600.0));
            SmartDashboard.putNumber("Bottom Wheel Motor RPM",
                    bottomWheelMotor.countsToUnits(bottomWheelMotor.motor.getSelectedSensorVelocity() * 600.0));
          //  SmartDashboard.putNumber("Bottom Percent", bottomWheelMotor.motor.getMotorOutputPercent());
            SmartDashboard.putNumber("Tilt Motor Angle", getTiltPosition());
          //  SmartDashboard.putNumber("Tilt Counts", tiltMotor.motor.getSelectedSensorPosition());
            SmartDashboard.putNumber("Pan Motor Angle", getPanPosition());
          //  SmartDashboard.putNumber("Pan Counts", panMotor.motor.getSelectedSensorPosition());
            SmartDashboard.putNumber("Feed Motor RPM",
                    middleFondlerMotor.countsToUnits(middleFondlerMotor.motor.getSelectedSensorVelocity() * 600.0));
            SmartDashboard.putNumber("Left fondler Motor RPM",
                    leftFondlerMotor.countsToUnits(leftFondlerMotor.motor.getSelectedSensorVelocity() * 600.0));
            SmartDashboard.putNumber("Right fondler Motor RPM",
                    rightFondlerMotor.countsToUnits(rightFondlerMotor.motor.getSelectedSensorVelocity() * 600.0));
            SmartDashboard.putBoolean("Tilt Limit", !tiltLimit.get());
            SmartDashboard.putBoolean("Left Ball Sensor: ", leftBallSensor.get());
           // SmartDashboard.putBoolean("Left Ball Sensor Debounced: ", isLeftBallPresent());
            SmartDashboard.putBoolean("Middle Ball Sensor1: ", middleBallSensor.get());
            SmartDashboard.putBoolean("Middle Ball Sensor2: ", middleBallSensor2.get());
            SmartDashboard.putBoolean("Middle Ball Sensor Combined: ", isMiddleBallPresent());
            SmartDashboard.putBoolean("Right Ball Sensor: ", rightBallSensor.get());
           // SmartDashboard.putBoolean("Right Ball Sensor Debounced: ", isRightBallPresent());/*
            if (ballFondlerState == BallFondlerStates.STOPPED) {
                SmartDashboard.putString("BallfondlerState", "stopped");
            } else if (ballFondlerState == BallFondlerStates.RUNNING_LEFT) {
                SmartDashboard.putString("BallfondlerState", "run left");
            } else if (ballFondlerState == BallFondlerStates.RUNNING_RIGHT) {
                SmartDashboard.putString("BallfondlerState", "run right");
            }
      //  SmartDashboard.putNumber("X Velocity", calculateXVelocity(driveSpeed));
      //  SmartDashboard.putNumber("Y Velocity", calculateYVelocity(driveSpeed));
    }

    public void setPIDF() {
        bottomWheelMotor.getPIDF();
        topWheelMotor.getPIDF();
        tiltMotor.getPIDF();
        panMotor.getPIDF();
        middleFondlerMotor.getPIDF();
        leftFondlerMotor.getPIDF();
        rightFondlerMotor.getPIDF();
    }

    public double getTiltPosition() {
        return tiltMotor.countsToUnits(tiltMotor.motor.getSelectedSensorPosition());
    }

    public double getPanPosition() {
        return panMotor.countsToUnits(panMotor.motor.getSelectedSensorPosition());
    }

    public boolean isMiddleBallPresent() {
        if (isPracticeBot) {
            return middleBallSensor.get();
        } else {
            return middleBallSensor.get() || middleBallSensor2.get();
        }
    }

    public boolean isLeftBallPresent() {
        if (leftBallSensor.get()) {
            left_sensed = 15;
        } else if (left_sensed > 0) {
            left_sensed--;
        }
        return left_sensed > 0;
    }

    public boolean isRightBallPresent() {
        if (rightBallSensor.get()) {
            right_sensed = 15;
        } else if (right_sensed > 0) {
            right_sensed--;
        }
        return right_sensed > 0;
    }

    public void pan(double panAdjust, double driveSpeed) {
        double driveVelocity = driveSpeed;
        double ballVelocity = 8.3;
        double panAngle = getPanPosition();
        
        double panSetpoint = getPanPosition();


        Double[] asdf = {};
        Double[] cornersXY = table.getEntry("tcornxy").getDoubleArray(asdf);

        int leftCornerInd = 0;
        int rightCornerInd = 0;
        int indLowestX = 0;
        int ind2ndLowestX = 0;
        double threePtPanAdjust = 0;

        double tmp = 3000; // Higher than any corner we should see
        if (cornersXY.length == 8) { // 4 corners = 4 x + 4 y
            for (int i = 0; i < cornersXY.length; i+=2) {
                SmartDashboard.putString("Corner " + i, "(" + cornersXY[i] + "," + cornersXY[i+1] + ")");
                if(cornersXY[i] < tmp) {
                    indLowestX = i;
                    tmp = cornersXY[i];
                }
            }
            tmp = 3000;
            for (int i = 0; i < cornersXY.length; i+=2) {
                if(cornersXY[i] < tmp && i != indLowestX) {
                    ind2ndLowestX = i;
                    tmp = cornersXY[i];
                }
            }

            double leftCornerX = 0;
            double rightCornerX = 0;

            if (cornersXY[indLowestX + 1] < cornersXY[ind2ndLowestX + 1]) {
                leftCornerX = cornersXY[indLowestX];
                rightCornerX = cornersXY[ind2ndLowestX];
            } else {
                leftCornerX = cornersXY[ind2ndLowestX];
                rightCornerX = cornersXY[indLowestX];
            }

            threePtPanAdjust = (rightCornerX - leftCornerX) * threePtPanAdjustCoeff;

            SmartDashboard.putNumber("LeftCornerX", leftCornerX);
            SmartDashboard.putNumber("RightCornerX", rightCornerX);
            SmartDashboard.putNumber("DiffCornerX", rightCornerX - leftCornerX);
            SmartDashboard.putNumber("threePtPanAdjust", threePtPanAdjust);
        }

        if (activeAim) {
            if (table.getEntry("tv").getDouble(0.0) > 0.0) {
                double ty = table.getEntry("ty").getDouble(0.0);
                double panVelocityAdjust = Math.toDegrees(Math.asin((driveVelocity*Math.sin(Math.toRadians(panAngle))/ballVelocity)));
                SmartDashboard.putNumber("Pan Velocity Adjust: ", panVelocityAdjust);
                double panNavxRotationAdjust = -7.0 * navx.getRate(); // Use navx to counter slow feedback loop on camera
             //   SmartDashboard.putNumber("Pan navx rotation Adjust: ", panNavxRotationAdjust);
                
                panSetpoint = getPanPosition() + ty + panVelocityAdjust + panNavxRotationAdjust;
                if  (threePtPanAdjust < 1.7 && threePtPanAdjust > -1.7) {
                    panSetpoint += threePtPanAdjust;
                    SmartDashboard.putBoolean("isThreePointAdjusting", true);
                } else {
                    SmartDashboard.putBoolean("isThreePointAdjusting", false);
                }

            } else if(!(shootWhenReady || shootOverride)){
                if (panAdjust<0) {
                    panSetpoint = Robot.scaleAndClamp(panAdjust, -1, 0, limitPanLeft, 0);
                } else {
                    panSetpoint = Robot.scaleAndClamp(panAdjust, 0, 1, 0, limitPanRight);
                }
            }
        } 
        else if(prePan){
            if (panAdjust<0) {
                panSetpoint = Robot.scaleAndClamp(panAdjust, -1, 0, limitPanLeft, 0);
            } else {
                panSetpoint = Robot.scaleAndClamp(panAdjust, 0, 1, 0, limitPanRight);
            }

        }
        else {
            panSetpoint = 0;
        }
        panMotor.setPosition(Robot.scaleAndClamp(panSetpoint, limitPanLeft, limitPanRight, limitPanLeft, limitPanRight));
       SmartDashboard.putNumber("Pan Setpoint:", panSetpoint);
    }

    public double angleToControllerPercent(double angle) {
        if (angle<0) {
            return Robot.scaleAndClamp(angle, limitPanLeft, 0, -1, 0);
        } else {
            return Robot.scaleAndClamp(angle, 0, limitPanRight, 0, 1);
        }
    }

    public double calculateYVelocity(double driveSpeed) {
        double angle = getPanPosition();
        driveSpeed*=Math.cos(Math.toRadians(angle));
        return driveSpeed;
    }

    public double calculateXVelocity(double driveSpeed) {
        double angle = getPanPosition();
        driveSpeed*=Math.sin(Math.toRadians(angle));
        return driveSpeed;
    }
    public void enableTrenchTilt() {
        trenchTilt = true;
    }

    public void disableTrenchTilt() {
        trenchTilt = false;
    }

    public void tilt(double tiltAdjust, double driveSpeed) {
        double driveVelocity = driveSpeed;
        double ballVelocity = 8.3;
        double shotAngle = getTiltPosition() + 24 + 90;

        double setPoint = getTiltPosition();


        if (!trenchTilt) {
            if (activeAim && !weSpinNow) {
                if ((table.getEntry("ta").getDouble(0.0))>0) {
                    double distance = calculateDistance();
                    // old factor = -10
                    //double tiltVelocityAdjust = -10*Math.asin((driveVelocity*Math.sin(Math.toRadians(shotAngle))/ballVelocity));
                    double tiltVelocityAdjustTheSequel = Math.toDegrees(Math.asin(driveVelocity*Math.sin(69.5-getTiltPosition())/(ballVelocity+driveVelocity)));
                    double tiltVelocityAdjust = -0.5*Math.toDegrees(Math.asin((driveVelocity*Math.sin(Math.toRadians(shotAngle))/ballVelocity)));
                    SmartDashboard.putNumber("Tilt Velocity Adjust: ", tiltVelocityAdjust);
                    SmartDashboard.putNumber("Tilt Velocity Adjust The Sequel: ", tiltVelocityAdjustTheSequel);
                    double tx = table.getEntry("tx").getDouble(0.0);
                  //  SmartDashboard.putNumber("Distance: ", distance);
                    if (distance > 0 && distance < 50) {
                        setPoint = getTiltPosition() + tx + tiltVelocityAdjust + 2.5;
                    } else if (distance > 50 && distance < 120) {
                        setPoint = getTiltPosition() + tx + tiltVelocityAdjust - 0.5;
                    } else if (distance>120 && distance<260) {
                        double scaled = (Robot.scaleAndClamp(distance, 120.0, 260.0, 2.0, 11.0));
                     //   SmartDashboard.putNumber("Scaled Tilt", scaled);
                        setPoint = getTiltPosition() + tx - scaled + tiltVelocityAdjust;
                    }
                } else if (!(shootOverride||shootWhenReady)){
                    setPoint = Robot.scaleAndClamp(tiltAdjust, -1.0, 1.0, 0.0, limitTilt);
                }
            } else if(weSpinNow) {
                setPoint = limitTilt;
            } else {
                setPoint = limitTilt;
            }
        } else {
            setPoint = 0;
        }
        tiltMotor.setPosition(Robot.scaleAndClamp(setPoint, 0, limitTilt, 0, limitTilt));

    }

    public void tiltToSpin() {
        weSpinNow = true;
    }

    public void untiltFromSpin() {
        weSpinNow = false;
    }

    public double getLimeAngle() {
        return 180.0 - (24.0 + getTiltPosition()) - 90.0;
    }

    public double getLimeHeight() {
        // bot = 20.5
        // top = 22
        return (1.5 / limitTilt) * getTiltPosition() + 20.5;
    }

    public double calculateDistance() {
        table.getEntry("ledMode").setNumber(0);
        double tv = table.getEntry("tv").getDouble(0.0);
        SmartDashboard.putNumber("LLtv: ", tv);
        if (tv > 0) {
            double tx = table.getEntry("tx").getDouble(0.0);
            double ty = table.getEntry("ty").getDouble(0.0);
            double angleLime = getLimeAngle();
            double heightLime = getLimeHeight();
            double distance = (94 - heightLime) / (Math.tan(Math.toRadians(angleLime - tx)));
            SmartDashboard.putNumber("LLtx: ", tx);
            SmartDashboard.putNumber("LLty: ", ty);
            SmartDashboard.putNumber("LLAngle: ", angleLime);
            SmartDashboard.putNumber("Distance: ", distance);

            return distance;
        }
        return (-1);

    }

    public boolean wheelsAtSpeed() {
        return (topWheelMotor.isAtVelocitySetPoint(300) && bottomWheelMotor.isAtVelocitySetPoint(300));
    }

    public void fire(boolean doFire) {
        if (doFire) {
            middleFondlerMotor.setVelocity(feedRPM);
        } else if (ballFondlerState == BallFondlerStates.RUNNING_LEFT
        || ballFondlerState == BallFondlerStates.RUNNING_RIGHT) {
            middleFondlerMotor.setVelocity(700);
        }
         else {
            middleFondlerMotor.setVelocity(0);
        }
    }

    public boolean isAimed() {
        if ((table.getEntry("ta").getDouble(0.0))>0 && ((table.getEntry("ty").getDouble(0.0)) <3) && ((table.getEntry("ty").getDouble(0.0))>-3))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public void turretControl() {
        if (shootOverride) {
            if (wheelsAtSpeed()) {
                fire(true);
            } else {
                fire(false);
            }
        } else {
            if (activeAim && isAimed() && wheelsAtSpeed() && shootWhenReady) {
                fire(true);
            } else {
                fire(false);
            }
        }
    }

    public void ballFondler(){
        if (lastLoopMiddleBallSensor && !isMiddleBallPresent()) {
            middleBallFalseEdgeDetection.reset();
            middleBallFalseEdgeDetection.start();
            numberBallsShot++;
        }

        lastLoopMiddleBallSensor = isMiddleBallPresent();

        if(!shootWhenReady && !shootOverride){
            //hopperStatusSaved = false;
            boolean middleBallPresent = isMiddleBallPresent();
            boolean leftBallPresent = isLeftBallPresent();
            boolean rightBallPresent = isRightBallPresent();
            if(!middleBallPresent && leftBallPresent && !(ballFondlerState==BallFondlerStates.RUNNING_RIGHT)){
                    ballFondlerState = BallFondlerStates.RUNNING_LEFT;
                } 
            else if(!middleBallPresent && rightBallPresent && !(ballFondlerState==BallFondlerStates.RUNNING_LEFT)){
                    ballFondlerState = BallFondlerStates.RUNNING_RIGHT;
                }
            else if(!middleBallPresent && !leftBallPresent && !rightBallPresent){
                ballFondlerState = BallFondlerStates.STOPPED;
            }
            else{
                ballFondlerState = BallFondlerStates.STOPPED;
            }
            middleBallFalseEdgeDetection.reset();
        }

        if(shootWhenReady||shootOverride) {
            if (leftBallSensor.get() && wheelsAtSpeed() && isMiddleBallPresent()) {
                ballFondlerState = BallFondlerStates.RUNNING_LEFT;
            } else if (rightBallSensor.get() && wheelsAtSpeed() && isMiddleBallPresent()) {
                ballFondlerState = BallFondlerStates.RUNNING_RIGHT;
            } else if (middleBallFalseEdgeDetection.get()>0.30 && leftBallSensor.get() && wheelsAtSpeed()) {
                ballFondlerState = BallFondlerStates.RUNNING_LEFT;
                middleBallFalseEdgeDetection.reset();
            } else if (middleBallFalseEdgeDetection.get()>0.30 && rightBallSensor.get() && wheelsAtSpeed()) {
                ballFondlerState = BallFondlerStates.RUNNING_RIGHT;
                middleBallFalseEdgeDetection.reset();
            }
        }


    // state actions
        if(ballFondlerState==BallFondlerStates.STOPPED && !unjamMode) {
            leftFondlerMotor.setVelocity(0);
            rightFondlerMotor.setVelocity(0);
        } else if (ballFondlerState==BallFondlerStates.RUNNING_LEFT && !unjamMode) {
            if (shootOverride||shootWhenReady) {
                leftFondlerMotor.setVelocity(fondleRPM);
            } else {
                leftFondlerMotor.setVelocity(400);
            }
            rightFondlerMotor.setVelocity(-0);
        } else if (ballFondlerState==BallFondlerStates.RUNNING_RIGHT && !unjamMode) {
            if (shootOverride||shootWhenReady) {
                rightFondlerMotor.setVelocity(fondleRPM);
            } else {
                rightFondlerMotor.setVelocity(400);
            }
            leftFondlerMotor.setVelocity(-0);
        }
    }

    public void unjam(boolean state){
        if(state == true){
            unjamMode = true;
            leftFondlerMotor.setVelocity(-100);
            rightFondlerMotor.setVelocity(-100);
        }
        if(state == false){
            leftFondlerMotor.setVelocity(0);
            rightFondlerMotor.setVelocity(0);
            unjamMode = false;
        }
        
    }

    public void stowForever() {
        stowed = true;
    }

    public void doThings(double panAdjust, double tiltAdjust, double driveSpeed) {
        printSpeed(driveSpeed);

        if (tiltLimit.get()) { // Not Pressed Limit Switch
            tiltMotor.motor.configPeakOutputReverse(-1.0);
        } 
        else { // Pressed Limit Switch
            tiltMotor.motor.configPeakOutputReverse(0.0);
            tiltMotor.motor.setSelectedSensorPosition(0, 0, 10);
        }

        if (hasZeroedTilt && hasZeroedPan) {
            
            if (tiltMotor.isAtPositionSetPoint(0.05) && !lastLoopWasAtTiltPosition) {
                tiltMotor.setPIDF(0, 0, 0, 0);
                lastLoopWasAtTiltPosition = true;

            } else if (!tiltMotor.isAtPositionSetPoint(0.05) && lastLoopWasAtTiltPosition){
                tiltMotor.setDefaultPIDF();
                lastLoopWasAtTiltPosition = false;
            }

            if (panMotor.isAtPositionSetPoint(0.5) && !lastloopWasAtPanPosition) {
                lastloopWasAtPanPosition = true;
                panMotor.setPIDF(0, 0, 0, 0);
            } else if (!panMotor.isAtPositionSetPoint(0.5) && lastloopWasAtPanPosition){
                lastloopWasAtPanPosition = false;
                panMotor.setDefaultPIDF();
            }
            if (!stowed) {
                pan(panAdjust, driveSpeed);
                tilt(tiltAdjust, driveSpeed);
                wheelControl();
                turretControl();
                ballFondler();  
            } else {
                panMotor.setPosition(20.0);
                tiltMotor.setPosition(0);
            }
        } else {
            /*
                Zero the tilt mechanism
            */
            // search for lower limit switch
            if (!tiltLimit.get()) { // Yes Pressed
                tiltMotor.setPercentOutput(0);
                hasZeroedTilt = true;
                tiltMotor.motor.setSelectedSensorPosition(0, 0, 10);
            }  else { // No Pressed
                tiltMotor.setPercentOutput(-0.6);
            }
        
            /*
                TODO zero the pan
            */
            hasZeroedPan = true;
            panMotor.motor.setSelectedSensorPosition(0, 0, 10);
        }
    }
}