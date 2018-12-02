package org.firstinspires.ftc.teamcode;

/**
 * Created by joonsoolee on 9/21/18.
 */
    import android.text.StaticLayout;

        import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
    import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.hardware.GyroSensor;
    import com.qualcomm.robotcore.hardware.Gyroscope;
    import com.qualcomm.hardware.bosch.BNO055IMU;

    import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
    import com.qualcomm.robotcore.util.ElapsedTime;
    import com.qualcomm.robotcore.util.Range;
        import com.qualcomm.robotcore.hardware.*;
        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import java.util.*;
    import org.firstinspires.ftc.teamcode.MovementEnum;

    import org.firstinspires.ftc.robotcore.external.ClassFactory;
    import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
    import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
    import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
    import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class bot {

    //TODO add vex motor as a Servo for extension
    // TODO change dump mechanism to a vex motor?
    static DcMotor BL, BR, FL, FR, hook, lift, intake;
    Servo dump, claw;
    DigitalChannel liftLimit, hookLimit;
    HardwareMap map;
    Telemetry tele;

    Double powerModifier = 0.02;
    double turnSpeed = 0.25;
    final double proportionalValue = 0.000005;
    double integralValue = 0;
    double derivativeValue = 0;

    //   double error = 180 - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    //Double turnSpeed = 0.5;
    //Integer angle = -45;
    static BNO055IMU gyro;
    BNO055IMU.Parameters parameters;
    Orientation angles;

    public bot() {
    }

    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        this.map = map;
        this.tele = tele;

        //2 motors(Lift and Intake) and one servo(Dump)

//        left = this.map.get(DcMotor.class, "left");
//        right = this.map.get(DcMotor.class, "right");
        hook = this.map.get(DcMotor.class, "hook");
        BR = this.map.get(DcMotor.class, "BR");
        BL = this.map.get(DcMotor.class, "BL");
        FL = this.map.get(DcMotor.class, "FL");
        FR = this.map.get(DcMotor.class, "FR");
        lift = this.map.get(DcMotor.class, "lift");
        intake = this.map.get(DcMotor.class, "intake");

        hookLimit = this.map.get(DigitalChannel.class, "hookLimit");
        liftLimit = this.map.get(DigitalChannel.class, "liftLimit");

        dump = this.map.get(Servo.class,"dump");
        claw = this.map.get(Servo.class, "claw");

//        left.setDirection(DcMotorSimple.Direction.FORWARD);
//        right.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        hook.setDirection(DcMotorSimple.Direction.FORWARD);

        liftLimit.setMode(DigitalChannel.Mode.INPUT);
        hookLimit.setMode(DigitalChannel.Mode.INPUT);

        this.changeRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);  //TODO: Change to Run With Encoders Later
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        gyro = this.map.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);
        tele.addData(">", "Gyro Calibrating. Do Not Move!");
        tele.update();
//        this.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void changeRunMode(DcMotor.RunMode runMode) {
        BL.setMode(runMode);
        BR.setMode(runMode);
        FL.setMode(runMode);
        FR.setMode(runMode);
        hook.setMode(runMode);
//        left.setMode(runMode);
//        right.setMode(runMode);
        lift.setMode(runMode);
        intake.setMode(runMode);
    }

//    public void setRunMode(DcMotor.RunMode encoderRunMode) {
//        BR.setMode(runMode);
//        BL.setMode(runMode);
//        FL.setMode(runMode);
//        FR.setMode(runMode);
//        left.setMode(encoderRunMode);
//        right.setMode(encoderRunMode);
//        lift.setMode(encoderRunMode);
//        intake.setMode(encoderRunMode);
//    }

    public void drive(double in) {
        BL.setPower(in);
        BR.setPower(in);
        FR.setPower(in);
        FL.setPower(in);
    }
//    public void tankDriveNoStrafe (double leftStick, double rightStick){
//        left.setPower(leftStick);
//        right.setPower(rightStick);
//
//    }

    public void notKevinDrive(double leftStick_y, double leftStick_x, double leftTrigger, double rightTrigger) {
        if (leftTrigger > .3) {
            drive(MovementEnum.LEFTSTRAFE, leftTrigger);
            return;
        }
        if (rightTrigger > .3) {
            drive(MovementEnum.RIGHTSTRAFE, rightTrigger);
            return;
        }
        //   leftStick *= i;
        //     rightStick *= i;
        FL.setPower(leftStick_y);
        FR.setPower(leftStick_y);
        BL.setPower(-leftStick_y);
        BR.setPower(-leftStick_y);


    }
    //Lift(Right Joystick)
    public void tankDrive(double leftStick, double rightStick, double leftTrigger, double rightTrigger, boolean invert, boolean brake) {
        double i = invert ? -0.75 : 0.75;
        if (leftTrigger > .3) {
            drive(MovementEnum.LEFTSTRAFE, leftTrigger * i);
            return;
        }
        if (rightTrigger > .3) {
            drive(MovementEnum.RIGHTSTRAFE, rightTrigger * i);
            return;
        }

        leftStick *= i;
        rightStick *= i;

        FL.setPower(leftStick);
        FR.setPower(rightStick);
        BL.setPower(-leftStick);
        BR.setPower(-rightStick);
    }

    public void setPower(double power) {
        FL.setPower(power);
        BL.setPower(power);
        FR.setPower(power);
        BR.setPower(power);
    }

 /*   public void twoDrive(MovementEnum movement, double power){
        switch(movement){
            case FORWARD:
                left.setPower(power);
                right.setPower(power);
                break;

            case BACKWARD:
                left.setPower(-power);
                right.setPower(-power);
                break;

            case LEFTTURN:
                left.setPower(-power);
                right.setPower(power);
                break;

            case RIGHTTURN:
                left.setPower(power);
                right.setPower(-power);
                break;

            case STOP:
                left.setPower(0);
                right.setPower(0);
                break;
        }
    }

  */

    public void autonDrive(MovementEnum movement, int target) {
        switch (movement) {
            case FORWARD:
                FL.setTargetPosition(target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(target);
                break;

            case BACKWARD:
                FL.setTargetPosition(-target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(-target);
                break;

            case LEFTSTRAFE:
                FL.setTargetPosition(-target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(-target);
                break;

            case RIGHTSTRAFE:
                FL.setTargetPosition(target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(target);
                break;

            case LEFTTURN:
                FL.setTargetPosition(-target);
                FR.setTargetPosition(target);
                BL.setTargetPosition(-target);
                BR.setTargetPosition(target);
                break;

            case RIGHTTURN:
                FL.setTargetPosition(target);
                FR.setTargetPosition(-target);
                BL.setTargetPosition(target);
                BR.setTargetPosition(-target);
                break;

            case STOP:
                FL.setTargetPosition(FL.getCurrentPosition());
                FR.setTargetPosition(FR.getCurrentPosition());
                BL.setTargetPosition(BL.getCurrentPosition());
                BR.setTargetPosition(BR.getCurrentPosition());
                break;
        }
    }

    //TODO fix the the driver values and restrict the motor values
    public void drive(MovementEnum movement, double power) {
        switch (movement) {
            case FORWARD:
                FL.setPower(power);
                FR.setPower(power);
                BL.setPower(power);
                BR.setPower(power);
                break;

            case BACKWARD:
                FR.setPower(power);
                FL.setPower(power);
                BL.setPower(-power);
                BR.setPower(-power);
                break;

            case LEFTSTRAFE:
                FL.setPower(power);
                FR.setPower(-power);
                BL.setPower(power);
                BR.setPower(-power);
                break;

            case RIGHTSTRAFE:
                FL.setPower(-power);
                FR.setPower(power);
                BL.setPower(-power);
                BR.setPower(power);
                break;

            case LEFTTURN:
                FL.setPower(-power);
                FR.setPower(power);
                BL.setPower(power);
                BR.setPower(power);
                break;

            case RIGHTTURN:
                FL.setPower(power);
                FR.setPower(-power);
                BL.setPower(power);
                BR.setPower(-power);
                break;

            case STOP:
                FL.setPower(0);
                FR.setPower(0);
                BL.setPower(0);
                BR.setPower(0);
                break;
        }
    }

    public void turn(double in) {
        BL.setPower(in);
        BR.setPower(-in);
        FR.setPower(-in);
        FL.setPower(in);
    }

    public void setTarget(int targetDistance) {
        int target = this.distanceToRevs(targetDistance);
        FL.setTargetPosition(FL.getTargetPosition() / 1120);
        FR.setTargetPosition(FR.getTargetPosition() / 1120);
        BL.setTargetPosition(BL.getTargetPosition() / 1120);
        BR.setTargetPosition(BR.getTargetPosition() / 1120);
    }

    public void getDrivePosition() {
        FL.getCurrentPosition();
        FR.getCurrentPosition();
        BL.getCurrentPosition();
        BR.getCurrentPosition();
    }

    public int hookTarget() {
        hook.setTargetPosition(19040);
        double hookDistance = (hook.getTargetPosition() / 1120);
        return (int) (hookDistance);
    }

    public int hookCurrentPosition() {
        hook.getTargetPosition();
        return (hook.getCurrentPosition() / 1120);
    }

    public int hookRevolutionsRemaining() {
        hook.getTargetPosition();
        hook.getCurrentPosition();
        return ((hook.getTargetPosition() - hook.getCurrentPosition()) / 1120);
    }

    private int distanceToRevs(double distance) {
        final double wheelCircumference = 31.9185813;
        final double gearMotorTickCount = 1120;  //Neverest 40 = 280 Pulses per revolution, 1120 Counts per revolution
        return (int) (gearMotorTickCount * (distance / wheelCircumference));
    }
//targetHeading = -45
    // if (absolute value(absolute value(-45) - absolute value(current)) < 0.5), set powers to 0 and return true.
    public boolean adjustHeading(int targetHeading) {
        double curHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (Math.abs(Math.abs(targetHeading) - Math.abs(curHeading)) < .5) {
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
            return true;
        }
        double headingError;
//if target heading is equal to zero,
//do error is equal to current place < 0 return targetHeading+curHeading if true and absolute value if false.
        // else error = target + current place
        //drivescale = error * proportional(0.02)

        if (targetHeading == 0) {
            headingError = curHeading < 0 ? targetHeading + curHeading : Math.abs(targetHeading + curHeading);
        } else
            headingError = targetHeading - curHeading;
        double driveScale = headingError * powerModifier;

        if (Math.abs(driveScale) < .06) {
            driveScale = .06 * (driveScale < 0 ? -1 : 1);
        }
        Range.clip(driveScale, -1, 1);
        turn(driveScale);
        return false;
    }

    public void motorSpeed() {
/*
        double FLerror = Math.abs(FL.getTargetPosition()) - Math.abs(FL.getCurrentPosition());
        double FRerror = Math.abs(FR.getTargetPosition()) - Math.abs(FR.getCurrentPosition());
        double BLerror = Math.abs(BL.getTargetPosition()) - Math.abs(BL.getCurrentPosition());
        double BRerror = Math.abs(BR.getTargetPosition()) - Math.abs(BR.getCurrentPosition());
        */

        if (Math.abs(FL.getCurrentPosition()) < Math.abs(FL.getTargetPosition())) {
        FL.setPower(Math.abs(FL.getTargetPosition()) - Math.abs(FL.getCurrentPosition() * proportionalValue));
        FR.setPower(Math.abs(FR.getTargetPosition()) - Math.abs(FR.getCurrentPosition() * proportionalValue));
        BL.setPower(Math.abs(BL.getTargetPosition()) - Math.abs(BL.getCurrentPosition() * proportionalValue));
        BR.setPower(Math.abs(BR.getTargetPosition()) - Math.abs(BR.getCurrentPosition() * proportionalValue));
        tele.update();

        } else {
            autonDrive(MovementEnum.STOP, 0);

        }
    }
}

