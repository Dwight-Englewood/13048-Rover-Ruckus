package org.firstinspires.ftc.teamcode.Hardware;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.TensorFlowStuff.TensorFlow;

public class BoBot {
    public static DcMotor BL, BR, FL, FR, hook, lift, intake, joint;
    public Servo door;
    public DigitalChannel liftLimit, hookLimit;
    public RevBlinkinLedDriver blinkin;
    HardwareMap map;
    Telemetry tele;
    TensorFlow tensorFlow;

    Double powerModifier = 0.02;
    double turnSpeed = 0.25;
    final double proportionalValue = 0.000005;

    //double error = 180 - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    //Double turnSpeed = 0.5;
    //Integer angle = -45;
    public static BNO055IMU gyro;
    BNO055IMU.Parameters parameters;
    Orientation angles;

    public BoBot() {
    }

    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        this.map = map;
        this.tele = tele;

        hook = this.map.get(DcMotor.class, "hook");
        BR = this.map.get(DcMotor.class, "BR");
        BL = this.map.get(DcMotor.class, "BL");
        FL = this.map.get(DcMotor.class, "FL");
        FR = this.map.get(DcMotor.class, "FR");
        lift = this.map.get(DcMotor.class, "lift");
        intake = this.map.get(DcMotor.class, "intake");
        joint = this.map.get(DcMotor.class, "joint");

        door = this.map.get(Servo.class, "door");
        blinkin = this.map.get(RevBlinkinLedDriver.class, "rgbReady");

        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        joint.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        hook.setDirection(DcMotorSimple.Direction.REVERSE);

        this.changeRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        gyro = this.map.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);
        tele.addData(">", "Gyro Calibrating. Do Not Move!");
        tele.update();

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        joint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static void changeRunMode(DcMotor.RunMode runMode) {
        BL.setMode(runMode);
        BR.setMode(runMode);
        FL.setMode(runMode);
        FR.setMode(runMode);
        hook.setMode(runMode);
        lift.setMode(runMode);
        intake.setMode(runMode);
    }

    public void drive(double in) {
        BL.setPower(in);
        BR.setPower(in);
        FR.setPower(in);
        FL.setPower(in);
    }

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

    public void tankDrive(double leftStick, double rightStick, double leftTrigger, double rightTrigger, boolean invert, boolean brake) {
        double i = invert ? 0.4 : 0.65;
        //  double s = sickoMode ? 0.4 : 1;

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

    public void BoBosAntimatterDefuser(MovementEnum movement, int target) {
        switch (movement) {
            case FORWARD:
                this.getPos();
                FL.setTargetPosition(FLcurPos()+target);
                FR.setTargetPosition(FRcurPos()+target);
                BL.setTargetPosition(BLcurPos()+target);
                BR.setTargetPosition(BRcurPos()+target);
                break;

            case BACKWARD:
                this.getPos();
                FL.setTargetPosition(FLcurPos()-target);
                FR.setTargetPosition(FRcurPos()-target);
                BL.setTargetPosition(BLcurPos()-target);
                BR.setTargetPosition(BRcurPos()-target);
                break;

            case LEFTSTRAFE:
                this.getPos();
                FL.setTargetPosition(FLcurPos()-target);
                FR.setTargetPosition(FRcurPos()+target);
                BL.setTargetPosition(BLcurPos()+target);
                BR.setTargetPosition(BRcurPos()-target);
                break;

            case RIGHTSTRAFE:
                this.getPos();
                FL.setTargetPosition(FLcurPos()+target);
                FR.setTargetPosition(FRcurPos()-target);
                BL.setTargetPosition(BLcurPos()-target);
                BR.setTargetPosition(BRcurPos()+target);
                break;

            case LEFTTURN:
                this.getPos();
                FL.setTargetPosition(FLcurPos()-target);
                FR.setTargetPosition(FRcurPos()+target);
                BL.setTargetPosition(BLcurPos()-target);
                BR.setTargetPosition(BRcurPos()+target);
                break;

            case RIGHTTURN:
                this.getPos();
                FL.setTargetPosition(FLcurPos()+target);
                FR.setTargetPosition(FRcurPos()-target);
                BL.setTargetPosition(BLcurPos()+target);
                BR.setTargetPosition(BRcurPos()-target);
                break;

            case STOP:
                FL.setTargetPosition(FL.getCurrentPosition());
                FR.setTargetPosition(FR.getCurrentPosition());
                BL.setTargetPosition(BL.getCurrentPosition());
                BR.setTargetPosition(BR.getCurrentPosition());
                break;
        }
    }

    public int FLcurPos() {
        return FL.getCurrentPosition();
    }

    public int FRcurPos() {
        return FR.getCurrentPosition();
    }

    public int BLcurPos() {
        return BL.getCurrentPosition();
    }

    public int BRcurPos() {
        return BR.getCurrentPosition();
    }

    public void getPos() {
        FLcurPos();
        FRcurPos();
        BLcurPos();
        BRcurPos();
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
                BL.setPower(-power);
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

    private int distanceToRevs(double distance) {
        final double wheelCircumference = 31.9185813;
        final double gearMotorTickCount = 1120;  //Neverest 40 = 280 Pulses per revolution, 1120 Counts per revolution
        return (int) (gearMotorTickCount * (distance / wheelCircumference));
    }

    public double fetchHeading() {
        return gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void autonDriveUltimate(MovementEnum movementEnum, int target, double power) {
        this.autonDrive(movementEnum, target);
        this.setPower(power);
        this.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (Math.abs(FL.getCurrentPosition()) >= Math.abs(FL.getTargetPosition())) {
            drive(MovementEnum.STOP, 0);
            tele.update();
        }
    }

    public void BoBoTractor(MovementEnum movementEnum, int target, double power) {
        this.BoBosEncoders(movementEnum, target);
        this.setPower(power);
        this.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (Math.abs(FL.getCurrentPosition()) >= Math.abs(FL.getTargetPosition())) {
            drive(MovementEnum.STOP, 0);
            tele.update();
        }
    }

    public void pidTest(MovementEnum movementEnum, int target) {
        this.autonDrive(movementEnum, target);
        this.setPower(motorSpeed());
        this.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (Math.abs(FL.getCurrentPosition()) >= Math.abs(FL.getTargetPosition())) {
            drive(MovementEnum.STOP, 0);
            tele.update();
        }
    }

    public boolean adjustHeading(int targetHeading) {
        double curHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double headingError;
        headingError = targetHeading - curHeading;
        double driveScale = headingError;
        this.changeRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(headingError < -0.3)
            driveScale = -0.15;
        else if(headingError > 0.3)
            driveScale = 0.15;
        else {
            driveScale = 0;
            this.drive(MovementEnum.LEFTTURN, driveScale);
            return true;
        }
        this.drive(MovementEnum.LEFTTURN, driveScale);
        //    this.tele.addData("drive Scale",driveScale);
        //   tele.update();
        //   this.tankDrive(driveScale, -driveScale, 0, 0, false, false);
        // this.changeRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return false;
    }

    public void headingAdjuster(int targetHeading) {
        if(Math.abs(targetHeading - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
            this.adjustHeading(targetHeading);
        }
        else if(Math.abs(targetHeading - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
            this.drive(MovementEnum.STOP, 0);
            tele.update();
        }
    }

    public double motorSpeed() {
        if (Math.abs(FL.getCurrentPosition()) < Math.abs(FL.getTargetPosition())) {
        } return Math.abs(FL.getTargetPosition()) - Math.abs(FL.getCurrentPosition() * proportionalValue);
    }


}
