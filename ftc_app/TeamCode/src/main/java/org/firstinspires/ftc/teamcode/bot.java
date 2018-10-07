package org.firstinspires.ftc.teamcode;

/**
 * Created by joonsoolee on 9/21/18.
 */
    import com.qualcomm.hardware.bosch.BNO055IMU;
        import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
        import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
        import com.qualcomm.robotcore.util.Range;
        import com.qualcomm.robotcore.hardware.*;
        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
        import java.util.*;
    import org.firstinspires.ftc.teamcode.MovementEnum;


public class bot {

    //TODO add vex motor as a Servo
    static DcMotor BL, BR, FL, FR, lift, intake, hook, hinge;
   //TODO add vex motor as a server 

    Servo dump;
    HardwareMap map;
    Telemetry tele;
    BNO055IMU.Parameters parameters;
    Orientation angles;
    Double powerModifier = 0.02;
    ModernRoboticsI2cColorSensor colorSensor;
    public bot() {}

    //TODO: Get Extension Stuff [Done]
    public void init(HardwareMap map, Telemetry tele, boolean auton){
        this.map = map;
        this.tele = tele;
        BL = this.map.get(DcMotor.class, "BL");
        BR = this.map.get(DcMotor.class, "BR");
        FL = this.map.get(DcMotor.class, "FL");
        FR = this.map.get(DcMotor.class, "FR");
        lift = this.map.get(DcMotor.class, "lift");
        hinge = this.map.get(DcMotor.class, "hinge");
        intake = this.map.get(DcMotor.class, "intake");
        hook = this.map.get(DcMotor.class, "hook");

        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        hinge.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        hook.setDirection(DcMotorSimple.Direction.FORWARD);

        this.changeRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //TODO: Set lift zero power mode [Done]
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hinge.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
    }
    public void changeRunMode(DcMotor.RunMode runMode){
        BL.setMode(runMode);
        BR.setMode(runMode);
        FL.setMode(runMode);
        FR.setMode(runMode);
    }

    public void drive(double in) {
        BL.setPower(in);
        BR.setPower(in);
        FR.setPower(in);
        FL.setPower(in);
    }
    public void tankDrive(double leftStick, double rightStick, double leftTrigger, double rightTrigger, boolean invert, boolean brake) {
        double i = invert ? -0.75:0.75;
        if (leftTrigger > .3) {
            drive(MovementEnum.LEFTSTRAFE, leftTrigger * i);
            return;
        }
        if (rightTrigger > .3) {
            drive(MovementEnum.RIGHTSTRAFE, rightTrigger *i);
            return;
        }
        leftStick *= i;
        rightStick *= i;

        FL.setPower(-leftStick);
        FR.setPower(-rightStick);
        BL.setPower(-leftStick);
        BR.setPower(-rightStick);
    }
    public void setPower(double power){
        FL.setPower(power);
        BL.setPower(power);
        FR.setPower(power);
        BR.setPower(power);
    }
    public void drive(MovementEnum movement, double power) {
        switch (movement) {
            case FORWARD:
                FL.setPower(power);
                FR.setPower(power);
                BL.setPower(power);
                BR.setPower(power);
                break;

            case BACKWARD:
                FL.setPower(-power);
                FR.setPower(-power);
                BL.setPower(-power);
                BR.setPower(-power);
                break;

            case LEFTSTRAFE:
                FL.setPower(power);
                FR.setPower(-power);
                BL.setPower(-power);
                BR.setPower(power);
                break;

            case RIGHTSTRAFE:
                FL.setPower(-power);
                FR.setPower(power);
                BL.setPower(power);
                BR.setPower(-power);
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
    public void turn(double in){
        BL.setPower(-in);
        BR.setPower(in);
        FR.setPower(in);
        FL.setPower(-in);
    }
}
