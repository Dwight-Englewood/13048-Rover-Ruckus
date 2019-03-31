package org.firstinspires.ftc.teamcode.EngineeringExpo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class botExpo {
    public static DcMotor BL, BR, FL, FR, FlyL, FlyR;
    HardwareMap map;
    Telemetry tele;

    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        this.map = map;
        this.tele = tele;

//        left = this.map.get(DcMotor.class, "left");
//        right = this.map.get(DcMotor.class, "right");

        BR = this.map.get(DcMotor.class, "BR");
        BL = this.map.get(DcMotor.class, "BL");
        FL = this.map.get(DcMotor.class, "FL");
        FR = this.map.get(DcMotor.class, "FR");
        FlyL = this.map.get(DcMotor.class, "FlyR");
        FlyR = this.map.get(DcMotor.class, "FlyR");

        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FlyL.setDirection(DcMotorSimple.Direction.REVERSE);

        FlyR.setDirection(DcMotorSimple.Direction.FORWARD);


    }
    public void drive(double left, double right){
        FL.setPower(left);
        FR.setPower(right);
        BL.setPower(left);
        BR.setPower(right);

    }
}
