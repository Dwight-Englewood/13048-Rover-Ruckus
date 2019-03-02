package org.firstinspires.ftc.teamcode.PIDStuff;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware.BoBot;
import org.firstinspires.ftc.teamcode.PIDStuff.PID;

@Disabled
public class PIDGyro extends PID {
    BNO055IMU imu;
    BoBot robot = new BoBot();

    public PIDGyro(double pGain, double iGain, double dGain, BNO055IMU imu) {
        super(pGain, iGain, dGain);
        this.imu = imu;
    }

    public void PIDTurn(){
        double pow = this.getPower();if (robot.FL.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            robot.changeRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        this.robot.turnPower(pow);
    }

    public double correction() {
        return Range.clip(super.correction(), -1, 1);
    }

    private double getPower() {
        this.updateError(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        return this.correction();
    }

    @Override
    public void setGoal(double goal) { //please only ever pass an int in to this, like really
        super.setGoal(goal);
        this.reset();
    }
    @Override
    public void reset() {
        //super.reset();
        robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}