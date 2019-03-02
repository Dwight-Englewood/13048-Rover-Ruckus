package org.firstinspires.ftc.teamcode.PIDStuff;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.BoBot;
import org.firstinspires.ftc.teamcode.PIDStuff.PID;

@Disabled
public class PIDController extends PID{
    BoBot robot = new BoBot();

    public PIDController(double pGain, double iGain, double dGain) {
        super(pGain, iGain, dGain);
    }

    public void PIDDrive() {
        double power = this.getPower();
        if (robot.FL.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            robot.changeRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        robot.drivePower(power);
    }

    public void PIDStrafe() {
        double power = this.getPower();
        if (robot.FL.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            robot.changeRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        robot.strafePower(power);
    }

    @Override
    public double correction() {
        return Range.clip(super.correction(), -1, 1);
    }

    private double getPower() {
        double currentPos = (Math.abs(robot.FL.getCurrentPosition()) + Math.abs(robot.FR.getCurrentPosition()) +
                Math.abs(robot.BL.getCurrentPosition()) + Math.abs(robot.BR.getCurrentPosition())) / 4.0;
        double sign = Math.signum(robot.FR.getCurrentPosition());
        this.updateError(currentPos*sign);
        return this.correction();
    }

    @Override
    public void setGoal(double goal) { //please only ever pass an int in to this, like really
        super.setGoal(goal);
        robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}