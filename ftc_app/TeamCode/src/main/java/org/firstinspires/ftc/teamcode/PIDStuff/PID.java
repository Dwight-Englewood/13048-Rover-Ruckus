package org.firstinspires.ftc.teamcode.PIDStuff;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.math.RoundingMode;
import java.text.DecimalFormat;

@Disabled
public class PID {
    public double proportionalGain;
    public double integralGain;
    public double derivativeGain;

    private double goal;
    public double error;

    private double lX;
    private double dError;
    private double iError;

    public PID(double pGain, double iGain, double dGain) {
        this.proportionalGain = pGain;
        this.integralGain = iGain;
        this.derivativeGain = dGain;
    }

    public PID(double pGain, double iGain, double dGain, double goal) {
        this.proportionalGain = pGain;
        this.integralGain = iGain;
        this.derivativeGain = dGain;
        this.goal = goal;
    }

    public static void main(String[] args) {
        int merp = 0;
        DecimalFormat df = new DecimalFormat("#.####");
        df.setRoundingMode(RoundingMode.CEILING);
        PID pid = new PID(.3, 0, 0);
        //PIDController pid = new PIDController(.6, 0, 2.5);
        //PIDController pid = new PIDController(.4, .001, 6.5);

        double currentPosition = 10;
        double currentVelocity = 0;
        double currentAcceleration = 0;
        final double mass = 100;
        final double gravity = 9.8;
        pid.setGoal(0);

        /*for (int i=0; i < 2000; i++) {
            pid.updateError(currentPosition);
            double force = Range.clip(pid.correction(), -1, 1)*5000;
            currentAcceleration = force/mass - ((10/mass) * currentVelocity)-gravity;
            currentVelocity = (currentVelocity + currentAcceleration / 100);
            currentPosition = (currentPosition + currentVelocity / 100);
            if (i == 150) {
                merp = 1;
            }
            System.out.println(df.format(currentPosition));
        }*/
    }

    public double correction() {
        return ((error * proportionalGain) + (iError * integralGain) - (dError * derivativeGain));
    }

    public void updateError(double currentPosition) {
        this.error = goal - currentPosition;
        this.iError = this.iError + this.error;
        this.dError = currentPosition - this.lX;
        this.lX = currentPosition;
    }

    public void setGoal(double goal) {
        this.reset();
        this.goal = goal;
    }

    protected void setGoalNoReset(double goal) {
        this.goal = goal;
    }

    public boolean goalReached(double resolution) {
        return Math.abs(this.error) < resolution;
    }

    public void reset() {
        this.goal = 0;
        this.error = 0;
        this.lX = 0;
        this.dError = 0;
        this.iError = 0;
    }
}
    /*
   public bot robot = new bot();

    final double proportionalValue = 0.00015;

    public void pidutonDrive(MovementEnum movementEnum, int target) {
        robot.autonDrive(movementEnum, target);
        this.motorSpeed();
        robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.drive(MovementEnum.STOP, 0);
        robot.tele.update();
    }
*/
    /**Description:
     * robot.FL.setPower((Math.abs(robot.FL.getTargetPosition()) - Math.abs(robot.FL.getCurrentPosition()) * proportionalValue));
     * robot.FL.setPower(); sets the power using the variables within the brackets ().
     * Math.abs(robot.FL.getTargetPosition()) -  Math.abs(robot.FL.getCurrentPosition()); subtracts encoder values to eventually reach 0.
     * The amount from the top times the proportionalValue, which is 0.00015, gives the power.
     * When target position of 100 - current position of 0, times the proportional value, the speed is 0.015, or 1.5% of the motor's power
     */
/*
    public void motorSpeed() {
        if (Math.abs(bot.FL.getCurrentPosition()) < Math.abs(robot.FL.getTargetPosition())) {
            robot.FL.setPower((Math.abs(robot.FL.getTargetPosition()) - Math.abs(robot.FL.getCurrentPosition()) * proportionalValue));
            robot.FR.setPower((Math.abs(robot.FR.getTargetPosition()) - Math.abs(robot.FR.getCurrentPosition()) * proportionalValue));
            robot.BL.setPower((Math.abs(robot.BL.getTargetPosition()) - Math.abs(robot.BL.getCurrentPosition()) * proportionalValue));
            robot.BR.setPower((Math.abs(robot.BR.getTargetPosition()) - Math.abs(robot.BR.getCurrentPosition()) * proportionalValue));
        } else {
            robot.autonDrive(MovementEnum.STOP, 0);
        }
    }
}
*/

