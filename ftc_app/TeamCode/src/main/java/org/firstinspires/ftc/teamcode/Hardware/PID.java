package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Hardware.MovementEnum;
import org.firstinspires.ftc.teamcode.Hardware.bot;

public class PID {
   public bot robot = new bot();

    final double proportionalValue = 0.00015;

    public void pidutonDrive(MovementEnum movementEnum, int target) {
        robot.autonDrive(movementEnum, target);
        this.motorSpeed();
        robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.drive(MovementEnum.STOP, 0);
        robot.tele.update();
    }

    /**Description:
     * robot.FL.setPower((Math.abs(robot.FL.getTargetPosition()) - Math.abs(robot.FL.getCurrentPosition()) * proportionalValue));
     * robot.FL.setPower(); sets the power using the variables within the brackets ().
     * Math.abs(robot.FL.getTargetPosition()) -  Math.abs(robot.FL.getCurrentPosition()); subtracts encoder values to eventually reach 0.
     * The amount from the top times the proportionalValue, which is 0.00015, gives the power.
     * When target position of 100 - current position of 0, times the proportional value, the speed is 0.015, or 1.5% of the motor's power
     */

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
