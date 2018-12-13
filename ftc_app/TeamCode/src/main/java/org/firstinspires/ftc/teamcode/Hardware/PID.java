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
        motorSpeed();
        robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.drive(MovementEnum.STOP, 0);
        robot.tele.update();
    }

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
