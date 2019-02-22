package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware.MovementEnum;
import org.firstinspires.ftc.teamcode.Hardware.PID;
import org.firstinspires.ftc.teamcode.TensorFlowStuff.TensorFlow;
import org.firstinspires.ftc.teamcode.Hardware.bot;
import java.util.Random;

@Autonomous(name="TensorFlow owo (It work)", group="Autonomous")
public class AutonTeamMakerWithTensorFlow extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    bot robot = new bot();
    TensorFlow tensorFlow = new TensorFlow();
    int auto = 1;

    int center = 150;
    int left = 550;
    int right = 350;

    TensorFlow.TFState BigThonk, actualState;
    Random rand = new Random();


    public void init() {
        robot.init(hardwareMap, telemetry, false);
        tensorFlow.init(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.FR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.hook.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.claw.setPosition(0.0);
        //hinge.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        tensorFlow.start();
        BigThonk = tensorFlow.getState();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (auto) {
            case 0:
                // BigThonk = tensorFlow.getState();

                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    telemetry.addLine("Sleep Failed");
                    telemetry.update();
                }

                auto++;
                break;

            case 1:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.hook.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.hook.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.claw.setPosition(0.0);
                auto++;
                break;

            case 2:
                robot.hook.setTargetPosition(7055);
                robot.hook.setPower(1);
                robot.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BigThonk = (tensorFlow.getState() == TensorFlow.TFState.NOTVISIBLE) ? BigThonk : tensorFlow.getState();

                if (!robot.hookLimit.getState() || Math.abs(robot.hook.getCurrentPosition()) >= 7055) {
                    //  BigThonk = (BigThonk != TensorFlow.TFState.NOTVISIBLE) ? BigThonk : tensorFlow.getState();
                    robot.hook.setPower(0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 3:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto = 12345;
                break;
            case 12345:
                if (Math.abs(0 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > 3) {
                    robot.adjustHeading(0);
                } else if (Math.abs(0 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 3) {
                    // robot.tankDrive(0, 0, 0, 0, false, false);
                    robot.drive(MovementEnum.STOP, 0);
                    auto++;
                }
                break;
            case 12346:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto = 5;
                break;
            case 4:
                tensorFlow.stop();
                robot.autonDriveUltimate(MovementEnum.BACKWARD, 280 / 2, 0.5);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())) {
                    telemetry.update();
                    auto = 6;
                }

                break;

            case 5:
                BigThonk =  (tensorFlow.getState() == TensorFlow.TFState.NOTVISIBLE) ?  BigThonk: tensorFlow.getState();
                if (BigThonk != TensorFlow.TFState.NOTVISIBLE) {
                    auto = 4;
                }

                break;
            case 6:
                // tensorFlow.stop();
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;


            case 7:
                robot.autonDriveUltimate(MovementEnum.RIGHTSTRAFE, 560 / 2, 0.2);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    telemetry.update();
                    auto++;
                }
                break;
            case 8:
                if (Math.abs(-80 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > 3) {
                    robot.adjustHeading(-80);
                } else if (Math.abs(-80 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 3) {
                    // robot.tankDrive(0, 0, 0, 0, false, false);
                    robot.drive(MovementEnum.STOP, 0);
                    auto++;
                }
                break;
            case 9:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
            case 10:
                robot.autonDriveUltimate(MovementEnum.FORWARD, 1120 / 4, 0.6);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    telemetry.update();
                    auto++;
                }

                break;
            case 11:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
            case 12:
                switch(BigThonk){
                    case LEFT:
                        robot.autonDriveUltimate(MovementEnum.LEFTSTRAFE, 300 + 250, 0.2);
                        if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                            telemetry.update();
                            auto++;
                        }
                        break;
                    case RIGHT:
                        robot.autonDriveUltimate(MovementEnum.RIGHTSTRAFE, 400 - 50, 0.2);
                        if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                            telemetry.update();
                            auto++;
                        }

                        break;
                    case CENTER:
                        robot.autonDriveUltimate(MovementEnum.LEFTSTRAFE, 1120 / 6 -36, 0.2);
                        if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                            telemetry.update();
                            auto++;
                        }

                        break;
                    case NOTVISIBLE:
                        int yeet = rand.nextInt(2)+1;
                        if (yeet == 1 ){BigThonk = TensorFlow.TFState.CENTER;}
                        else if(yeet ==2){BigThonk = TensorFlow.TFState.RIGHT;}
                        else {BigThonk = TensorFlow.TFState.LEFT;}
                        break;
                }
                break;
            case 13:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
            case 14:
                switch(BigThonk){
                    case LEFT:
                        robot.autonDriveUltimate(MovementEnum.FORWARD, 980, 0.3);
                        if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                            telemetry.update();
                            auto++;
                        }

                        break;
                    case RIGHT:
                        robot.autonDriveUltimate(MovementEnum.FORWARD, 840  , 0.3);
                        if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                            telemetry.update();
                            auto++;
                        }
                        break;
                    case CENTER:
                        robot.autonDriveUltimate(MovementEnum.FORWARD, 840  , 0.3);
                        if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                            telemetry.update();
                            auto++;
                        }
                        break;
                    case NOTVISIBLE:
                        break;

                }
                break;
            case 15:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
            case 16:
                robot.autonDriveUltimate(MovementEnum.BACKWARD, 840 , 0.4);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    telemetry.update();
                    auto++;
                }
                break;
            case 17:
                if (Math.abs(0 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) > 3) {
                    robot.adjustHeading(0);
                } else if (Math.abs(0 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) < 3) {
                    // robot.tankDrive(0, 0, 0, 0, false, false);
                    robot.drive(MovementEnum.STOP, 0);
                    auto++;
                }
                break;
            case 18:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
            case 19:
                switch (BigThonk){
                    case LEFT:
                        robot.autonDriveUltimate(MovementEnum.FORWARD, 1200 , 0.4);
                        if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                            telemetry.update();
                            auto++;
                        }
                        break;
                    case RIGHT:
                        robot.autonDriveUltimate(MovementEnum.FORWARD, 1820 , 0.4);
                        if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                            telemetry.update();
                            auto++;
                        }
                        break;
                    case CENTER:
                        robot.autonDriveUltimate(MovementEnum.FORWARD, 1960 , 0.4);
                        if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                            telemetry.update();
                            auto++;
                        }
                        break;
                    case NOTVISIBLE:
                        BigThonk = TensorFlow.TFState.LEFT;
                        break;
                }
                break;
            case 20:
                if(Math.abs(45 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                    robot.adjustHeading(45);
                }
                else if(Math.abs(45 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3){
                    // robot.tankDrive(0, 0, 0, 0, false, false);
                    robot.drive(MovementEnum.STOP,0);
                    // auto = 313;
                    auto++;
                }
                break;
            case 21:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
            case 22:
                robot.autonDriveUltimate(MovementEnum.BACKWARD, 1500, 0.4);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    telemetry.update();
                    auto++;
                }
                break;
            case 23:
                robot.claw.setPosition(0.7);
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if(robot.claw.getPosition() >= 0.7) {

                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                        telemetry.addLine("Sleep Failed");
                        telemetry.update();
                    }
                    auto++;
                }
                break;

            case 24:
                robot.autonDriveUltimate(MovementEnum.RIGHTSTRAFE, 560  , 0.3);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    telemetry.update();
                    auto++;
                }
                break;
            case 25:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
            case 26:
                robot.autonDriveUltimate(MovementEnum.FORWARD, 2100 , 0.2);
                if (Math.abs(robot.FL.getCurrentPosition()) >= Math.abs(robot.FL.getTargetPosition())){
                    telemetry.update();
                    //  auto++;
                }
                break;




        }
        telemetry.addData("Team Marker Position", robot.claw.getPosition());
        telemetry.addData("Position", tensorFlow.getState());
        telemetry.addData("BiggieThonk", BigThonk);
        telemetry.addData("Case Number: ", auto);
        telemetry.addData("YET", robot.hook.getPower());
        telemetry.addData("twtr", robot.hook.getCurrentPosition());
        telemetry.addData("EWT",robot.hookLimit.getState());
        telemetry.addData("Degrees: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("Difference: ", Math.abs(90 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ));
        telemetry.update();
    }
}
