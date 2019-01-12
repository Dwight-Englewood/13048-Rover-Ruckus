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

@Autonomous(name="TensorFLow :O ", group="Autonomous")
public class AutonTeamMakerWithTensorFlow extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel DigChannel;
    bot robot = new bot();
    TensorFlow tensorFlow = new TensorFlow();

    TensorFlow.TFState BigThonk, actualState;

    int auto = 1;
    int turned = 0;
    public void init() {
        robot.init(hardwareMap, telemetry, false);
        tensorFlow.init(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.BR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.BL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.FL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.FR.setDirection(DcMotorSimple.Direction.REVERSE);

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
                }
                catch (InterruptedException e){
                    telemetry.addLine("Sleep Failed");
                    telemetry.update();
                }

                auto++;
                break;

            case 1:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.claw.setPosition(0.0);
                auto++;
                break;

            case 2:
                robot.hook.setTargetPosition(50000);
                robot.hook.setPower(0.5);
                robot.hook.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BigThonk = (BigThonk != TensorFlow.TFState.NOTVISIBLE) ? BigThonk : tensorFlow.getState();

                if(!robot.hookLimit.getState()){
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
                if(Math.abs(0- robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                    robot.adjustHeading(0);
                }
                else if(Math.abs(0 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                    // robot.tankDrive(0, 0, 0, 0, false, false);
                    robot.drive(MovementEnum.STOP, 0);
                    auto++;
                }
                break;
            case 12346:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto = 4;
                break;
            case 4:
                robot.autonDrive(MovementEnum.BACKWARD, 280 / 2);
                robot.setPower(0.5);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(robot.BR.getCurrentPosition() <= -280 / 2){
                    robot.drive(MovementEnum.STOP, 0);
                    telemetry.update();
                    auto++;
                }
                break;

            case 5:
                BigThonk = tensorFlow.getState();
                    if(BigThonk != TensorFlow.TFState.NOTVISIBLE){auto++;}
                break;
            case 6:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;


            case 7:
                robot.autonDrive(MovementEnum.RIGHTSTRAFE, 1120 / 4);
                robot.setPower(0.2);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(robot.BR.getCurrentPosition() >= 1120 / 4) {
                    robot.drive(MovementEnum.STOP, 0);
                    telemetry.update();
                    auto++;
                }
                break;
            case 8:
                if(Math.abs(-80- robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                    robot.adjustHeading(-80);
                }
                else if(Math.abs(-80 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
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
                robot.autonDrive(MovementEnum.FORWARD, 1120 / 4);
                robot.setPower(0.6);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(robot.BR.getCurrentPosition() >= 1120 / 4) {
                    robot.drive(MovementEnum.STOP, 0);
                    telemetry.update();
                    auto++;
                }
                break;
            case 11:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              /*
                try {
                    Thread.sleep(2000);
                }
                catch (InterruptedException e){
                    telemetry.addLine("Sleep Failed");
                    telemetry.update();
                }
*/
                if(BigThonk == TensorFlow.TFState.CENTER){
                    auto++;
                }else if(BigThonk == TensorFlow.TFState.LEFT){
                    auto = 13;
                }else if(BigThonk == TensorFlow.TFState.RIGHT){
                    auto = 14;
                }
                break;
           /*

           CASE FOR CENTER
           */
            case 12:
                robot.autonDrive(MovementEnum.LEFTSTRAFE, 1120 / 6);
                robot.setPower(0.5);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(Math.abs(robot.BR.getCurrentPosition()) >= 1120 / 6) {
                    robot.drive(MovementEnum.STOP, 0);
                    auto = 15;
                    telemetry.update();

                }
                break;
                /*

           CASE FOR LEFT
           */
            case 13:
                robot.autonDrive(MovementEnum.LEFTSTRAFE, 1120/2);
                robot.setPower(0.2);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(robot.BR.getCurrentPosition() >= 1120 / 2) {
                    robot.drive(MovementEnum.STOP, 0);
                    telemetry.update();
                    // auto++;
                }
                break;
                /*

           CASE FOR RIGHT
           */
            case 14:
                break;
            case 15:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
            case 16:
                robot.autonDrive(MovementEnum.FORWARD, 2500/2);
                robot.setPower(0.4);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(robot.BR.getCurrentPosition() >= 2500 / 2) {
                    robot.drive(MovementEnum.STOP, 0);
                    telemetry.update();
                     auto++;
                }
                break;
            case 17:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
            case 18:
                robot.autonDrive(MovementEnum.BACKWARD, 2500/ 4);
                robot.setPower(0.3);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(Math.abs(robot.BR.getCurrentPosition()) >= 2500 / 4) {
                    auto++;
                }
                break;
            case 19:
                if(Math.abs(90- robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                    robot.adjustHeading(90);
                }
                else if(Math.abs(90 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                    // robot.tankDrive(0, 0, 0, 0, false, false);
                    robot.drive(MovementEnum.STOP, 0);
                    auto++;
                }
                break;
            case 20:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
            case 21:
                robot.autonDrive(MovementEnum.BACKWARD, 2500/ 4);
                robot.setPower(0.3);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(Math.abs(robot.BR.getCurrentPosition()) >= 2500 / 4) {
                    auto++;
                }
                break;
            case 22:
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
            case 23:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
            case 24:
                if(Math.abs(40- robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) > 3) {
                    robot.adjustHeading(40);
                }
                else if(Math.abs(40 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ) < 3) {
                    // robot.tankDrive(0, 0, 0, 0, false, false);
                    robot.drive(MovementEnum.STOP, 0);
                    auto++;
                }
                break;
            case 25:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
            case 26:
                robot.autonDrive(MovementEnum.FORWARD, 6720 / 2);
                robot.setPower(0.3);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);

                if(robot.BR.getCurrentPosition() >= 6720 / 2) {
                    robot.drive(MovementEnum.STOP, 0);
                    telemetry.update();
               //     auto++;
                }
                break;
            case 27:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
            case 28:
                robot.autonDrive(MovementEnum.RIGHTSTRAFE, 1120/16);
                robot.setPower(0.3);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(robot.BR.getCurrentPosition() >= 1120/16){
                    robot.drive(MovementEnum.STOP,0);
                    telemetry.update();
                    auto++;
                }
                break;
            case 29:
                robot.changeRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                auto++;
                break;
            case 30:
                robot.autonDrive(MovementEnum.FORWARD,6720 / 3);
                robot.setPower(0.3);
                robot.changeRunMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(robot.BR.getCurrentPosition() >= 6720 / 3){
                    robot.drive(MovementEnum.STOP,0);
                    telemetry.update();
                   // auto++;
                }
                break;


        }
        telemetry.addData("Team Marker Position", robot.claw.getPosition());
        telemetry.addData("Position", tensorFlow.getState());
        telemetry.addData("BiggieThonk", BigThonk);
        telemetry.addData("Case Number: ", auto);
        telemetry.addData("BR POSITION", robot.BR.getCurrentPosition());
        telemetry.addData("Degrees: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("Difference: ", Math.abs(90 - robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ));
        telemetry.update();
    }
}
