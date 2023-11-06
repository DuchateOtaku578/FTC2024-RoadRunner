package org.firstinspires.ftc.teamcode.teleOpMaster;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.test.motor.posicion.PruebaMotoresPosicion;
import org.firstinspires.ftc.teamcode.test.motor.posicion.PruebaMotoresPosicionConfig;


@TeleOp(name="TeleOpMaster", group="Pushbot")

public class TeleOpMaster extends LinearOpMode {

    PruebaMotoresPosicionConfig robot = new PruebaMotoresPosicionConfig();
    PruebaMotoresPosicion elevador = new PruebaMotoresPosicion();
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();


        while (opModeIsActive()) {

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);
            telemetry.update();


            if (gamepad1.y && robot.motor.getCurrentPosition() <= 2900){
                usingEncoder();
                robot.motor.setPower(1);
                telemetry.addLine("Arriba");
                telemetry.addData("Puslos: ", robot.motor.getCurrentPosition());
                telemetry.addData("gamepad y: ", gamepad1.y);
            } else if (gamepad1.a && robot.motor.getCurrentPosition() >= 0) {
                usingEncoder();
                robot.motor.setPower(-1);
                telemetry.addLine("Abajo");
                telemetry.addData("Puslos: ", robot.motor.getCurrentPosition());
                telemetry.addData("gamepad a: ", gamepad1.a);
            } else {
                mantenerse();
                telemetry.addLine("Manteniendo");
                telemetry.addData("Puslos: ", robot.motor.getCurrentPosition());
            }









        }
    }
    public void usingEncoder(){
        robot.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetearEncoder(){
        robot.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setTarget(int distancia){
        robot.motor.setTargetPosition(distancia);
    }

    public void setRunToPosition(){
        robot.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void activarMotor(int potencia){
        robot.motor.setPower(potencia);
    }

    public void mantenerse (){

        robot.motor.setTargetPosition(robot.motor.getCurrentPosition());

        robot.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motor.setPower(1);

    }

    public void moverseDistanciaMantener(double potencia , int distance) {
        robot.motor.setTargetPosition(distance);
        telemetry.addLine("Set target position");
        telemetry.update();

        robot.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addLine("Run to position");
        telemetry.update();

        moverseEnfrente(potencia);
        telemetry.addLine("Moverse Enfrente");
        telemetry.update();

        while(robot.motor.isBusy()){
            telemetry.addLine("Dentro del Busy");
            telemetry.update();
        }

    }

    public void moverseEnfrente(double potencia){
        robot.motor.setPower(potencia);
    }

    public void pararMotores(){
        robot.motor.setPower(0);
    }

}







