package org.firstinspires.ftc.teamcode.autonomos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configuracion.RobotConfigMaster;
import org.firstinspires.ftc.teamcode.domain.Chasis;
//@Disabled
@Autonomous(name="ChasisAutonomo2", group="Pushbot")

public class ChasisAutonomo2 extends LinearOpMode {

   RobotConfigMaster robot = new RobotConfigMaster();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        Chasis chasis = new Chasis(robot.enfrenteDer, robot.enfrenteIzq, robot.atrasDer, robot.atrasIzq);


        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            chasis.moverseIzquierda(0.9);

            chasis.parar();
            sleep(300);

            chasis.moverseEnfrente(0.5, -250);

        }
    }
}
