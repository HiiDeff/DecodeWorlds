package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.RobotBase;
import org.firstinspires.ftc.teamcode.drive.RobotFactory;
import org.firstinspires.ftc.teamcode.drive.SensorUpdateThread;
import org.firstinspires.ftc.teamcode.task.KickerTask;
import org.firstinspires.ftc.teamcode.task.Presets;
import org.firstinspires.ftc.teamcode.task.Task;
import org.firstinspires.ftc.teamcode.util.GamePad;
@Config
public abstract class Robot1TeleOpForNeel extends LinearOpMode {
    public static int FLYWHEEL_RPM = 2500, MANUAL_OVERRIDE_FLYWHEEL_RPM = 2500;
    public static double MANUAL_OVERRIDE_PIVOT_POS = 0.56, INTAKE_POWER = 0.5, PUSHER_INTAKE_POWER = 0.2, PUSHER_OUTTAKE_POWER = 0.1, SERVO_SKIP_CORRECTION = 0.0;
    public static boolean kickerUp, aiming, autoaim = true; // when autoaim is false use manual override
    private Task task;
    public MultipleTelemetry multipleTelemetry;
    private SensorUpdateThread sensorUpdateThread;
    public GamePad gp1, gp2;
    private RobotBase robot;

    @Override
    public void runOpMode() throws InterruptedException {
        multipleTelemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = (RobotBase) RobotFactory.createRobot(hardwareMap);
        robot.teleOpInit();
        robot.setStartingPose(new Pose());
        gp1 = new GamePad(gamepad1);
        gp2 = new GamePad(gamepad2);
        sensorUpdateThread = new SensorUpdateThread(robot);

        waitForStart();

        robot.startTeleopDrive();
        robot.startLimelight();
        robot.setLimelightAllianceColor(isRed());
        sensorUpdateThread.start();

        while (opModeIsActive()) {
            update();
            updateAiming();
            updateIntake();
            updateShooting();
            updateDrive();
            updateUnjamming();
        }

        sensorUpdateThread.interrupt();
        robot.stopLimelight();
    }

    private void update() {
        robot.updateEverything();
        if(task != null && task.perform()) task = null;

        if(gp1.back() && gp1.onceY()) {
            autoaim = !autoaim;
        }

        gp1.update();
        gp2.update();

        multipleTelemetry.addData("dist to goal", robot.getDistToGoalInches());
        multipleTelemetry.addData("robot pos", robot.getPose().getX()+" "+robot.getPose().getY()+" "+robot.getPose().getHeading());
        multipleTelemetry.addData("current rpm", robot.getFlywheelVelocityRpm());
        multipleTelemetry.addData("target rpm", FLYWHEEL_RPM);
        multipleTelemetry.update();
    }

    private void updateShooting() {
        if(!autoaim||aiming && task == null) {
            if(gp1.onceA() && robot.flywheelPID.isDone()) {
                task = Presets.createShootTask(robot, FLYWHEEL_RPM);
            }
        } if(!autoaim) {
            if(gp1.onceA()) {
                kickerUp = !kickerUp;
                robot.setKickerPower(kickerUp? KickerTask.Direction.STANDARD :KickerTask.Direction.REVERSED);
            }
        }
    }

    private void updateIntake() {
        //safer? no jamming?
//        if(gp2.rightTrigger()>0.3) {
//            robot.runIntake();
//            if(gp2.rightBumper()) {
//                robot.runPusher(0.5);
//            } else {
//                robot.stopPusher();
//            }
//        } else if(gp2.rightBumper()) {
//            robot.stopIntake();
//            robot.runPusher();
//        } else {
//            robot.stopIntake();
//            robot.runPusherReversed();
//        }
        if(gp2.rightTrigger()>0.3) {
            robot.runIntakeWithPower(INTAKE_POWER);
//            if(robot.hasArtifact()) {
//                robot.runPusherReversed();
//            } else {
//                robot.runPusher(PUSHER_INTAKE_POWER);
//            }
        } else if(gp2.rightBumper()) {
            robot.stopIntake();
//            robot.stopPusher();
        } else if(gp2.leftTrigger()>0.3) {
//            robot.runPusherReversed();
            robot.runIntakeReversed();
        }
        else {
            robot.stopIntake();
//            robot.runPusher(-PUSHER_OUTTAKE_POWER);
        }
    }

    private void updateAiming() {
        if(autoaim) {
            if(gp1.onceX()) {
                aiming = !aiming;
                if(!aiming) {
                    robot.startTeleopDrive();
                    robot.setFlywheelTargetVelocity(0);
                } else {
                    robot.holdPoint(new BezierPoint(robot.getPose()), robot.getAngleToGoal(), false);
                    robot.setFlywheelTargetVelocity(FLYWHEEL_RPM);
                }
            }
            if(task==null) robot.setPivotPosition(calcPivotPosition(robot.getDistToGoalInches()));
            FLYWHEEL_RPM = calcFlywheelRpm(robot.getDistToGoalInches());
        } else {
            robot.setPivotPosition(MANUAL_OVERRIDE_PIVOT_POS);
            robot.setFlywheelTargetVelocity(MANUAL_OVERRIDE_FLYWHEEL_RPM);
        }
    }

    private void updateDrive() {
        double x = 0, y = 0, a = 0;
        if (gp1.dpadLeft() || gp1.dpadRight()) {
            a = 0.5 * (gp1.dpadLeft() ? 1 : -1);
        } else if (gp1.dpadUp() || gp1.dpadDown()) {
            x = 0.3 * (gp1.dpadUp() ? 1 : -1);
        } else {
            x = -gp1.leftStickY();
            y = -gp1.leftStickX();
            a = -gp1.rightStickX() * 0.5;
        }
        double pow = Math.sqrt(x * x + y * y);

        if(pow>0.5) {
            if(aiming) {
                aiming = false;
                if(task != null) {
                    task.cancel();
                    task = null;
                }
//                robot.setKickerPosition(KickerTask.Position.DOWN); kickerUp = false;
                robot.setFlywheelTargetVelocity(0);
                robot.startTeleopDrive();
            }
        }

        if(Math.abs(a)<0.05 && !aiming) robot.updateRobotPoseUsingLimelight();
        if(!aiming) robot.setTeleOpDrive(x, y, a, true);
    }
    private void updateUnjamming() {
        if(gp1.back() && gp1.onceLeftBumper()) {
            if(aiming) {
                aiming = false;
                if(task != null) {
                    task.cancel();
                    task = null;
                }
//                robot.setKickerPosition(KickerTask.Position.DOWN); kickerUp = false;
                robot.setFlywheelTargetVelocity(0);
                robot.startTeleopDrive();
            }
            task = Presets.createUnjammingTask(robot);
        }
    }

    //regression
    private double calcPivotPosition(double distToGoalInches) {
        double x = distToGoalInches;
        if(distToGoalInches<60) {
            //regression
            return -0.00003731*Math.pow(x,2)-0.000944536*x+0.591796;
        } else if(distToGoalInches<80) {
            return -0.08/20*(x-60) + 0.44;
        } else {
            return 0.31;
        }
    }
    private int calcFlywheelRpm(double distToGoalInches) {
        if(distToGoalInches<60) return 2500;
        else if(distToGoalInches<80) return 2750;
        else if(distToGoalInches>130) return 3700;
        return (int)(1000.0/143*(distToGoalInches-7)+2500);
    }

    protected abstract boolean isRed();
}