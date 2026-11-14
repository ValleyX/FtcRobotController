package org.firstinspires.ftc.team2844simple.teleops.testing;

import com.pedropathing.math.Pose;
import com.vcs.valleylib.core.command.Command;
import com.vcs.valleylib.core.command.Commands;
import com.vcs.valleylib.core.command.InstantCommand;
import com.vcs.valleylib.core.command.decorators.ParallelCommandGroup;
import com.vcs.valleylib.core.fsm.StateMachine;
import com.vcs.valleylib.ftc.input.CommandGamepad;
import com.vcs.valleylib.ftc.input.Trigger;

import org.firstinspires.ftc.team2844simple.autos.AutoPaths;
import org.firstinspires.ftc.team2844simple.commands.drive.GoToCmd;
import org.firstinspires.ftc.team2844simple.helpers.Constants;
import org.firstinspires.ftc.team2844simple.helpers.PIDF;
import org.firstinspires.ftc.team2844simple.helpers.Subsystems;
import org.firstinspires.ftc.team2844simple.subsystems.ShooterSubsystem;

public class TestingContainer {
    public enum Cycle {DRIVE, PIDF, SHOOTING}
    boolean nextState = false;
    boolean previousState = false;
    double velocity;
    int velocityMultiplier, pidfMultiplier;
    double[] velocityArray, pidfArray;

    int currentPIDF;
    Trigger leftStickRight, leftStickLeft, leftStickUp, leftStickDown,
            rightStickLeft, rightStickRight;
    StateMachine<Cycle> testingMachine;
    Subsystems subsystems;
    Pose resetPose;

    /* ---------- Controller and Buttons ---------- */
    CommandGamepad driver;
    CommandGamepad operator;
    /*
    * Used Buttons on the Driver Controller (gamepad 1)
    * Bumpers: Switch to next and previous state
    * Left and Right Triggers: Throughput and Intake or Outtake
    * X, B: Change Selected PIDF Value
    * Y: Open/Close Gate
    * DPad: increase/decrease velocity; autoPark
    * Guide: reset pose
    *
    * Used Sticks on the Operator Controller (gamepad 2)
    * Right Stick X: Multiplier for Velocity
    * Left Stick: PIDF multiplier and increase/decrease
    * */
    public TestingContainer(Subsystems subsystems, CommandGamepad gamepad1, CommandGamepad gamepad2){
        /* --- Set the Defaults --- */
        this.subsystems = subsystems;
        driver = gamepad1;
        operator = gamepad2;
        velocity = Constants.SHOOT_RPM;
        currentPIDF = 0;


        resetPose = (Constants.alliance == Constants.BLUE_ALLIANCE)
                ?new Pose(144.0-(Constants.ROBOT_LENGTH/2.0), Constants.ROBOT_WIDTH/2.0, Math.toRadians(180.0))
                :new Pose(Constants.ROBOT_LENGTH/2.0, (144-Constants.ROBOT_WIDTH/2.0), Math.toRadians(0.0));

        /* --- Arrays and Multipliers --- */
        velocityMultiplier = 0;
        pidfMultiplier = 0;
        velocityArray = new double[]{10, 50, 100, 500};
        pidfArray = new double[]{0.005, 0.01, 0.05, 0.1, 0.5, 1.0, 5.0, 10.0, 50.0, 100.0, 1000.0};

        /* --- Left Stick Triggers --- */
        leftStickLeft = new Trigger(() -> operator.leftX()<= -0.75);
        leftStickRight = new Trigger(() -> operator.leftX() >= 0.75);
        leftStickDown = new Trigger(() -> operator.leftY()>= 0.75);
        leftStickUp = new Trigger(() -> operator.leftY() <= -0.75);


        /* --- Right Stick Triggers --- */
        rightStickLeft = new Trigger(() -> operator.rightX()<= -0.75);
        rightStickRight = new Trigger(() -> operator.rightX()>= 0.75);


        /* --- State Machine --- */
        testingMachine = new StateMachine<>(Cycle.DRIVE)
                //!DRIVE state
                .state(Cycle.DRIVE, new InstantCommand(()-> {}))

                //!PIDF State
                .onEnter(Cycle.PIDF, ()-> {
                    subsystems.pollen.idleShoot();
                })
                .state(Cycle.PIDF, Commands.run(()-> {
                    subsystems.pollen.shoot(velocity);
                }))
                .onExit(Cycle.PIDF, ()-> {
                    subsystems.pollen.shoot(0.0);
                })

                //!LINREG State
                .onEnter(Cycle.SHOOTING, ()-> {
                    subsystems.pollen.idleShoot();
                })
                .state(Cycle.SHOOTING, Commands.run(()-> {
                    subsystems.pollen.shoot(velocity);
                }))
                .onExit(Cycle.SHOOTING, ()-> {
                    subsystems.pollen.shoot(0.0);
                })

                //?Transition forward
                .transition(Cycle.DRIVE,  Cycle.PIDF, () -> nextState)
                .transition(Cycle.PIDF, Cycle.SHOOTING, () -> nextState)

                .onTransition((from, to) -> nextState = false)

                //?Transition backward
                .transition(Cycle.PIDF,  Cycle.DRIVE, () -> previousState)
                .transition(Cycle.SHOOTING, Cycle.PIDF, () -> previousState)

                .onTransition((from, to) -> previousState = false);

        configureBindings();
    }

    public void configureBindings(){
        configureDefaults();
        configureDRIVE();
        configurePIDF();
        configureSHOOTING();
    }

    public void configureDefaults(){
        subsystems.drive.setDefaultCommand(subsystems.drive.run(
                () -> subsystems.drive.driveFieldCentric(
                        -driver.leftY(),
                        -driver.leftX(),
                        -driver.rightX()
                )
        ));
        driver.rightBumper().onTrue(new InstantCommand(() -> nextState = true));
        driver.leftBumper().onTrue(new InstantCommand(() -> previousState = true));

        //! Resets the pose to the corner near the Human Player
        driver.guide().onTrue(new InstantCommand(() -> subsystems.drive.setPose(resetPose)));
    }

    public void configureDRIVE(){
        when(Cycle.DRIVE, driver.dpadDown()).whileTrue(
                new GoToCmd(subsystems.drive,
                        () -> AutoPaths.routeToPark(AutoPaths.getField(), subsystems.drive.getPose()), 0.8)
                        .andThen(Commands.run(() -> {}, subsystems.drive))
        );
    }

    public void configurePIDF(){
        configureVelocity(Cycle.PIDF);
        configureShooting(Cycle.PIDF);
        configureMultipliers(Cycle.PIDF);

        when(Cycle.PIDF, leftStickUp).onTrue(new InstantCommand(()-> addToPIDF(currentPIDF, pidfArray[pidfMultiplier], subsystems.pollen))); //increase
        when(Cycle.PIDF, leftStickDown).onTrue(new InstantCommand(()-> subtractFromPIDF(currentPIDF, pidfArray[pidfMultiplier], subsystems.pollen))); //decrease
        when(Cycle.PIDF, leftStickRight).onTrue(new InstantCommand(()-> pidfMultiplier = addCapped(pidfMultiplier, pidfArray.length-1))); //Increase multiplier
        when(Cycle.PIDF, leftStickLeft).onTrue(new InstantCommand(()-> pidfMultiplier = subtractCapped(pidfMultiplier))); //Decrease multiplier

        when(Cycle.PIDF, driver.x()).onTrue(new InstantCommand(()-> currentPIDF = subtractCapped(currentPIDF))); //Previous PIDF
        when(Cycle.PIDF, driver.b()).onTrue(new InstantCommand(()-> currentPIDF = addCapped(currentPIDF, 4))); //Next PIDF
    }

    public void configureSHOOTING(){
        configureVelocity(Cycle.SHOOTING);
        configureShooting(Cycle.SHOOTING);
        configureMultipliers(Cycle.SHOOTING);
    }

    public void configureMultipliers(Cycle state){
        /* --- All of the Multiplier Stuff --- */
        Command multiplyVelocity = new InstantCommand(()->
            velocityMultiplier = addCapped(velocityMultiplier, velocityArray.length-1));
        Command divideVelocity = new InstantCommand(()->
            velocityMultiplier = subtractCapped(velocityMultiplier));

        when(state, rightStickLeft).onTrue(divideVelocity);
        when(state, rightStickRight).onTrue(multiplyVelocity);
    }


    /* --- Configure the Velocity --- */
    public void configureVelocity(Cycle state){
        when(state, driver.dpadRight()).onTrue(new InstantCommand(()->velocity+=velocityArray[velocityMultiplier]));
        when(state, driver.dpadLeft()).onTrue(new InstantCommand(()->velocity-=velocityArray[velocityMultiplier]));
    }

    public void configureShooting(Cycle state){
        //?Intake with right trigger, outtake with left trigger
        when(state, driver.rightTriggerButton(0.05)).whileTrue(new ParallelCommandGroup(
                subsystems.intake.startEnd(subsystems.intake::intake, subsystems.intake::stopIntake),
                subsystems.transfer.startEnd(subsystems.transfer::transfer, subsystems.transfer::stop)
        ));
        when(state, driver.leftTriggerButton(0.05)).whileTrue(new ParallelCommandGroup(
                subsystems.intake.startEnd(()->subsystems.intake.intake(-1.0), subsystems.intake::stopIntake),
                subsystems.transfer.startEnd(()->subsystems.transfer.transfer(-1.0), subsystems.transfer::stop)
        ));

        //?Opens the gate when it is closed, and closes when it is open
        when(state, driver.y()).onTrue(new InstantCommand(()->{
            if(subsystems.transfer.areGatesOpen()){
                subsystems.transfer.closeGates();
            } else {
                subsystems.transfer.openGates();
            }
        }));
    }

    public StateMachine<Cycle> getStateMachine(){
        return testingMachine;
    }

    /* --- Private Helpers --- */
    private int addCapped(int adder, int max){
        int temp = adder;
        if(temp+1 <= max){
            temp += 1;
        }
        return temp;
    }

    private int subtractCapped(int subtracter){
        int temp = subtracter;
        if(!(temp-1 < 0)){
            temp -= 1;
        }
        return temp;
    }

    private void addToPIDF(int pidf, double addAmount, ShooterSubsystem shooter){
        PIDF newPIDF = shooter.getPIDF();
        switch(pidf){
            case (0):
                newPIDF.kP += addAmount;
                break;
            case (1):
                newPIDF.kI += addAmount;
                break;
            case (2):
                newPIDF.kD += addAmount;
                break;
            case (3):
                newPIDF.kS += addAmount;
                break;
            case(4):
                newPIDF.kV += addAmount;
                break;
        }
        shooter.changePIDF(newPIDF);
    }

    private void subtractFromPIDF(int pidf, double subtractAmount, ShooterSubsystem shooter){
        PIDF newPIDF = shooter.getPIDF();
        switch(pidf){
            case (0):
                newPIDF.kP -= subtractAmount;
                break;
            case (1):
                newPIDF.kI -= subtractAmount;
                break;
            case (2):
                newPIDF.kD -= subtractAmount;
                break;
            case (3):
                newPIDF.kS -= subtractAmount;
                break;
            case(4):
                newPIDF.kV -= subtractAmount;
                break;
        }
        shooter.changePIDF(newPIDF);
    }

    public String getSelectedPIDF(){
        switch(currentPIDF){
            case (0):
                return "P";
            case (1):
                return "I";
            case (2):
                return "D";
            case (3):
                return "kS";
            case(4):
                return "kV";
            default:
                return "Invalid";
        }
    }

    private Trigger when(Cycle c, Trigger button){
        return button.and(testingMachine.in(c));
    }
}
