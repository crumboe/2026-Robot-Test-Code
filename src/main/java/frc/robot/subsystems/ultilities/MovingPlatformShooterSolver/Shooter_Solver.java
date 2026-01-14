
package frc.robot.subsystems.ultilities.MovingPlatformShooterSolver;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.simulation.ProjectileVisualizer;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter_Solver extends SubsystemBase {
    // private ProjectileVisualizer visualizer = new ProjectileVisualizer("Algae");
    private  StructArrayPublisher<Translation3d> trajectoryPublisher;
    private  StructArrayPublisher<Translation3d> linePublisher;
    private  StructArrayPublisher<Translation3d> vecPublisher;
    private Supplier<Pose2d> robotPoseGetter;
    private Supplier<Transform2d> robotVelocityGetter;
    private Supplier<Transform3d> robotToShooter;
    private Supplier<Pose3d> targetPoseGetter;
    private Supplier<Double> shooterMuzzleV0Supplier;
    private double RefreshRate = 0.02; // 20 ms
    private double lastTimeSolved = 0.0;
    private double filteredRequiredRobotYawRad = 0.0;
    private int shooter_number;
    private boolean shooting_permisive = false;
    private double shooting_watchdog = 0.0;
    private static final int num_balls_vis = 30;
    private Translation3d[] trajectoryvis = new Translation3d[num_balls_vis];
    private Translation3d[] trajectoryvis_prime = new Translation3d[num_balls_vis];
    private int ball_index = 0;
    private double last_shot_time = 0.0;


    private static class Trajectory {
        public final double angle;
        public final double time;
        public final boolean isValid;
        public final double Yerror;
        public Trajectory(double angle, double time, double Yerror) {
            this.angle = angle;
            this.time = time;
            this.Yerror = Yerror;
            this.isValid = true;
        }   
        @SuppressWarnings("unused")
        public Trajectory(){
            this.angle = 0;
            this.time = 0;
            this.isValid = false;
            this.Yerror = Double.POSITIVE_INFINITY;
        }    
    }
    public static class Solution {
        public final String status;
        public final double requiredRobotYawRad; // Used in kinematics solution
        public final double requiredShooterPitchRad; // Used in kinematics solution
        public final double timeOfFlight; // Used in kinematics solution
        public final double muzzleVelocityCheck; // Used in kinematics solution
        public final Translation3d shooterVelocityCheck; // Used in kinematics solution\
        public final double minYError;
        public double timestamp;
        public Solution(String status, double requiredRobotYawRad, double requiredShooterPitchRad, double timeOfFlight, double muzzleVelocityCheck,Translation3d shooterVelocityCheck, double minYError) {
            this.status = status;
            this.requiredRobotYawRad = requiredRobotYawRad;
            this.requiredShooterPitchRad = requiredShooterPitchRad;
            this.timeOfFlight = timeOfFlight;
            this.muzzleVelocityCheck = muzzleVelocityCheck;
            this.shooterVelocityCheck = shooterVelocityCheck;
            this.minYError = minYError;
            timestamp = Timer.getFPGATimestamp();
        }
    }
    private Solution _best_solution = new Solution("No Solution", 0, 0, 0, 0, new Translation3d(0,0,0),Double.POSITIVE_INFINITY);
    public Shooter_Solver(      int number,
                             Supplier<Pose2d> robotPoseGetter,
                             Supplier<Transform2d> robotVelocityGetter,
                             Supplier<Transform3d> robotToShooter,
                             Supplier<Pose3d> targetPoseGetter,
                             Supplier<Double> shooterMuzzleV0Supplier) {
        
        this.robotPoseGetter = robotPoseGetter;
        this.robotVelocityGetter = robotVelocityGetter;
        this.robotToShooter = robotToShooter;
        this.targetPoseGetter = targetPoseGetter;
        this.shooterMuzzleV0Supplier = shooterMuzzleV0Supplier;
        this.lastTimeSolved = Timer.getFPGATimestamp();
        this.shooter_number = number;

        for(int i =0; i < num_balls_vis; i++){
            trajectoryvis[i] = new Translation3d(0,0,0);
            trajectoryvis_prime[i] = new Translation3d(0,0,0);
        }
        trajectoryPublisher = 
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("trajectoryvis" + number, Translation3d.struct)
            .publish();
        linePublisher = 
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("linevis" + number, Translation3d.struct)
            .publish();
        vecPublisher = 
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("vecvis" + number, Translation3d.struct)
            .publish();

    }
    public static Shooter_Solver getInstance(  int number,
                                        Supplier<Pose2d> robotPoseGetter,
                                        Supplier<Transform2d> robotVelocityGetter,
                                        Supplier<Transform3d> robotToShooter,
                                        Supplier<Pose3d> targetPoseGetter,
                                        Supplier<Double> shooterMuzzleV0Supplier) {
        return new Shooter_Solver(number,robotPoseGetter, robotVelocityGetter, robotToShooter, targetPoseGetter, shooterMuzzleV0Supplier);
    }
    
    private Trajectory solve2DTrajectory(double deltaX, double deltaY, double v0, double projectedVelocityFromRobot) {
        double g = 9.81; // gravity
        double lower = Math.toRadians(45);
        double upper = Math.toRadians(90);
        double tol = Math.toRadians(.5); // stopping tolerance
        double gr = (Math.sqrt(5) - 1) / 2; // golden ratio factor (~0.618)
        
        // Function to compute the absolute y error for a given launch angle
        java.util.function.DoubleFunction<Double> errorFunc = (theta) -> {
            double vx = v0 * Math.cos(theta) + projectedVelocityFromRobot;
            
            if (vx <= 0) return Double.POSITIVE_INFINITY; // invalid case
            double t = deltaX / vx;
            double y_pred = v0 * Math.sin(theta) * t - 0.5 * g * t * t;
            double y_vel = v0 * Math.sin(theta) - g * t;
            // System.out.print(vx + ", " + t + ", " + y_pred + " " + y_vel + "\n");
            if (y_vel > 0) {
                return Double.POSITIVE_INFINITY; // undershot and ascending
            }
            return Math.abs(y_pred - deltaY);
        };
    
        // Initialize interior points
        double c = upper - gr * (upper - lower);
        double d = lower + gr * (upper - lower);
        double fc = errorFunc.apply(c);
        double fd = errorFunc.apply(d);
        int iteration = 0;
        // Iterate until interval is sufficiently small
        while (Math.abs(upper - lower) > tol) {
            if (fc < fd) {
                upper = d;
                d = c;
                fd = fc;
                c = upper - gr * (upper - lower);
                fc = errorFunc.apply(c);
            } else {
                lower = c;
                c = d;
                fc = fd;
                d = lower + gr * (upper - lower);
                fd = errorFunc.apply(d);
            }
            iteration++;
        }
        // System.out.println("Golden Section Iterations: " + iteration);
        // Best angle is the midpoint of final interval
        double bestTheta = (lower + upper) / 2.0;
        double bestError = errorFunc.apply(bestTheta);
        double t_flight = deltaX / (v0 * Math.cos(bestTheta) + projectedVelocityFromRobot);
    
        // // Optionally print for debugging
        // System.out.printf("Best θ = %.2f°, error = %.4f, t = %.3fs%n",
        //                   Math.toDegrees(bestTheta), bestError, t_flight);
    
        return new Trajectory(bestTheta, t_flight,bestError);
    }
  
    private double objectiveFunction(Pose3d shooterPoseWorld,Translation3d shooterVelWorld) {
        
        Pose3d targetPose = this.targetPoseGetter.get();
        // System.out.print(targetPose.getX() + ", " + targetPose.getY() + ", " + targetPose.getZ() + "\n");
        // 1. Compute required effective projectile displacement
        
        double deltaX = targetPose.getX() - shooterPoseWorld.getX() ;
        double deltaY = targetPose.getY() - shooterPoseWorld.getY() ;
        double deltaZ = targetPose.getZ() - shooterPoseWorld.getZ() ;
        
        // System.out.print(deltaX + ", " + deltaY + ", " + deltaZ + "\n");
        // 2. Project to 2D plane
        double dxEff = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double dyEff = deltaZ;
        double projectedVelocityFromRobot = (shooterVelWorld.getX()*deltaX + shooterVelWorld.getY()*deltaY)/dxEff;
        // 3. Solve 2D trajectory
        Trajectory trajectory = solve2DTrajectory(dxEff, dyEff, this.shooterMuzzleV0Supplier.get(),projectedVelocityFromRobot);
        

        // System.out.print(trajectory.angle + ", " + trajectory.time + ", " + trajectory.Yerror + "\n");

        // Calculate full kinematics solution for the best time found so far
        double requiredYawWorld = Math.atan2(deltaY, deltaX);

        // Calculate Projectile World Velocity (V_p_w)
        
        double vpwZ =this.shooterMuzzleV0Supplier.get() * Math.sin(trajectory.angle);

        double vx_mag_required = dxEff / trajectory.time;
        Translation2d vector_xy_required = new Translation2d(vx_mag_required, 0).rotateBy(new Rotation2d(requiredYawWorld));
        Translation2d shooter_xy_vector_required = vector_xy_required.minus(new Translation2d(shooterVelWorld.getX(),
                                                                                        shooterVelWorld.getY()));
       
        double requiredRobotYawRad = shooter_xy_vector_required.getAngle().getRadians();  
        SmartDashboard.putNumber(this.shooter_number+"SS required angle RobotRel", Math.toDegrees(requiredRobotYawRad-this.robotPoseGetter.get().getRotation().getRadians()));
        // filteredRequiredRobotYawRad += 0.1 * (Math.IEEEremainder(requiredRobotYawRad - filteredRequiredRobotYawRad, 2 * Math.PI)); 
        Translation2d filtered_shooter_xy_vector_required = new Translation2d(
            Math.cos(filteredRequiredRobotYawRad + robotToShooter.get().getRotation().getZ()) * shooter_xy_vector_required.getNorm(),
            Math.sin(filteredRequiredRobotYawRad + robotToShooter.get().getRotation().getZ()) * shooter_xy_vector_required.getNorm()
        );
        // Final check (magnitude of V_p_s should be V0)
        double muzzleVelocityCheck = 0;
        Translation3d shooterVelocityCheck = new Translation3d(
            shooterVelWorld.getX() + shooter_xy_vector_required.getX(),
            shooterVelWorld.getY() + shooter_xy_vector_required.getY(),
            shooterVelWorld.getZ() + vpwZ
        );
        if(RobotBase.isSimulation()){
           
            if(this.shooting_permisive == true && (Timer.getFPGATimestamp() - last_shot_time) > 0.05){
                // System.out.print("here");
                
                // shoot ball in sim 
                trajectoryvis[ball_index] = new Translation3d(shooterPoseWorld.getX(), shooterPoseWorld.getY(), shooterPoseWorld.getZ());
                trajectoryvis_prime[ball_index] = new Translation3d(
                    shooterVelocityCheck.getX(),
                    shooterVelocityCheck.getY(),
                    shooterVelocityCheck.getZ()
                );
                ball_index = (ball_index + 1) % num_balls_vis;

                last_shot_time = Timer.getFPGATimestamp();
            }else{
                
            }
            
        }
        if (!trajectory.isValid || Math.abs(trajectory.Yerror) >.2) {
            
            _best_solution = new Solution("No Solution", requiredRobotYawRad, trajectory.angle, trajectory.time, muzzleVelocityCheck,shooterVelocityCheck, trajectory.Yerror);
            return 1e8; // Penalize solutions that are physically impossible
        }
        _best_solution = new Solution("Solution Found", requiredRobotYawRad, trajectory.angle, trajectory.time, muzzleVelocityCheck,shooterVelocityCheck, trajectory.Yerror);
        
    return trajectory.Yerror;
    }


    public Solution solveKinematics() {
        // Reset tracking variables for this solver run
        
        _best_solution = new Solution("No Solution", 0, 0, 0, 0, new Translation3d(0,0,0),Double.POSITIVE_INFINITY);

        // Compute shooter world pose
        double yawR = this.robotPoseGetter.get().getRotation().getRadians();
        Pose3d robotPose3d = new Pose3d(new Translation3d(this.robotPoseGetter.get().getX(), this.robotPoseGetter.get().getY(), 0.0), 
                                        new Rotation3d(0.0, 0.0, yawR));
        Pose3d shooterPoseWorld =robotPose3d.transformBy(this.robotToShooter.get());

        // Compute shooter world velocity
        Transform2d robotVelocity = this.robotVelocityGetter.get();
        // System.out.printf(robotVelocity.getX() + ", " + robotVelocity.getY() + ", " + robotVelocity.getRotation().getDegrees() + "\n");
        double robotOmegaZ= robotVelocity.getRotation().getRadians();
        
        Translation3d shootervelFromRobotRotation = new Translation3d(-robotOmegaZ * this.robotToShooter.get().getTranslation().getY(),
                                                                robotOmegaZ * this.robotToShooter.get().getTranslation().getX(),
                                                                0);
        shootervelFromRobotRotation = shootervelFromRobotRotation.rotateBy(new Rotation3d(0.0, 0.0, yawR));
       
        double shooterVelX = robotVelocity.getX() + shootervelFromRobotRotation.getX();
        double shooterVelY = robotVelocity.getY() + shootervelFromRobotRotation.getY();
        double shooterVelZ = 0.0; // Replace with actual Z velocity if available
        Translation3d shooterVelWorld_final = new Translation3d(shooterVelX, shooterVelY, shooterVelZ);
        // System.out.printf("Shooter Velocity solver (m/s): VX: %.2f, VY: %.2f, VZ: %.2f\n", shooterVelX, shooterVelY, shooterVelZ);



    
        double yError = objectiveFunction(shooterPoseWorld, shooterVelWorld_final);
            

        // Check if a valid solution was found
        if (_best_solution.status.equals("Solution Found")) {
            SmartDashboard.putNumber(this.shooter_number+"SS required yaw", Math.toDegrees(_best_solution.requiredRobotYawRad));
            SmartDashboard.putNumber(this.shooter_number+"SS required pitch",Math.toDegrees(_best_solution.requiredShooterPitchRad));
            SmartDashboard.putNumber(this.shooter_number+"SS time of flight", _best_solution.timeOfFlight);
            // SmartDashboard.putNumber(this.shooter_number+"SS muzzle velocity check", _best_solution.muzzleVelocityCheck);
            // SmartDashboard.putNumber(this.shooter_number+"SS min y error", yError);
            
            return _best_solution;
        } else {
            return new Solution("No Solution Found", 0, 0, 0, 0,new Translation3d(0,0,0), Double.POSITIVE_INFINITY);
        }
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double currentTime = Timer.getFPGATimestamp(); // dont Rerun too Quickly and waste compute
        if (currentTime - lastTimeSolved < RefreshRate) {
            return;
        }
        double timeBeforeSolving = Timer.getFPGATimestamp();
        solveKinematics();
        double timeAfterSolving = Timer.getFPGATimestamp();
        // SmartDashboard.putNumber(this.shooter_number+"SS Solve Time (ms)", (timeAfterSolving - timeBeforeSolving)*1000.0);
        if(currentTime - shooting_watchdog > 100){
            this.shooting_permisive = false;
        }
        
        if(RobotBase.isSimulation()){
            
            // Publish as struct array
            for(int i =0; i < num_balls_vis; i++){
                trajectoryvis_prime[i] = new Translation3d(trajectoryvis_prime[i].getX(), trajectoryvis_prime[i].getY(), trajectoryvis_prime[i].getZ()-9.81*(currentTime-lastTimeSolved));
                trajectoryvis[i] = trajectoryvis[i].plus(trajectoryvis_prime[i].times(currentTime-lastTimeSolved));
            }
            trajectoryPublisher.set(trajectoryvis);
        }
        lastTimeSolved = currentTime;
    }
    public double getBestSolutionYError(){ // closest we can get to the target
        return _best_solution.minYError; 
    }
    public double getBestSolutionTimestamp(){
        return _best_solution.timestamp;
    }
    public double getRobotRequiredYawRad(){
        return _best_solution.requiredRobotYawRad;
    }
    public double getShooterRequiredPitchRad(){
        return _best_solution.requiredShooterPitchRad;
    }
    public String getBestSolutionStatus(){
        return _best_solution.status;
    }
    public void AllowShooting(boolean allowed){
        this.shooting_watchdog = Timer.getFPGATimestamp();
        this.shooting_permisive = allowed;
    }
}
