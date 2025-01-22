package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;

public class PositionTracker {
    private Supplier<Double> elevatorPositionSupplier;
    private Supplier<Double> armAngleSupplier;
    private Supplier<Pose3d> carriagePoseSupplier;

    public void setElevatorPositionSupplier(Supplier<Double> elevatorPositionSupplier) {
        this.elevatorPositionSupplier = elevatorPositionSupplier;
    }

    public void setArmAngleSupplier(Supplier<Double> armAngleSupplier) {
        this.armAngleSupplier = armAngleSupplier;
    }

    public void setCarriagePoseSupplier(Supplier<Pose3d> carriagePoseSupplier) {
        this.carriagePoseSupplier = carriagePoseSupplier;
    }

    public double getElevatorPosition() {
        return elevatorPositionSupplier.get();
    }

    public double getArmAngle() {
        return armAngleSupplier.get();
    }

    public Pose3d getCarriagePose() {
        return carriagePoseSupplier.get();
    }
}