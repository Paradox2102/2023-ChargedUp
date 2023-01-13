package frc.robot;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.sensors.PigeonIMU;

public class Sensor implements SensorData{
	private DoubleSupplier m_leftPos;
	private DoubleSupplier m_rightPos;
	private DoubleSupplier m_leftVel;
	private DoubleSupplier m_rightVel;
	private PigeonIMU m_gyro;
	
	public Sensor(DoubleSupplier leftPos, DoubleSupplier rightPos, DoubleSupplier leftVel, DoubleSupplier rightVel, PigeonIMU gyro) {
		m_leftPos = leftPos;
		m_rightPos = rightPos;
		m_leftVel = leftVel;
		m_rightVel = rightVel;
		
		m_gyro = gyro;
	}
	
	public double getLeftEncoderPos() {
		return m_leftPos.getAsDouble()*Constants.k_feetPerTick;
	}
	
	public double getRightEncoderPos() {
		return m_rightPos.getAsDouble()*Constants.k_feetPerTick;
	}
	
	public double getLeftEncoderVel() {
		return m_leftVel.getAsDouble()*10*Constants.k_feetPerTick;
	}
	
	public double getRightEncoderVel() {
		return m_rightVel.getAsDouble()*10*Constants.k_feetPerTick;
	}

	public PigeonIMU getGyro() {
		return m_gyro;
	}
	
	public double getAngle() {
		double[] data = new double[3];
		m_gyro.getYawPitchRoll(data);
		return data[0];
	}
}