package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.leds.Leds;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class LedsTest {

    static final double DELTA = 1e-9; // Error tolerance for double comparisons

    Leds m_Leds;
    DriverStation m_mockDriverStation;
    Notifier m_mockNotifier;
    Timer m_mockTimer;

    @BeforeEach // This method will run before each test
    public void setUp() {

        // Create DriverStation mock and set default behavior
        m_mockDriverStation = mock(DriverStation.class);
        when(m_mockDriverStation.isFMSAttached()).thenReturn(true);
        when(m_mockDriverStation.getAlliance()).thenReturn(DriverStation.Alliance.Blue);
        when(m_mockDriverStation.isDisabled()).thenReturn(true);
        when(m_mockDriverStation.isAutonomous()).thenReturn(false);
        when(m_mockDriverStation.isEStopped()).thenReturn(false);

        m_mockNotifier = mock(Notifier.class);
        

        m_mockTimer = mock(Timer.class);
        when(m_mockTimer.getFPGATimestamp()).thenReturn(0.0);


        m_Leds = Leds.getInstance();
    }

    @AfterEach // This method will run after each test
    public void tearDown() throws Exception {
        m_Leds.close();
    }

    @Test
    public void testGetInstance() {
        assertNotNull(m_Leds);
    }

    @Test
    public void testPeriodicDisabled() {
        when(m_mockDriverStation.isDisabled()).thenReturn(true);
        m_Leds.periodic();
        // Add assertions to verify the behavior when the robot is disabled
        assertTrue(true);
    }

    @Test
    public void testPeriodicAutonomous() {
        // when(mockDriverStation.isAutonomous()).thenReturn(true);
        m_Leds.periodic();
        // Add assertions to verify the behavior when the robot is in autonomous mode
        assertTrue(true);
    }

    @Test
    public void testPeriodicTeleop() {
        // when(mockDriverStation.isAutonomous()).thenReturn(false);
        // when(mockDriverStation.isDisabled()).thenReturn(false);
        m_Leds.periodic();
        // Add assertions to verify the behavior when the robot is in teleop mode
        assertTrue(true);
    }
}
