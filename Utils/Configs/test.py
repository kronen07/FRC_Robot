import wpilib as wp
import wpilib.drive as wpd
import navx
import rev
from ..motors import REVSparkMax
from ..constants import BRUSHED


class TestRobotConfig:
    def __init__(self):
        # CAN ID
        power_distribution  : int = 1
        left_motor          : int = 2
        left_motor_follower : int = 3
        right_motor         : int = 4
        right_motor_follower: int = 5
        
        # Power Distribution
        self.pd : wp.PowerDistribution = wp.PowerDistribution(power_distribution, wp.PowerDistribution.ModuleType.kRev)
        self.pd.clearStickyFaults()
        
        # NavX
        self.navx: navx.AHRS = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)
        
        # Motors
        self.left_motor          : REVSparkMax = REVSparkMax(left_motor, BRUSHED, True)
        self.left_motor_follower : REVSparkMax = REVSparkMax(left_motor_follower, BRUSHED, True)
        self.right_motor         : REVSparkMax = REVSparkMax(right_motor, BRUSHED)
        self.right_motor_follower: REVSparkMax = REVSparkMax(right_motor_follower, BRUSHED)
        
        # Follower Motors
        self.left_motor_follower.follow(left_motor)
        self.right_motor_follower.follow(right_motor)
        
        # Differential Drive
        self.drive: wpd.DifferentialDrive = wpd.DifferentialDrive(self.left_motor, self.right_motor)
        
        # Controller
        self.driver: wp.Joystick = wp.Joystick(0)
        
        
    def setDriveIdle(self, drive_idle: rev.SparkMaxConfig.IdleMode) -> None:
        self.left_motor.idle(drive_idle)
        self.left_motor_follower.idle(drive_idle)
        self.right_motor.idle(drive_idle)
        self.right_motor.idle(drive_idle)
        
        
        