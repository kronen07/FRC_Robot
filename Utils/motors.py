import rev
from .constants import NO_PERSIST, SAFE_RESET, NO_SAFE_RESET


class REVSparkMax(rev.SparkMax):
    def __init__(self, can_id: int, motor_type: rev.SparkBase.MotorType, is_inverted: bool = False) -> None:
        super().__init__(can_id, motor_type)
        
    
    def invert(self, is_inverted: bool) -> None:
        set_inverted: rev.SparkMaxConfig = rev.SparkMaxConfig().inverted(is_inverted)
        self.configure(set_inverted, SAFE_RESET, NO_PERSIST)
    
    
    def follow(self, leader_id: int) -> None:
        set_leader: rev.SparkMaxConfig = rev.SparkMaxConfig().follow(leader_id)
        self.configure(set_leader, NO_SAFE_RESET, NO_PERSIST)
    
    
    def idle(self, motor_idle: rev.SparkBaseConfig.IdleMode) -> None:
        set_idle: rev.SparkMaxConfig = rev.SparkMaxConfig().setIdleMode(motor_idle)
        self.configure(set_idle, NO_SAFE_RESET, NO_PERSIST)
    
    
    def setEncoderPos(self, position: float) -> None:
        self.getEncoder().setPosition(position)
    
    
    def getEncoderPos(self) -> float:
        return self.getEncoder().getPosition()