import rev


BRUSHED: rev.SparkBase.MotorType = rev.SparkBase.MotorType.kBrushed
BRUSHLESS: rev.SparkBase.MotorType = rev.SparkBase.MotorType.kBrushless

SAFE_RESET: rev.SparkBase.ResetMode = rev.SparkBase.ResetMode.kResetSafeParameters
NO_SAFE_RESET: rev.SparkBase.ResetMode = rev.SparkBase.ResetMode.kNoResetSafeParameters

NO_PERSIST: rev.SparkBase.PersistMode = rev.SparkBase.PersistMode.kNoPersistParameters

COAST: rev.SparkBase.IdleMode = rev.SparkBaseConfig.IdleMode.kCoast
BRAKE: rev.SparkBase.IdleMode = rev.SparkBaseConfig.IdleMode.kBrake