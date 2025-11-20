from dataclasses import dataclass, asdict
from typing import Dict

@dataclass
class SensorData:
    """Represents a logged driver event."""
    status: str
    heartRate: int = 0
    bloodOxygenLevel: int = 0
    vehicleSpeed: int = 0

    def to_map(self) -> Dict:
        """Convert the event to a Firestore-friendly dictionary."""
        return asdict(self)