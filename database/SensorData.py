from dataclasses import dataclass, asdict
from typing import Dict, Optional

@dataclass
class SensorData:
    """Represents a logged driver event."""
    status: Optional[str] = None
    heartRate: Optional[int] = None
    bloodOxygenLevel: Optional[int] = None
    vehicleSpeed: Optional[int] = None

    def to_map(self) -> Dict:
        """Convert the event to a Firestore-friendly dictionary."""
        return asdict(self)
