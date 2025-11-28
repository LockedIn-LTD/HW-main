from dataclasses import dataclass, asdict
from typing import Dict

@dataclass
class Event:
    """Represents a logged driver event."""
    eventId: str
    status: str
    timeStamp: str
    date: str
    videoLink: str
    heartRate: int = 0
    bloodOxygenLevel: int = 0
    vehicleSpeed: int = 0

    def to_map(self) -> Dict:
        """Convert the event to a Firestore-friendly dictionary."""
        return asdict(self)