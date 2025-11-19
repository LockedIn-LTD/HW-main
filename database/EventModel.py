from dataclasses import dataclass, asdict
from typing import Dict

@dataclass
class Event:
    """Represents a logged driver event."""
    event_id: str
    status: str
    time_stamp: str
    date: str
    video_link: str
    heart_rate: int = 0
    blood_oxygen_level: int = 0
    vehicle_speed: int = 0

    def to_map(self) -> Dict:
        """Convert the event to a Firestore-friendly dictionary."""
        return asdict(self)