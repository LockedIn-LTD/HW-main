PROJECT_ID = "drivesense-c1d4c"
EVENT_COLLECTION = "events"
DRIVER_COLLECTION = "drivers"
from typing import Dict
from database.EventModel import Event
from database.Database import Database

class EventService:
    """
    Handles Firestore interactions for creating and linking events.
    Designed to run on Jetson Orin Nano or any Python environment.
    """

    def __init__(self, project_id: str, credentials_file: str):
        self.db = Database(project_id, credentials_path=credentials_file)

    def create_event(self, event: Event, driver_id: str) -> Dict:
        """
        Creates an event in Firestore and attaches it to the driver.
        """
        try:
            # Convert Event dataclass into Firestore dictionary
            event_data = event.to_map()
            event_data["driverId"] = driver_id

            # Save event in Firestore under /events/{eventId}
            self.db.set_document(EVENT_COLLECTION, event.eventId, event_data)

            # Fetch driver document
            driver_data = self.db.get_document(DRIVER_COLLECTION, driver_id)

            if not driver_data:
                raise Exception(f"Driver '{driver_id}' not found in Firestore.")

            # Append event summary to the driver's events list
            current_events = driver_data.get("events", [])
            current_events.append(event_data)

            # Update driver document
            self.db.update_document(DRIVER_COLLECTION, driver_id, {
                "events": current_events
            })

            return event_data

        except Exception as e:
            raise Exception(f"Failed to create Firestore event: {str(e)}")