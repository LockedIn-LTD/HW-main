PROJECT_ID = "drivesense-c1d4c"
EVENT_COLLECTION = "events"
DRIVER_COLLECTION = "drivers"
from typing import Dict
from database.SensorData import SensorData
from database.Database import Database

class SensorDataService:
    """
    Handles Firestore interactions for creating and linking events.
    Designed to run on Jetson Orin Nano or any Python environment.
    """

    def __init__(self, project_id: str, credentials_file: str):
        self.db = Database(project_id, credentials_path=credentials_file)

    def upload_sensor_data(self, data: SensorData, driver_id: str) -> Dict:
        """
        Creates an event in Firestore and attaches it to the driver.
        """
        try:
            # Convert Event dataclass into Firestore dictionary
            sensor_map = data.to_map()
            sensor_map["driverId"] = driver_id

            # Fetch driver document
            driver_data = self.db.get_document(DRIVER_COLLECTION, driver_id)

            if not driver_data:
                raise Exception(f"Driver '{driver_id}' not found in Firestore.")

            # Append event summary to the driver's events list

            # Update driver document
            self.db.update_document(DRIVER_COLLECTION, driver_id, sensor_map)

            return sensor_map

        except Exception as e:
            raise Exception(f"Failed to create Firestore event: {str(e)}")