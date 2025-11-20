from SensorData import SensorData
from SensorDataService import SensorDataService

service = SensorDataService(
    project_id = "drivesense-c1d4c",
    credentials_file = "firebase-admin.json"
)

sensor_data = SensorData(
    eventId="evt_001",
    status="CRITICAL",
    timeStamp="14:55:12",
    date="2025-02-18",
    videoLink="gs://bucket/video1.mp4",
    heartRate=90,
    bloodOxygenLevel=96,
    vehicleSpeed=62
)

created = service.upload_sensor_data(sensor_data, driver_id="driver_alice_rogan_1763414940586")
print("Uploaded:", created)