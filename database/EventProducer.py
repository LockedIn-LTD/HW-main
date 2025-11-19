from EventModel import Event
from EventService import EventService

service = EventService(
    project_id = "drivesense-c1d4c",
    credentials_file = "firebase-admin.json"
)

event = Event(
    event_id="evt_001",
    status="CRITICAL",
    time_stamp="14:55:12",
    date="2025-02-18",
    video_link="gs://bucket/video1.mp4",
    heart_rate=90,
    blood_oxygen_level=96,
    vehicle_speed=62
)

created = service.create_event(event, driver_id="driver_alice_rogan_1763414940586")
print("Uploaded:", created)