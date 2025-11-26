import zmq
import time
from datetime import datetime, date
from database.SensorDataService import SensorDataService
from database.SensorData import SensorData
from database.EventService import EventService
from database.EventModel import Event
import json 

PORT = "tcp://localhost:5557"

SENSOR_TOPIC = "sensor_out"
MODEL_TOPIC = "model_out"
STATUS_TOPIC = "status_out" # this topic contains the overall result: EMERGENCY, WARNING, NORMAL (we have to calculate it and pubish to this topic)

# Notes


sensor_service = SensorDataService(
        project_id = "drivesense-c1d4c",
        credentials_file = "firebase_admin.json"
    )

event_service = EventService(
        project_id = "drivesense-c1d4c",
        credentials_file = "firebase_admin.json"
    )

counter = 0
sensor_data_object = None
event_data_object = None


def main():
    print(f"Established ZMQ server on {SENSOR_TOPIC} for sensor data")
    print(f"Established ZMQ server on {MODEL_TOPIC} for model data")
    print(f"Established ZMQ server on {STATUS_TOPIC} for status data")

    # 1. start screen UI
    # 2. start cameras
    # 3. start sensors

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(PORT)
    socket.setsockopt_string(zmq.SUBSCRIBE, SENSOR_TOPIC)
    socket.setsockopt_string(zmq.SUBSCRIBE, MODEL_TOPIC)
    socket.setsockopt_string(zmq.SUBSCRIBE, STATUS_TOPIC)

    print("Listening for messages...")

    while True:
        # Main logic loop
        # read sensor data and read model status
        msg = socket.recv_multipart()
        if len(msg) == 1:
            # message contains topic + payload together
            raw = msg[0].decode()
            if " " in raw:
                topic, payload = raw.split(" ", 1)
                payload = payload.encode()  # make payload bytes again
                handle_message(topic, payload)
            else:
                topic = raw
                payload = b""
        else:
            # standard multipart (topic, payload)
            topic = msg[0].decode()
            payload = msg[1]
            handle_message(topic, payload)
        
        

def handle_message(topic, message):
    print("Handling message for topic: " + topic)
    if topic.strip().startswith(SENSOR_TOPIC):
        data = parse_raw_data(message)
        push_data_to_firebase(sensor_service, sensor_data=data)

    elif topic.strip().startswith(MODEL_TOPIC):
        data = parse_raw_data(message)
        push_data_to_firebase(event_service, model_data=data)

    elif topic.strip().startswith(STATUS_TOPIC):
        parse_raw_data(message)

    return


def parse_raw_data(message):
    json_string = message.decode('utf-8')
    sensor_dict = json.loads(json_string)
    return sensor_dict



def push_data_to_firebase(service, sensor_data=None, model_data=None):
    global counter
    global sensor_data_object
    global event_data_object

    if model_data:
        if model_data["status"] == "FATIGUE":
            event_data_object = Event(
                event_id=f"evt_{counter}",
                status=model_data["status"],
                timeStamp=datetime.now().strftime("%H:%M:%S"),
                date=date.today().isoformat(),
                videoLink=model_data["image"],
                heartRate=sensor_data_object.heartRate if sensor_data_object else 0,
                bloodOxygenLevel=sensor_data_object.bloodOxygenLevel if sensor_data_object else 0,
                vehicleSpeed=0
            )
            if sensor_data_object is not None:
                sensor_data_object.status = model_data["status"]

            if service.create_event(event_data_object, driver_id="driver_alice_rogan_1763414940586"):
                counter += 1
                return True

    else:
        sensor_data_object = SensorData(
            heartRate=sensor_data["heartrate_sensor_1"]["heartRate"],
            bloodOxygenLevel=sensor_data["heartrate_sensor_1"]["spO2"],
            vehicleSpeed="0"
        )
        if service.upload_sensor_data(sensor_data_object, driver_id="driver_alice_rogan_1763414940586"):
            return True


    return False



if __name__ == "__main__":
    main()

 