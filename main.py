import zmq
import time
from database.SensorDataService import SensorDataService
from database.SensorData import SensorData
import json 

PORT = "tcp://localhost:5557"

SENSOR_TOPIC = "sensor_out"
MODEL_TOPIC = "model_out"
STATUS_TOPIC = "status_out" # this topic contains the overall result: EMERGENCY, WARNING, NORMAL (we have to calculate it and pubish to this topic)

# Notes


service = SensorDataService(
        project_id = "drivesense-c1d4c",
        credentials_file = "firebase_admin.json"
    )

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
        topic = msg[0].decode()
        payload = msg[1]
        handle_message(topic, payload)
        
        

def handle_message(topic, message):
    print("Handling message for topic: " + topic)
    if topic.strip().startswith(SENSOR_TOPIC):
        data = parse_raw_data(message)
        push_data_to_firebase(service, sensor_data=data)

    elif topic.strip().startswith(MODEL_TOPIC):
        data = parse_model_data(message)
        push_data_to_firebase(service, model_data=data)

    elif topic.strip().startswith(STATUS_TOPIC):
        parse_raw_data(message)

    return


def parse_raw_data(message):
    json_string = message.decode('utf-8')
    sensor_dict = json.loads(json_string)
    return sensor_dict



def push_data_to_firebase(service, sensor_data=None, model_data=None):
    if sensor_data is None:
        sensor_data_obj = SensorData(status=model_data["model_status"]["status"],)
    elif model_data is None:
        sensor_data_obj = SensorData(heartRate=sensor_data["heartrate_sensor_1"]["heartRate"],
                            bloodOxygenLevel=sensor_data["heartrate_sensor_1"]["spO2"],
                            vehicleSpeed="0")

    if service.upload_sensor_data(sensor_data_obj, driver_id="driver_alice_rogan_1763414940586"):
        return True
    
    return False
    


if __name__ == "__main__":
    main()

 