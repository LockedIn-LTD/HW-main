import zmq
import time
from datetime import date, datetime
import json
from collections import deque
# The user's provided imports for database services
from database.SensorDataService import SensorDataService
from database.SensorData import SensorData
from database.EventService import EventService
from database.EventModel import Event

# --- FIREBASE SERVICE INITIALIZATION ---
# NOTE: The credentials_file and project_id are provided by the user's skeleton.
sensor_service = SensorDataService(
    project_id="drivesense-c1d4c",
    credentials_file="firebase-admin.json"
)

event_service = EventService(
    project_id="drivesense-c1d4c",
    credentials_file="firebase-admin.json"
)

# --- GLOBAL CONSTANTS ---
# Using a constant driver ID for event and sensor data logging
DRIVER_ID = "driver_alice_rogan_1763414940586"

# --- ZMQ PORTS ---
MODEL_PORT = 5557
SENSOR_PORT = 5558
STATUS_PUBLISHER_PORT = 5559

# --- ZMQ TOPICS ---
SENSOR_TOPIC = "sensor_out"
MODEL_TOPIC = "model_out"
STATUS_TOPIC = "status_out"

# --- GLOBAL STRUCTURES AND CONSTANTS ---
# History stores all incoming data (sensor and model)
MAX_HISTORY_SIZE = 120  # Holds up to 2 minutes of data
data_history = deque(maxlen=MAX_HISTORY_SIZE)

# Thresholds (Centralized configuration for main logic)
THRESHOLDS = {
    # Health Sensors
    "HR_GOOD_MIN": 60, "HR_GOOD_MAX": 160,
    "HR_DROWSY_MIN": 55, "HR_DROWSY_MAX": 180,
    "SPO2_GOOD_MIN": 65,
    "SPO2_DROWSY_MIN": 50,

    # YOLO (Perclos timing)
    "PERCLOS_DROWSY_S": 1.5, "PERCLOS_CRITICAL_S": 4.0,

    # Pressure (Time-based in seconds)
    "PRESSURE_ONE_HAND_OFF_TIME_S": 10,
    "PRESSURE_BOTH_HANDS_OFF_TIME_S": 5,
    "LOW_PRESSURE_THRESHOLD": 6000
}

# --- Helper Functions for Time-Based Analysis ---


def get_recent_sensor_readings(history):
    """Filter history for sensor data and return list of payloads."""
    return [d['payload'] for d in history if d['topic'] == SENSOR_TOPIC]


def get_recent_model_outputs(history):
    """Filter history for model data and return list of payloads."""
    return [d['payload'] for d in history if d['topic'] == MODEL_TOPIC]


def calculate_avg_hr_spo2(data):
    """Calculates the averaged HR and SpO2 for a single sensor data entry."""
    hr1 = data.get("heartrate_sensor_1", {}).get("heartRate", 0)
    hr2 = data.get("heartrate_sensor_2", {}).get("heartRate", 0)
    spO2_1 = data.get("heartrate_sensor_1", {}).get("spO2", 0)
    spO2_2 = data.get("heartrate_sensor_2", {}).get("spO2", 0)

    valid_hrs = [hr for hr in [hr1, hr2] if hr > 0]
    valid_spO2s = [s for s in [spO2_1, spO2_2] if s > 0]

    avg_hr = sum(valid_hrs) / len(valid_hrs) if valid_hrs else 0
    avg_spO2 = sum(valid_spO2s) / len(valid_spO2s) if valid_spO2s else 0

    return avg_hr, avg_spO2

# --- Fatigue Detection Logic ---


def detect_fatigue_and_publish_status():
    """
    Analyzes the combined data history based on defined thresholds,
    determines the final status, publishes via ZMQ, and logs
    EMERGENCY events to the EventService.
    """
    sensor_readings = get_recent_sensor_readings(data_history)
    model_outputs = get_recent_model_outputs(data_history)

    # Initialize metric variables for logging and event creation
    latest_hr = 0.0
    latest_spo2 = 0.0
    latest_perclos = 0.0
    log_one_off = 0
    log_both_off = 0

    # We must have sensor data to run the checks
    if not sensor_readings:
        final_status = "AWAITING_SENSOR_DATA"
        return

    # Initialize alert flags
    hr_alert_drowsy = False
    hr_alert_critical = False
    spo2_alert_drowsy = False
    spo2_alert_critical = False
    is_pressure_drowsy = False
    is_pressure_critical = False
    is_perclos_drowsy = False
    is_perclos_critical = False

    # --- 1. Health Sensor Analysis (HR & SpO2) ---
    if len(sensor_readings) > 0:
        first_hr, _ = calculate_avg_hr_spo2(sensor_readings[0])
        latest_hr, latest_spo2 = calculate_avg_hr_spo2(sensor_readings[-1])

        # A. Absolute Threshold Check (Latest Reading)
        if latest_hr > 0:
            if latest_hr < THRESHOLDS["HR_DROWSY_MIN"] or latest_hr > THRESHOLDS["HR_DROWSY_MAX"]:
                hr_alert_critical = True
            elif latest_hr < THRESHOLDS["HR_GOOD_MIN"] or latest_hr > THRESHOLDS["HR_GOOD_MAX"]:
                hr_alert_drowsy = True

        if latest_spo2 > 0:
            if latest_spo2 < THRESHOLDS["SPO2_DROWSY_MIN"]:
                spo2_alert_critical = True
            elif latest_spo2 < THRESHOLDS["SPO2_GOOD_MIN"]:
                spo2_alert_drowsy = True

        # B. Relative Change Check (HR: +/- 10% from starting baseline)
        if first_hr > 0 and latest_hr > 0:
            lower_bound = first_hr * 0.9
            upper_bound = first_hr * 1.1
            if latest_hr < lower_bound or latest_hr > upper_bound:
                hr_alert_drowsy = True

        # --- 2. Pressure Sensor (Hands-on-Wheel) Analysis ---
        consecutive_one_hand_off_time = 0
        consecutive_both_off_time = 0
        low_p = THRESHOLDS["LOW_PRESSURE_THRESHOLD"]

        # Iterate backwards from the most recent reading (sensor_readings[-1])
        for data in reversed(sensor_readings):
            # Use .get() for safe access to sensor data
            pressure_data = data.get("pressure_sensor_adc", {})
            left_p = pressure_data.get("left", 0)
            right_p = pressure_data.get("right", 0)

            is_left_off = left_p < low_p
            is_right_off = right_p < low_p

            # Check for BOTH HANDS OFF (Critical condition)
            if is_left_off and is_right_off:
                # Only count 'both off' if we haven't started counting 'one hand off' from this streak
                if consecutive_one_hand_off_time == 0:
                    consecutive_both_off_time += 1
            # Check for EXACTLY ONE HAND OFF (Drowsy condition)
            elif is_left_off or is_right_off:
                # Only count 'one hand off' if we're not currently in 'both off' mode
                if consecutive_both_off_time == 0:
                    consecutive_one_hand_off_time += 1
            # HANDS ON (High Pressure) - Stop both consecutive counts
            else:
                break  # Stop the loop, as the consecutive streak is broken

        # The loop has finished, or the streak was broken. Check the longest streak.
        is_pressure_drowsy = consecutive_one_hand_off_time >= THRESHOLDS[
            "PRESSURE_ONE_HAND_OFF_TIME_S"]
        is_pressure_critical = consecutive_both_off_time >= THRESHOLDS[
            "PRESSURE_BOTH_HANDS_OFF_TIME_S"]

        # Variables for logging
        log_one_off = consecutive_one_hand_off_time
        log_both_off = consecutive_both_off_time

    # --- 3. YOLO (PERCLOS) Analysis ---

    # We only need the LATEST perclos time from the standardized model output
    latest_perclos = next((data.get("perclos_time_s", 0) for data in reversed(
        model_outputs) if "perclos_time_s" in data), 0)

    if latest_perclos > 0:
        if latest_perclos >= THRESHOLDS["PERCLOS_CRITICAL_S"]:
            is_perclos_critical = True
        elif latest_perclos >= THRESHOLDS["PERCLOS_DROWSY_S"]:
            is_perclos_drowsy = True

    # --- 4. Combined Decision Logic ---

    # CRITICAL: Any single critical flag triggers an EMERGENCY
    if is_pressure_critical or is_perclos_critical or hr_alert_critical or spo2_alert_critical:
        final_status = "CRITICAL"

    # WARNING: Any single drowsy flag
    elif is_pressure_drowsy or is_perclos_drowsy or hr_alert_drowsy or spo2_alert_drowsy:
        final_status = "MILD"

    # NORMAL: No alerts
    else:
        final_status = "STABLE"


    # --- 5. Event Logging to Firebase (EventService) ---

    try:
        sensor_data_upload = SensorData(
            status=final_status,
            heartRate=latest_hr,
            bloodOxygenLevel=latest_spo2,
            vehicleSpeed="0"
        )

        if sensor_service.upload_sensor_data(sensor_data_upload, driver_id=DRIVER_ID):
            print(f"[SensorDataService] Uploaded latest sensor reading.")
            #pass

    except Exception as e:
        print(
            f"[SensorDataService] Failed to create or upload sensor data: {e}")

    if final_status == "CRITICAL":
        event_data = Event(
            eventId=f"evt_{datetime.now().timestamp()}",
            status=final_status,
            timeStamp=datetime.now().strftime("%H:%M:%S"),
            date=date.today().isoformat(),
            # Placeholder for video link, as it's not present in current inputs
            videoLink="N/A - Check video storage for timestamp",
            heartRate=latest_hr,
            bloodOxygenLevel=latest_spo2,
            vehicleSpeed=0
        )

        if event_service.create_event(event_data, driver_id=DRIVER_ID):
            print(
                f"[EventService] Successfully logged {final_status} event to Firestore.")
        else:
            print(
                f"[EventService] FAILED to log {final_status} event to Firestore.")

    # Print detailed log for debugging
    print(f"Detail Log | HR: {latest_hr:.1f}, SpO2: {latest_spo2:.1f}, Perclos: {latest_perclos:.1f}s, Both Off Consecutive: {log_both_off}s, One Off Consecutive: {log_one_off}s")


# --- ZMQ Handling and Main Loop ---


def handle_message(topic, message):
    """
    Parses the message, populates the global data history, and triggers status check.
    Also handles continuous sensor data logging.
    """
    data = parse_raw_data(message)

    global data_history
    entry = {
        "timestamp": datetime.now().isoformat(),
        "topic": topic.strip(),
        "payload": data
    }
    data_history.append(entry)

    detect_fatigue_and_publish_status()

    return


def parse_raw_data(message):
    """Decodes the JSON message payload."""
    try:
        json_string = message.decode('utf-8')
        data_dict = json.loads(json_string)
        return data_dict
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON: {e}")
        return {}


def main():
    print(
        f"Listening for Model data on tcp://localhost:{MODEL_PORT} (Topic: {MODEL_TOPIC})")
    print(
        f"Listening for Sensor data on tcp://localhost:{SENSOR_PORT} (Topic: {SENSOR_TOPIC})")

    context = zmq.Context()

    # 1. Setup Model Data Subscriber (Port 5557)
    model_socket = context.socket(zmq.SUB)
    model_socket.connect(f"tcp://localhost:{MODEL_PORT}")
    model_socket.setsockopt_string(zmq.SUBSCRIBE, MODEL_TOPIC)

    # 2. Setup Sensor Data Subscriber (Port 5558)
    sensor_socket = context.socket(zmq.SUB)
    sensor_socket.connect(f"tcp://localhost:{SENSOR_PORT}")
    sensor_socket.setsockopt_string(zmq.SUBSCRIBE, SENSOR_TOPIC)

    # 3. Setup Poller to monitor both sockets
    poller = zmq.Poller()
    poller.register(model_socket, zmq.POLLIN)
    poller.register(sensor_socket, zmq.POLLIN)

    print("Main logic loop started. Processing data...")

    while True:
        try:
            # Poll both sockets with a small timeout (e.g., 100ms)
            # This makes the loop slightly more responsive to keyboard interrupts
            socks = dict(poller.poll(100))

            # Check for Model Socket message
            if model_socket in socks and socks[model_socket] == zmq.POLLIN:
                msg = model_socket.recv_multipart()
                topic = msg[0].decode()
                payload = msg[1]
                handle_message(topic, payload)

            # Check for Sensor Socket message
            if sensor_socket in socks and socks[sensor_socket] == zmq.POLLIN:
                msg = sensor_socket.recv_multipart()
                topic = msg[0].decode()
                payload = msg[1]
                handle_message(topic, payload)

        except KeyboardInterrupt:
            print("\nShutting down main logic.")
            break
        except Exception as e:
            print(f"An error occurred in the ZMQ loop: {e}")
            time.sleep(1)

    # Clean up ZMQ resources
    model_socket.close()
    sensor_socket.close()
    context.term()


if __name__ == "__main__":
    main()
