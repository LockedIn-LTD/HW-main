import zmq
import json
import time

STATUS_PORT = "tcp://localhost:5559"
STATUS_TOPIC = "status_out"

def run_status_subscriber():
    """
    Connects to the STATUS_PORT and subscribes to the STATUS_TOPIC
    to receive and display the final fatigue status updates.
    """
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    
    # Connect to the port where main.py is publishing the status
    try:
        print(f"Connecting to status publisher at {STATUS_PORT}...")
        socket.connect(STATUS_PORT)
    except Exception as e:
        print(f"Error connecting to ZMQ socket: {e}")
        print("Please ensure main.py is running and publishing on 5558.")
        return

    # Subscribe only to the STATUS_TOPIC
    socket.setsockopt_string(zmq.SUBSCRIBE, STATUS_TOPIC)
    print(f"Subscribed to topic: '{STATUS_TOPIC}'. Waiting for updates...")
    print("-" * 50)

    while True:
        try:
            # Receive multipart message (Topic, Payload)
            msg = socket.recv_multipart()
            
            if len(msg) < 2:
                print("Received incomplete message.")
                continue

            topic = msg[0].decode('utf-8')
            payload_raw = msg[1].decode('utf-8')

            # The topic should be STATUS_TOPIC, but we print it anyway
            # The payload is a JSON string containing the final status
            data = json.loads(payload_raw)

            final_status = data.get('status', 'UNKNOWN')
            timestamp = data.get('timestamp', 'N/A')

            print(f"[{timestamp}] STATUS: {final_status}")

        except zmq.error.ContextTerminated:
            print("ZMQ context terminated.")
            break
        except zmq.error.Again:
            # Non-blocking receive, just wait
            time.sleep(0.01)
            continue
        except json.JSONDecodeError:
            print(f"Error decoding JSON payload: {payload_raw}")
        except KeyboardInterrupt:
            print("\nSubscriber stopped by user.")
            break
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
            break

    # Clean up ZMQ resources
    socket.close()
    context.term()

if __name__ == "__main__":
    run_status_subscriber()