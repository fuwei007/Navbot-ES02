import base64  # Used for Base64 encoding and decoding, and for encoding and transmitting audio data
import json  # Handling data exchange in JSON format
import os  # Operating system interface, used to read environment variables
import queue  # Thread-safe queue for microphone data buffering
import socket  # Network socket operations
import ssl
import subprocess  # Run external programs (e.g., open Notepad)
import threading  # Multithreading support
import time  # Time-related operations
import pyaudio  # Audio input and output processing
import socks  # SOCKS proxy support
import websocket  # WebSocket client
from dotenv import load_dotenv  # Load environment variables from.env file

import ES02_def_function

# Load the environment variable configuration from the.env file
load_dotenv()

# Set the SOCKS5 proxy (globally replace the socket implementation)
socket.socket = socks.socksocket

# Get the OpenAI API key from the environment variable
API_KEY = os.getenv("OPENAI_API_KEY")
if not API_KEY:
    raise ValueError("Please set the 'OPENAI_API_KEY' environment variable. The API key is missing.")

# WebSocket server URL (real-time voice conversion interface)
# WS_URL = 'wss://app-dev.easyone.ai:8815/api/realtime-convert'
WS_URL = 'wss://api.openai.com/v1/realtime?model=gpt-4o-mini-realtime-preview-2024-12-17'

# Audio stream parameter configuration
CHUNK_SIZE = 1024  # Size of each audio data chunk to process (bytes)
RATE = 24000  # Audio sampling rate (Hz)
FORMAT = pyaudio.paInt16  # Audio format (16-bit integer PCM)

# Global variable definitions
audio_buffer = bytearray()  # Store the received audio data (for speaker playback)
mic_queue = queue.Queue()  # Thread-safe queue for microphone collection data

stop_event = threading.Event()  # Thread stop event flag

mic_on_at = 0  # Microphone activation timestamp (for echo cancellation)
mic_active = None  # Current state record of the microphone
REENGAGE_DELAY_MS = 500  # Delay time to reactivate the microphone (milliseconds)


def clear_audio_buffer():
    """Clear the audio playback buffer"""
    global audio_buffer
    audio_buffer = bytearray()  # Reset to an empty byte array
    print('🔵 Audio buffer cleared')


def stop_audio_playback():
    """Stop audio playback (set the playback status flag)"""
    global is_playing
    is_playing = False
    print('🔵 Audio playback stopped')


def mic_callback(in_data, frame_count, time_info, status):
    """
    Microphone data collection callback function
    Parameters:
        in_data: Input audio data (binary)
        frame_count: Number of frames (number of audio frames in this callback)
        time_info: Time information dictionary
        status: Status flag (normal/error)
    Returns:
        (None, pyaudio.paContinue) indicating to continue collection
    """
    global mic_on_at, mic_active

    # Detect and update the microphone status
    if mic_active != True:
        print('🎙️🟢 Microphone activated')
        mic_active = True

    # Put the audio data into the queue (thread-safe)
    mic_queue.put(in_data)

    # Commented-out echo cancellation logic (can be enabled as needed)
    # if time.time() > mic_on_at:
    #     if mic_active != True:
    #         print('🎙️🟢 Microphone activated')
    #         mic_active = True
    #     mic_queue.put(in_data)
    # else:
    #     if mic_active != False:
    #         print('🎙️🔴 Microphone muted')
    #         mic_active = False

    return (None, pyaudio.paContinue)  # Return empty data and the flag to continue collection


def send_mic_audio_to_websocket(ws):
    """
    Send microphone audio data to the WebSocket server
    Parameters:
        ws: Established WebSocket connection object
    """
    try:
        while not stop_event.is_set():  # Keep running until a stop signal is received
            if not mic_queue.empty():  # Check if there is data in the queue
                # Get an audio data chunk from the queue
                mic_chunk = mic_queue.get()

                # Base64 encode the audio data (suitable for JSON transmission)
                encoded_chunk = base64.b64encode(mic_chunk).decode('utf-8')

                # Construct a WebSocket message (JSON format)
                message = json.dumps({
                    'type': 'input_audio_buffer.append',  # Message type
                    'audio': encoded_chunk  # Encoded audio data
                })

                try:
                    ws.send(message)  # Send the message
                except Exception as e:
                    print(f'Error sending microphone audio: {e}')
    except Exception as e:
        print(f'Exception in the microphone data sending thread: {e}')
    finally:
        print('Microphone data sending thread exited')


def speaker_callback(in_data, frame_count, time_info, status):
    """
    Speaker playback callback function
    Parameters:
        in_data: Input data (not used)
        frame_count: Number of audio frames required
        time_info: Time information dictionary
        status: Status flag
    Returns:
        (audio_chunk, pyaudio.paContinue) audio data and the flag to continue playback
    """
    global audio_buffer, mic_on_at

    # Calculate the number of bytes needed (16-bit audio = 2 bytes/sample)
    bytes_needed = frame_count * 2
    current_buffer_size = len(audio_buffer)

    if current_buffer_size >= bytes_needed:
        # The buffer has enough data: take out the required data
        audio_chunk = bytes(audio_buffer[:bytes_needed])
        audio_buffer = audio_buffer[bytes_needed:]  # Update the buffer

        # Set the microphone activation time (current time + delay) to avoid echo
        mic_on_at = time.time() + REENGAGE_DELAY_MS / 1000
    else:
        # The buffer is insufficient: fill the remaining part with silent data
        audio_chunk = bytes(audio_buffer) + b'\x00' * (bytes_needed - current_buffer_size)
        audio_buffer.clear()  # Clear the buffer

    return (audio_chunk, pyaudio.paContinue)


def receive_audio_from_websocket(ws):
    """
    Receive audio data from the WebSocket and process server events
    Parameters:
        ws: Established WebSocket connection object
    """
    global audio_buffer

    try:
        while not stop_event.is_set():  # Keep running until a stop signal is received
            try:
                # Receive a WebSocket message
                message = ws.recv()

                # Handle empty messages (connection closed or EOF)
                if not message:
                    print('🔵 Received an empty message (possibly the connection is closed)')
                    break

                # Parse the JSON message
                message = json.loads(message)
                event_type = message['type']  # Extract the event type
                print(f'⚡️ Received a WebSocket event: {event_type}')

                # Process according to the event type
                if event_type == 'session.created':
                    # Session creation event: send session configuration
                    send_fc_session_update(ws)

                elif event_type == 'response.audio.delta':
                    # Audio data chunk event: decode and store in the playback buffer
                    audio_content = base64.b64decode(message['delta'])
                    audio_buffer.extend(audio_content)
                    print(f'🔵 Received {len(audio_content)} bytes of audio, total buffer size: {len(audio_buffer)}')

                elif event_type == 'input_audio_buffer.speech_started':
                    # Speech start event: clear the buffer and stop playback
                    print('🔵 Speech start detected, clearing the buffer and stopping playback')
                    clear_audio_buffer()
                    stop_audio_playback()

                elif event_type == 'response.audio.done':
                    # Audio end event
                    print('🔵 AI voice ended')

                elif event_type == 'response.function_call_arguments.done':
                    # Function call completion event
                    handle_function_call(message, ws)

            except Exception as e:
                print(f'Error receiving audio data: {e}')
    except Exception as e:
        print(f'Exception in the audio receiving thread: {e}')
    finally:
        print('Audio receiving thread exited')


def handle_function_call(event_json, ws):
    """
    Process the function call request received from the server
    Parameters:
        event_json: JSON object containing function call information
        ws: Current WebSocket connection object
    """
    try:
        # Extract the function name and call ID from the event data
        name = event_json.get("name", "")  # Function name
        call_id = event_json.get("call_id", "")  # Unique identifier for this call

        # Get and parse the function parameters (convert JSON string to dictionary)
        arguments = event_json.get("arguments", "{}")
        function_call_args = json.loads(arguments)

        # Perform different operations according to the function name
        if name == "move_robot":
            # Extract the angle parameter
            action = function_call_args.get("action")
            value = function_call_args.get("value")

            print("receive action:", action)
            print("receive value:", value)

            if value == None:
                if action == "advance":
                    value = os.getenv("ADVANCE_DEFAULT_VALUE")
                elif action == "retreat":
                    value = os.getenv("RETREAT_DEFAULT_VALUE")
                elif action == "left_rotation":
                    value = os.getenv("LEFT_ROTATION_DEFAULT_VALUE")
                elif action == "right_rotation":
                    value = os.getenv("RIGHT_ROTATION_DEFAULT_VALUE")
                elif action == "leg_length":
                    value = os.getenv("LEG_LENGTH_DEFAULT_VALUE")

            print("receive value:", value)

            if value:
                # Call the function and execute the robot instruction
                move_robot(action, value)
                # Send the result of successful function execution
                send_function_call_result("Execution successful", call_id, ws)
            else:
                print("No parameters provided, cannot execute")

    except Exception as e:
        print(f"Error parsing function call parameters: {e}")


def send_function_call_result(result, call_id, ws):
    """
    Send the function call result to the server
    Parameters:
        result: Function execution result (string or serializable object)
        call_id: Corresponding function call ID
        ws: WebSocket connection object
    """
    # Construct the result message structure
    result_json = {
        "type": "conversation.item.create",  # Message type
        "item": {
            "type": "function_call_output",  # Item type
            "output": result,  # Actual result
            "call_id": call_id  # Associated call ID
        }
    }

    try:
        # Send the function call result
        ws.send(json.dumps(result_json))
        print(f"Function call result sent: {result_json}")

        # Construct and send a response creation request (notify the server that it can generate an AI response)
        rp_json = {"type": "response.create"}
        ws.send(json.dumps(rp_json))
        print(f"Response creation request sent: {rp_json}")
    except Exception as e:
        print(f"Failed to send the function call result: {e}")


def move_robot(action, number):
    # Execute the command
    if action == "advance":
        ES02_def_function.advance(number)
    elif action == "retreat":
        ES02_def_function.retreat(number)
    elif action == "left_rotation":
        ES02_def_function.left_rotation(number)
    elif action == "right_rotation":
        ES02_def_function.right_rotation(number)
    elif action == "leg_length":
        ES02_def_function.leg_length(number)

    return


def send_fc_session_update(ws):
    """
    Send session configuration information to the server
    Parameters:
        ws: WebSocket connection object
    """
    # Complete session configuration definition
    session_config = {
        "type": "session.update",  # Message type
        "session": {
            # AI behavior instructions
            # Reference template:
            # 1. What is its role and task? Identify the user's instructions and call appropriate functions to control the robot.
            # 2. What capabilities does it have? It can control robot actions (e.g., turning, moving, etc.).
            # 3. What is its tone and style? Optional: concise and direct, non-redundant.
            # 4. What should it not do? Do not execute non-functional requests, do not engage in small talk or tell jokes.
            "instructions": (
                "You can control the robot to perform actions such as turning left, turning right, moving forward, moving backward, and raising its hand. Express the actions through the move_robot function."
                "Your goal is to understand the user's voice commands and convert them into function calls for the robot's behavior."
                "Do not answer questions unrelated to control, and do not explain the rules."
                "Function calls must accurately match the user's intentions."
                "If the user does not give a clear command, remain silent."
                "Trigger function calls as soon as possible whenever possible."
            ),
            # Voice activity detection configuration
            "turn_detection": {
                "type": "server_vad",  # Use server-side VAD
                "threshold": 0.5,  # Voice detection threshold
                "prefix_padding_ms": 300,  # Prefix silence duration
                "silence_duration_ms": 500  # Silence determination duration
            },
            "voice": "alloy",  # Voice type
            "temperature": 1,  # Response randomness (0-2)
            "max_response_output_tokens": 4096,  # Maximum output token count
            "modalities": ["text", "audio"],  # Interaction modes
            "input_audio_format": "pcm16",  # Input audio format
            "output_audio_format": "pcm16",  # Output audio format
            # Speech recognition configuration
            "input_audio_transcription": {
                "model": "whisper-1"  # Use the Whisper model
            },
            "tool_choice": "auto",  # Tool selection mode
            # List of available tools
            "tools": [
                # Function to control the robot's turning, moving forward, and moving backward
                {
                    "type": "function",
                    "name": "move_robot",
                    "description": "Control the robot's movement, including moving forward, moving backward, turning, and adjusting the leg length. All distance parameters are in meters, angle parameters are in degrees, and leg length parameters are in centimeters.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "action": {
                                "type": "string",
                                "enum": [
                                    "advance",
                                    "retreat",
                                    "left_rotation",
                                    "right_rotation",
                                    "leg_length"
                                ],
                                "description": "Action type: advance (move forward)/retreat (move backward)/left_rotation (turn left)/right_rotation (turn right)/leg_length (adjust leg length)"
                            },
                            "value": {
                                "type": "number",
                                "description": "Action parameter value: distance for moving forward/backward (meters), with a default value of " + os.getenv(
                                    "ADVANCE_DEFAULT_VALUE") + " meters." +
                                               "Or the turning angle (degrees), with a default value of " + os.getenv(
                                    "LEFT_ROTATION_DEFAULT_VALUE") + " degrees." +
                                               "Or the leg length (centimeters), with a default value of " + os.getenv(
                                    "LEG_LENGTH_DEFAULT_VALUE") + " centimeters."
                            }
                        },
                        "required": ["action"]
                    }
                }
            ]
        }
    }

    try:
        # Send the session configuration
        ws.send(json.dumps(session_config))
        print("Session configuration update sent")
    except Exception as e:
        print(f"Failed to send the session configuration: {e}")


def create_connection_with_ipv4(*args, **kwargs):
    """
    Force the use of IPv4 to create a WebSocket connection
    Parameters:
        *args: Variable positional arguments (passed to websocket.create_connection)
        **kwargs: Variable keyword arguments (passed to websocket.create_connection)
    Returns:
        WebSocket connection object
    """
    # Save the original getaddrinfo function
    original_getaddrinfo = socket.getaddrinfo

    def getaddrinfo_ipv4(host, port, family=socket.AF_INET, *args):
        """
        Overridden getaddrinfo function, forcing the use of IPv4
        Parameters:
            host: Target host
            port: Target port
            family: Forced to AF_INET (IPv4)
        """
        return original_getaddrinfo(host, port, socket.AF_INET, *args)

    # Temporarily replace the system getaddrinfo implementation
    socket.getaddrinfo = getaddrinfo_ipv4
    try:
        # Create a WebSocket connection
        return websocket.create_connection(*args, **kwargs)
    finally:
        # Restore the original getaddrinfo function
        socket.getaddrinfo = original_getaddrinfo


def connect_to_openai():
    """
    Establish a connection to the OpenAI WebSocket API and manage communication threads
    """
    ws = None
    try:
        # Create a WebSocket connection with an authentication header
        ws = create_connection_with_ipv4(
            WS_URL,  # WebSocket server URL
            header=[
                f'Authorization: Bearer {API_KEY}',  # API key authentication
                'OpenAI-Beta: realtime=v1'  # Protocol version identifier
            ],
            sslopt={"cert_reqs": ssl.CERT_NONE}  # Disable certificate verification
        )

        print('Successfully connected to the OpenAI WebSocket')

        # Start the receiving thread (process server responses)
        receive_thread = threading.Thread(
            target=receive_audio_from_websocket,
            args=(ws,)
        )
        receive_thread.start()

        # Start the sending thread (send microphone data)
        mic_thread = threading.Thread(
            target=send_mic_audio_to_websocket,
            args=(ws,)
        )
        mic_thread.start()

        # Main loop: wait for the stop event to trigger
        while not stop_event.is_set():
            time.sleep(0.1)  # Reduce CPU usage

        # Graceful shutdown: send a close frame
        print('Sending a WebSocket close frame...')
        ws.send_close()

        # Wait for the threads to end
        receive_thread.join()
        mic_thread.join()

        print('WebSocket connection closed, threads terminated')
    except Exception as e:
        print(f'Failed to connect to OpenAI: {e}')
    finally:
        if ws is not None:
            try:
                ws.close()
                print('WebSocket connection closed')
            except Exception as e:
                print(f'Error closing the WebSocket connection: {e}')


def main():
    """
    Main function: initialize the audio stream and start the core logic
    """
    # Initialize the PyAudio instance
    p = pyaudio.PyAudio()

    # Configure the microphone input stream
    mic_stream = p.open(
        format=FORMAT,  # Audio format (16-bit integer)
        channels=1,  # Mono
        rate=RATE,  # Sampling rate 24kHz
        input=True,  # Input device
        stream_callback=mic_callback,  # Real-time callback function
        frames_per_buffer=CHUNK_SIZE  # 1024 bytes per chunk
    )

    # Configure the speaker output stream
    speaker_stream = p.open(
        format=FORMAT,
        channels=1,
        rate=RATE,
        output=True,  # Output device
        stream_callback=speaker_callback,
        frames_per_buffer=CHUNK_SIZE
    )

    ES02_def_function.init_sbus()
    ES02_def_function.start_ES02_ch_timing_processing_thread()

    try:
        # Start the audio stream
        mic_stream.start_stream()
        speaker_stream.start_stream()

        # Connect to the OpenAI service
        connect_to_openai()

        # Keep the main thread running (detect the audio stream status)
        while mic_stream.is_active() and speaker_stream.is_active():
            time.sleep(0.1)

    except KeyboardInterrupt:
        # Handle Ctrl+C interruption
        print('Gracefully shutting down...')
        stop_event.set()  # Notify all threads to stop

    finally:
        # Clean up audio resources
        mic_stream.stop_stream()
        mic_stream.close()
        speaker_stream.stop_stream()
        speaker_stream.close()

        p.terminate()  # Release PyAudio resources
        print('Audio stream stopped, resources released. Program exited')


if __name__ == '__main__':
    """Program entry point"""
    main()