import gpiod
import requests
import time

CHIP = "gpiochip0"
LINE_OFFSET = 4

chip = gpiod.Chip(CHIP)
line = chip.get_line(LINE_OFFSET)

# Request events with a pull-down bias (adjust as needed for your wiring)
line.request(
    consumer="button-monitor",
    type=gpiod.LINE_REQ_EV_BOTH_EDGES,
    flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_DOWN
)

# Optional: Debounce mechanism to filter rapid consecutive events
debounce_time = 0.5  # 200ms
last_event_time = 0

print("Monitoring GPIO events on pin 4 for rising edge events only...")

while True:
    if line.event_wait(sec=10):
        event = line.event_read()
        current_time = time.monotonic()
        # Debounce: skip events if they occur too quickly in succession
        if (current_time - last_event_time) < debounce_time:
            continue
        last_event_time = current_time

        # Only process rising edge events
        if event.type != gpiod.LineEvent.RISING_EDGE:
            continue

        # For a rising edge, we assume the button is being released
        state = "pressed"
        print(f"Detected rising edge event: {state}")

        try:
            response = requests.post("http://localhost:5000/button", json={"state": state})
            print(f"Event sent, status code: {response.status_code}")
        except Exception as e:
            print("Error sending event:", e)
